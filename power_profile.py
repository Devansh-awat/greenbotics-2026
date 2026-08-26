#!/usr/bin/env python3
"""
Power / thermal / resource profiler for the Greenbotics Pi 5.

Runs two phases and, for each, samples `vcgencmd pmic_read_adc` (per-rail
current / voltage -> power), CPU temperature, per-core-aggregate CPU usage,
RAM usage, ARM clock and throttling flags over the sampling window.

Phases
------
  A  idle_baseline   Nothing initialized. Just the OS at rest.
  B  full_processing Faithful replica of the main_v2 runtime: camera thread,
                     sensor thread (both ToF ranging), IMU thread, video-writer
                     thread, and the per-frame image-processing loop
                     (process_video_frame + annotate + encode) at the same
                     ~60 fps cadence. Motor + servo are initialized (as main_v2
                     does) but the DRIVE MOTOR IS NEVER COMMANDED -- the robot
                     does not move.

For each phase it prints the full averaged `pmic_read_adc` dump plus:
  - Total power (sum of all PMIC rails)
  - Onboard power (total minus the 3V3_SYS rail = Pi SoC/RAM/etc.)
  - 3V3_SYS rail power (feeds the camera + I2C sensors), with current & voltage
and a resource table (CPU%, temp, clock, RAM) with avg + min/max over 15 s.

Run from the repo root:
    python3 power_profile.py                  # default 4s settle, 15s sample
    python3 power_profile.py --settle 3 --sample 10
    python3 power_profile.py --skip-full      # only the idle phase

Nothing here moves the robot. Ctrl-C aborts cleanly.
"""

import argparse
import re
import subprocess
import sys
import threading
import time
import traceback

# The 3V3 system rail we break out separately (camera + I2C sensors ride on it).
PERIPH_RAIL = "3V3_SYS"

# ---------------------------------------------------------------------------
# Low-level metric collection
# ---------------------------------------------------------------------------

# Matches lines like "   VDD_CORE_A current(7)=3.25052000A"
#                and "   VDD_CORE_V volt(15)=0.88349120V"
_PMIC_LINE_RE = re.compile(r"^\s*(\S+?)_([AV])\s+(\w+)\((\d+)\)=([-\d.]+)([AV])\s*$")


def _vcgencmd(*args):
    try:
        return subprocess.run(
            ["vcgencmd", *args],
            capture_output=True, text=True, timeout=3,
        ).stdout
    except Exception:
        return ""


def read_pmic():
    """Parse pmic_read_adc.

    Returns (total_power_W, powers{base:W}, amps{base:A}, volts{base:V},
             channels[(base,kind,func,idx,val,unit)], raw_text).
    """
    raw = _vcgencmd("pmic_read_adc")
    amps, volts, channels = {}, {}, []
    for line in raw.splitlines():
        m = _PMIC_LINE_RE.match(line)
        if not m:
            continue
        base, kind, func, idx, val, unit = m.groups()
        val = float(val)
        channels.append((base, kind, func, int(idx), val, unit))
        (amps if kind == "A" else volts)[base] = val
    powers, total = {}, 0.0
    for base, i in amps.items():
        v = volts.get(base)
        if v is None:
            continue
        p = i * v
        powers[base] = p
        total += p
    return total, powers, amps, volts, channels, raw


def read_temp():
    m = re.search(r"temp=([\d.]+)", _vcgencmd("measure_temp"))
    return float(m.group(1)) if m else None


def read_arm_clock_mhz():
    m = re.search(r"=(\d+)", _vcgencmd("measure_clock", "arm"))
    return int(m.group(1)) / 1e6 if m else None


def read_throttled():
    m = re.search(r"=(0x[0-9a-fA-F]+)", _vcgencmd("get_throttled"))
    return m.group(1) if m else "?"


def read_cpu_times():
    """Aggregate jiffies from /proc/stat 'cpu' line: (idle, total)."""
    with open("/proc/stat") as f:
        parts = f.readline().split()
    vals = [int(x) for x in parts[1:]]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)  # idle + iowait
    return idle, sum(vals)


def read_mem_used_mb():
    total = avail = None
    with open("/proc/meminfo") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                total = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                avail = int(line.split()[1])
            if total is not None and avail is not None:
                break
    if total is None or avail is None:
        return None, None
    return (total - avail) / 1024.0, total / 1024.0


# ---------------------------------------------------------------------------
# Sampler
# ---------------------------------------------------------------------------

class Stat:
    def __init__(self):
        self.vals = []

    def add(self, v):
        if v is not None:
            self.vals.append(v)

    def avg(self):
        return sum(self.vals) / len(self.vals) if self.vals else None

    def mn(self):
        return min(self.vals) if self.vals else None

    def mx(self):
        return max(self.vals) if self.vals else None


def sample_phase(name, settle, sample, interval=0.5):
    """Sample metrics for `sample` seconds after a `settle` warm-up."""
    print(f"\n[{name}] settling {settle}s ...", flush=True)
    time.sleep(settle)
    print(f"[{name}] sampling {sample}s ...", flush=True)

    power, temp, cpu, mem, clock = Stat(), Stat(), Stat(), Stat(), Stat()
    rail_i, rail_v, rail_p = {}, {}, {}
    throttled_seen = set()
    mem_total_mb = None
    channel_meta = {}          # base -> (a_idx, v_idx) for ordered dump
    raw_snapshot = ""

    prev_idle, prev_total = read_cpu_times()
    t_end = time.monotonic() + sample
    mid = time.monotonic() + sample / 2
    took_snapshot = False
    while time.monotonic() < t_end:
        time.sleep(interval)
        idle, tot = read_cpu_times()
        d_idle, d_tot = idle - prev_idle, tot - prev_total
        prev_idle, prev_total = idle, tot
        if d_tot > 0:
            cpu.add(100.0 * (1.0 - d_idle / d_tot))

        p_total, powers, amps, volts, channels, raw = read_pmic()
        power.add(p_total)
        for b, val in amps.items():
            rail_i.setdefault(b, Stat()).add(val)
        for b, val in volts.items():
            rail_v.setdefault(b, Stat()).add(val)
        for b, val in powers.items():
            rail_p.setdefault(b, Stat()).add(val)
        for base, kind, func, idx, val, unit in channels:
            meta = channel_meta.setdefault(base, {"a": None, "v": None})
            meta["a" if kind == "A" else "v"] = idx

        # grab one representative raw dump near the middle of the window
        if not took_snapshot and time.monotonic() >= mid:
            raw_snapshot = raw
            took_snapshot = True

        temp.add(read_temp())
        clock.add(read_arm_clock_mhz())
        throttled_seen.add(read_throttled())
        used, total_mb = read_mem_used_mb()
        mem.add(used)
        mem_total_mb = total_mb

    if not raw_snapshot:
        raw_snapshot = raw

    return {
        "name": name, "n": len(power.vals),
        "power": power, "temp": temp, "cpu": cpu, "mem": mem, "clock": clock,
        "rail_i": rail_i, "rail_v": rail_v, "rail_p": rail_p,
        "channel_meta": channel_meta, "throttled": throttled_seen,
        "mem_total_mb": mem_total_mb, "raw_snapshot": raw_snapshot,
    }


# ---------------------------------------------------------------------------
# Phase B worker: replicate the main_v2 per-frame processing load (no motion)
# ---------------------------------------------------------------------------

def run_full_processing(stop_event):
    """Start all the main_v2 threads and run its image-processing loop.

    Mirrors the hot path of main_v2's main loop (camera capture -> sensor +
    IMU reads -> process_video_frame -> annotate -> video encode -> 60 fps
    cap). The drive motor is initialized but NEVER commanded to move.
    """
    from src.obstacle_challenge import main_v2
    from src.sensors import bno055, camera, distance
    from src.motors import motor, servo
    import cv2
    import os

    scratch = os.environ.get("SCRATCH_DIR", "/tmp")
    video_path = os.path.join(scratch, "power_profile_dummy.mp4")

    camera_thread = sensor_thread = imu_thread = video_writer_thread = None
    motor_ok = servo_ok = False
    try:
        print("[full] initializing camera / motor / servo ...", flush=True)
        camera.initialize()
        motor_ok = motor.initialize()   # ends in standby(), duty 0 -> no motion
        servo_ok = servo.initialize()
        if servo_ok:
            servo.set_angle(0)          # center wheels; does not drive the robot

        fourcc = cv2.VideoWriter_fourcc(*"avc1")
        video_writer_thread = main_v2.VideoWriterThread(video_path, fourcc, 10, (640, 360))
        video_writer_thread.start()

        camera_thread = main_v2.CameraThread(camera)
        camera_thread.start()

        sensors_evt = threading.Event()
        sensor_thread = main_v2.SensorThread(distance, sensors_evt)
        sensor_thread.start()
        print("[full] waiting for ToF sensors ...", flush=True)
        sensors_evt.wait(timeout=15)

        imu_evt = threading.Event()
        imu_thread = main_v2.ImuThread(bno055, imu_evt)
        imu_thread.start()
        print("[full] waiting for IMU ...", flush=True)
        imu_evt.wait(timeout=15)

        print("[full] processing loop running (robot stationary) ...", flush=True)
        past_counter = 0
        frame_start = time.perf_counter()
        while not stop_event.is_set():
            got = camera_thread.get_frame()
            if got is None:
                time.sleep(0.005)
                continue
            frame, counter = got
            if counter == past_counter:
                time.sleep(0.001)
                continue
            past_counter = counter

            # same reads main_v2 does every frame
            _ = sensor_thread.get_readings()
            _ = imu_thread.get_heading()

            detections = main_v2.process_video_frame(frame)
            annotated = main_v2.annotate_video_frame(
                frame, detections, "clockwise", debug_info="power_profile"
            )
            try:
                video_writer_thread.write(annotated)
            except Exception:
                pass

            # NOTE: deliberately NO motor.forward()/servo.set_angle() here.

            elapsed = time.perf_counter() - frame_start
            if elapsed < 1 / 60:
                time.sleep(1 / 60 - elapsed)
            frame_start = time.perf_counter()

    except Exception:
        print("[full] ERROR in processing worker:", flush=True)
        traceback.print_exc()
    finally:
        print("[full] tearing down threads ...", flush=True)
        for t in (camera_thread, sensor_thread, imu_thread, video_writer_thread):
            try:
                if t is not None:
                    t.stop()
            except Exception:
                pass
        for t in (camera_thread, sensor_thread, imu_thread, video_writer_thread):
            try:
                if t is not None:
                    t.join(timeout=5)
            except Exception:
                pass
        try:
            camera.cleanup()
        except Exception:
            pass
        try:
            if servo_ok:
                servo.cleanup()
        except Exception:
            pass
        try:
            if motor_ok:
                motor.brake()
                motor.cleanup()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------

def _f(v, p=3):
    return f"{v:.{p}f}" if v is not None else "--"


def print_pmic_dump(res):
    """Print the full pmic_read_adc picture for a phase: a representative raw
    snapshot plus the per-channel averages over the whole window."""
    print(f"\n{'#' * 78}\n# pmic_read_adc -- {res['name']}  (avg over {res['n']} samples / 15 s)\n{'#' * 78}")

    print(f"\n--- representative raw snapshot ({res['name']}) ---")
    print(res["raw_snapshot"].rstrip())

    print(f"\n--- averaged over the sampling window ({res['name']}) ---")
    meta = res["channel_meta"]
    # order by channel index (current channel if present, else voltage channel)
    order = sorted(meta, key=lambda b: (meta[b]["a"] if meta[b]["a"] is not None
                                        else meta[b]["v"]))
    print(f"{'rail':<14}{'current(A)':>14}{'voltage(V)':>14}{'power(W)':>12}")
    print("-" * 54)
    for base in order:
        i = res["rail_i"].get(base, Stat()).avg()
        v = res["rail_v"].get(base, Stat()).avg()
        p = res["rail_p"].get(base, Stat()).avg()
        print(f"{base:<14}{_f(i,5):>14}{_f(v,5):>14}{_f(p,4):>12}")
    print("-" * 54)
    print(f"{'TOTAL':<14}{'':>14}{'':>14}{_f(res['power'].avg(),4):>12}")


def print_summary(results):
    names = [r["name"] for r in results]
    W = 18

    print("\n" + "=" * 78)
    print("SUMMARY  (all values are averages over the 15 s sampling window)")
    print("=" * 78)

    hdr = f"{'metric':<30}" + "".join(f"{n:>{W}}" for n in names)
    print(hdr)
    print("-" * len(hdr))

    def row(label, fn):
        print(f"{label:<30}" + "".join(f"{fn(r):>{W}}" for r in results))

    # --- power / current / voltage ---
    def total_p(r):
        return f"{_f(r['power'].avg(),3)} W"

    def periph_p(r):
        return f"{_f(r['rail_p'].get(PERIPH_RAIL, Stat()).avg(),3)} W"

    def periph_i(r):
        return f"{_f(r['rail_i'].get(PERIPH_RAIL, Stat()).avg(),4)} A"

    def periph_v(r):
        return f"{_f(r['rail_v'].get(PERIPH_RAIL, Stat()).avg(),4)} V"

    def onboard_p(r):
        t = r["power"].avg()
        pr = r["rail_p"].get(PERIPH_RAIL, Stat()).avg()
        return f"{_f(t - pr, 3)} W" if (t is not None and pr is not None) else "--"

    row("Total power (all rails)", total_p)
    row("  Onboard power (total - 3V3)", onboard_p)
    row(f"  {PERIPH_RAIL} rail power", periph_p)
    row(f"  {PERIPH_RAIL} rail current", periph_i)
    row(f"  {PERIPH_RAIL} rail voltage", periph_v)

    print("-" * len(hdr))
    # --- resource metrics: avg + min/max ---
    row("CPU usage avg", lambda r: f"{_f(r['cpu'].avg(),1)} %")
    row("  CPU usage min / max", lambda r: f"{_f(r['cpu'].mn(),1)} / {_f(r['cpu'].mx(),1)} %")
    row("CPU temp avg", lambda r: f"{_f(r['temp'].avg(),1)} C")
    row("  CPU temp min / max", lambda r: f"{_f(r['temp'].mn(),1)} / {_f(r['temp'].mx(),1)} C")
    row("ARM clock avg", lambda r: f"{_f(r['clock'].avg(),0)} MHz")
    row("  ARM clock min / max", lambda r: f"{_f(r['clock'].mn(),0)} / {_f(r['clock'].mx(),0)} MHz")
    row("RAM used avg", lambda r: f"{_f(r['mem'].avg(),0)} MB")
    row("  RAM used min / max", lambda r: f"{_f(r['mem'].mn(),0)} / {_f(r['mem'].mx(),0)} MB")
    row("throttled flags", lambda r: ",".join(sorted(r["throttled"])) or "?")
    print("-" * len(hdr))

    if len(results) == 2:
        a, b = results
        dp = b["power"].avg() - a["power"].avg()
        dc = b["cpu"].avg() - a["cpu"].avg()
        dt = b["temp"].avg() - a["temp"].avg()
        d3 = (b["rail_p"].get(PERIPH_RAIL, Stat()).avg()
              - a["rail_p"].get(PERIPH_RAIL, Stat()).avg())
        print(f"\nDelta ({b['name']} - {a['name']}):")
        print(f"  total power   {dp:+.3f} W")
        print(f"  3V3 rail      {d3:+.3f} W")
        print(f"  CPU usage     {dc:+.1f} %")
        print(f"  CPU temp      {dt:+.1f} C")

    print("\nNotes:")
    print("  * PMIC power = sum of regulator-rail outputs (I*V) from")
    print("    `vcgencmd pmic_read_adc` -- on-board rail output, not battery/5V")
    print("    input (which is higher by converter losses; EXT5V ~5.16 V).")
    print("  * Onboard power = Total minus the 3V3_SYS rail; 3V3_SYS feeds the")
    print("    camera + I2C ToF/IMU sensors, so it is the peripheral share.")
    print(f"  * RAM total = {results[0]['mem_total_mb']:.0f} MB")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--settle", type=float, default=4.0, help="warm-up seconds per phase")
    ap.add_argument("--sample", type=float, default=15.0, help="sampling seconds per phase")
    ap.add_argument("--skip-full", action="store_true", help="skip the full-processing phase")
    args = ap.parse_args()

    results = []

    # --- Phase A: idle baseline -------------------------------------------
    results.append(sample_phase("idle_baseline", args.settle, args.sample))

    # --- Phase B: full image-processing load (no motion) ------------------
    if not args.skip_full:
        stop_event = threading.Event()
        worker = threading.Thread(target=run_full_processing, args=(stop_event,), daemon=True)
        worker.start()
        time.sleep(6)   # let hardware come up before the sampled window
        results.append(sample_phase("full_processing", args.settle, args.sample))
        stop_event.set()
        worker.join(timeout=20)

    # --- reporting --------------------------------------------------------
    for r in results:
        print_pmic_dump(r)
    print_summary(results)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted by user.", file=sys.stderr)
