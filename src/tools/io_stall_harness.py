"""Reproduce the mystery control-loop stall on a stationary (elevated) robot.

Roughly 1 obstacle run in 50 loses 1.3-1.8s inside a single control-loop iteration
(75+ camera frames skipped). It is not vision (VisionPool self-times-out at 100ms and
never logged a miss), not a CLOSE BLOCK maneuver, not thermal/undervolt (vcgencmd is
clean), and not OOM (2GB+ free, zram swap untouched). The camera keeps capturing
right through it, so only the main thread is affected.

This harness runs EVERY subsystem the real run does, at the same rates, so the stall
has the same chance to appear -- but it never navigates, so the robot can stay
elevated on a stand:

    * VisionPool (2 worker processes) + process_video_frame() on every frame
    * both avc1 encoder processes (raw + parking), exactly as main.py starts them
    * CameraThread at 56 fps, SensorThread (ToF), ImuThread (gyro)
    * servo PWM write every frame (sysfs) -- swept by a sine, NOT by the gyro
    * motor rpm control writing PWM continuously -- wheels spin in the air
    * the overlay .jsonl write, which is the thing under test

What it does NOT do: perform_initial_maneuver(), read the gyro to decide anything, or
steer toward blocks. The IMU is read every frame (so the I2C traffic is real) and then
thrown away. Nothing here depends on the robot rotating, so an elevated robot behaves
identically to one on the floor.

The A/B:

    --overlay buffered   OverlayLog as it ships now: append() only appends to a list,
                         the json.dumps()+write happens once in close().
    --overlay sync       the previous behaviour: json.dumps()+fh.write() inline on the
                         control loop, every recorded frame.

Run both for the same duration and compare stall counts. A stall is a loop iteration
that skipped >= --stall-frames camera frames with no maneuver to explain it.

    python3 -m src.tools.io_stall_harness --minutes 20 --overlay sync
    python3 -m src.tools.io_stall_harness --minutes 20 --overlay buffered

Add --probe to also run stall_probe.py against this process, which records whether the
main thread was in D (disk wait) or R (CPU starved) at the moment it stalled.

SAFETY: the drive motor spins for the whole run. Elevate the robot first, or pass
--no-motor.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime

import cv2

from src.motors import motor, servo
from src.sensors import bno086, camera, distance
from src.logs.setup import log, setup_logging, shutdown_logging, Throttle
from src.obstacle_challenge.tuning import *          # noqa: F401,F403
from src.obstacle_challenge.video import OverlayLog, VideoEncoderProcess
from src.threads.hw_threads import (
    CameraThread, ImuThread, PerfMonitor, SensorThread,
)
from src.vision import pipeline as vision
from src.vision.pipeline import detections_to_record, process_video_frame
from src.vision.pool import VisionPool


class SyncOverlayLog(OverlayLog):
    """OverlayLog as it behaved BEFORE the buffering change -- the A-side of the A/B.

    json.dumps() + fh.write() run inline on the control loop for every recorded
    frame, which is the synchronous disk I/O we suspect of parking the main thread.
    """

    def __init__(self, video_path):
        super().__init__(video_path)
        self._records = None          # unused in sync mode; catch accidental reads
        self._fh = None
        try:
            self._fh = open(self.path, "w")
        except Exception:
            log.exception("SyncOverlayLog: could not open %s", self.path)

    def append(self, video_frame, driving_direction, debug_info,
               visual_target_x=None, visual_target_line=None, detections=None):
        if self._fh is None:
            return
        rec = {"v": video_frame, "dir": driving_direction, "debug": list(debug_info)}
        if visual_target_x is not None:
            rec["tx"] = int(visual_target_x)
        if visual_target_line is not None:
            rec["tl"] = visual_target_line
        if detections is not None:
            rec["det"] = detections_to_record(detections)
        try:
            self._fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self.written += 1
        except Exception:
            log.exception("SyncOverlayLog: write failed")
            self._fh = None

    def append_passthrough(self, video_frame):
        if self._fh is None:
            return
        try:
            self._fh.write(json.dumps({"v": video_frame, "pre": True},
                                      separators=(",", ":")) + "\n")
            self.written += 1
        except Exception:
            log.exception("SyncOverlayLog: write failed")
            self._fh = None

    def close(self):
        if self._fh is None:
            return
        try:
            self._fh.close()
        except Exception:
            log.exception("SyncOverlayLog: close failed")
        self._fh = None
        log.info("Overlay log written: %s (%d frames)", self.path, self.written)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--minutes", type=float, default=15.0,
                    help="how long to run the loop (default 15)")
    ap.add_argument("--overlay", choices=("buffered", "sync"), default="buffered",
                    help="overlay-log write mode under test (default buffered)")
    ap.add_argument("--rpm", type=float, default=308.0,
                    help="wheel rpm to hold; matches the real runs (default 308)")
    ap.add_argument("--no-motor", action="store_true",
                    help="do not spin the drive motor (still writes servo PWM)")
    ap.add_argument("--no-servo", action="store_true", help="do not write servo PWM")
    ap.add_argument("--stall-frames", type=int, default=15,
                    help="skipped-frame count that counts as a stall (default 15)")
    ap.add_argument("--warmup-s", type=float, default=3.0,
                    help="ignore stalls in the first N seconds. The camera free-runs "
                         "while the loop starts, so the first iteration always reports "
                         "a large skip -- the real runs show the same thing as the "
                         "skip=4792 entry on their first logged frame (default 3)")
    ap.add_argument("--probe", action="store_true",
                    help="also run stall_probe.py against this process")
    ap.add_argument("--every-n", type=int, default=None,
                    help=f"record every Nth frame (default {VIDEO_EVERY_N})")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = os.path.join("iostall", f"{stamp}_{args.overlay}")
    os.makedirs(run_folder, exist_ok=True)
    raw_video_path = os.path.join(run_folder, "raw.mp4")
    setup_logging(os.path.join(run_folder, "harness_output.txt"))

    log.info("=== IO stall harness | %s | overlay=%s | %.1f min ===",
             stamp, args.overlay, args.minutes)
    log.info("Folder: %s", run_folder)

    # ---- child processes FIRST (fork safety -- see main.py) ----------------
    vision_pool = None
    if USE_VISION_POOL:
        vision_pool = VisionPool()
        vision_pool.start()
        vision.vision_pool = vision_pool
        cv2.setNumThreads(2)
    else:
        cv2.setNumThreads(4)

    video_encoder = VideoEncoderProcess(raw_video_path, VIDEO_FOURCC,
                                        every_n=args.every_n)
    video_encoder.start()
    overlay_cls = SyncOverlayLog if args.overlay == "sync" else OverlayLog
    overlay_log = overlay_cls(raw_video_path)
    log.info("Overlay mode: %s (%s)", args.overlay, overlay_cls.__name__)

    # The real run always starts a second, mostly-idle encoder for parking footage;
    # it contributes fork/CPU/file-descriptor load, so the harness starts one too.
    parking_encoder = VideoEncoderProcess(
        os.path.join(run_folder, "parking.mp4"), VIDEO_FOURCC, every_n=args.every_n)
    parking_encoder.start()

    # ---- hardware -----------------------------------------------------------
    if not camera.initialize():
        log.error("Camera failed to initialize (in use by another process?). Aborting.")
        for enc in (video_encoder, parking_encoder):
            try:
                enc.stop()
            except Exception:
                pass
        overlay_log.close()
        if vision_pool:
            vision_pool.stop()
        shutdown_logging()
        sys.exit(1)

    motor.initialize()
    servo.initialize()

    camera_thread = CameraThread(camera)
    camera_thread.start()
    sensors_ready = threading.Event()
    sensor_thread = SensorThread(distance, sensors_ready)
    sensor_thread.start()
    imu_ready = threading.Event()
    imu_thread = ImuThread(bno086, imu_ready)
    imu_thread.start()

    log.info("Waiting for sensors and IMU...")
    sensors_ready.wait()
    imu_ready.wait()
    log.info("Sensors and IMU ready.")

    probe_proc = None
    if args.probe:
        probe_out = os.path.join(run_folder, "stall_probe.log")
        probe_proc = subprocess.Popen(
            [sys.executable, "-m", "src.tools.stall_probe",
             "--pid", str(os.getpid()), "--out", probe_out, "--stall-ms", "150"],
            stdout=open(os.path.join(run_folder, "stall_probe_summary.txt"), "w"),
            stderr=subprocess.STDOUT)
        log.info("stall_probe running (pid %d) -> %s", probe_proc.pid, probe_out)

    perf = PerfMonitor()
    throttle = Throttle(1.0)
    stalls = []
    past_frame_counter = 0
    loop_frames = 0
    deadline = time.monotonic() + args.minutes * 60.0
    t_start = time.monotonic()

    if not args.no_motor:
        log.warning("Spinning drive motor at %.0f rpm -- robot MUST be elevated.",
                    args.rpm)
        motor.start_rpm_control(args.rpm, "forward")

    log.info("Entering harness loop for %.1f minutes.", args.minutes)
    try:
        while time.monotonic() < deadline:
            frame, frame_counter, capture_ts, fresh = camera_thread.get_next_frame(
                past_frame_counter)
            if frame is None or not fresh:
                continue
            skipped = max(0, frame_counter - past_frame_counter - 1)
            past_frame_counter = frame_counter
            loop_frames += 1

            # Same reads the control loop does -- the IMU value is deliberately
            # discarded, so nothing here needs the robot to actually rotate.
            sensor_thread.get_readings()
            imu_thread.get_heading()

            t_proc = time.perf_counter()
            detections = process_video_frame(frame)
            proc_ms = (time.perf_counter() - t_proc) * 1000.0

            # Servo swept by a sine instead of by a steering decision: same PWM sysfs
            # write, same rate, no dependence on heading or detections.
            if not args.no_servo:
                servo.set_angle(30.0 * math.sin((time.monotonic() - t_start) * 1.5))

            latency_ms = (time.monotonic() - capture_ts) * 1000.0

            if skipped >= args.stall_frames and (time.monotonic() - t_start) > args.warmup_s:
                stalls.append({
                    "t": round(time.monotonic() - t_start, 3),
                    "frame": frame_counter,
                    "skipped": skipped,
                    "stall_ms": round(skipped / CAMERA_FPS * 1000.0, 1),
                    "vis_ms": round(proc_ms, 2),
                    "lat_ms": round(latency_ms, 2),
                })
                log.error("STALL: %d frames skipped (~%.0f ms) at f=%d "
                          "(vis=%.1fms lat=%.1fms)", skipped,
                          skipped / CAMERA_FPS * 1000.0, frame_counter,
                          proc_ms, latency_ms)

            perf.add(latency_ms, proc_ms, skipped)
            perf.maybe_report()

            debug = [f"F:{frame_counter}", f"Lat:{int(latency_ms)}ms", "Harness"]
            if throttle:
                log.debug("f=%d lat=%5.2fms vis=%4.2fms blocks=%d walls=%d skip=%d",
                          frame_counter, latency_ms, proc_ms,
                          len(detections.get("detected_blocks", [])),
                          len(detections.get("detected_walls", [])), skipped)

            if video_encoder.write(frame, tag=frame_counter):
                overlay_log.append(len(video_encoder.tags) - 1, "clockwise", debug,
                                   None, None, detections)
    except KeyboardInterrupt:
        log.info("Interrupted -- shutting down.")
    finally:
        elapsed = time.monotonic() - t_start
        try:
            motor.stop_rpm_control()
            motor.brake()
            motor.cleanup()
        except Exception:
            log.exception("Motor cleanup failed")
        try:
            servo.set_angle(0)
            servo.cleanup()
        except Exception:
            log.exception("Servo cleanup failed")

        camera_thread.stop()
        sensor_thread.stop()
        imu_thread.stop()
        camera_thread.join(timeout=5)
        sensor_thread.join(timeout=5)
        imu_thread.join(timeout=5)

        for enc in (video_encoder, parking_encoder):
            try:
                enc.stop()
            except Exception:
                log.exception("Encoder stop failed")
        overlay_log.close()
        if vision_pool:
            try:
                vision_pool.stop()
            except Exception:
                log.exception("VisionPool stop failed")

        if probe_proc:
            probe_proc.terminate()
            try:
                probe_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                probe_proc.kill()

        summary = {
            "overlay_mode": args.overlay,
            "elapsed_s": round(elapsed, 1),
            "loop_frames": loop_frames,
            "mean_fps": round(loop_frames / elapsed, 1) if elapsed else 0,
            "stall_threshold_frames": args.stall_frames,
            "stall_count": len(stalls),
            "stalls": stalls,
        }
        with open(os.path.join(run_folder, "summary.json"), "w") as fh:
            json.dump(summary, fh, indent=2)

        log.info("=== HARNESS DONE | overlay=%s | %.1f s | %d frames | %.1f fps | "
                 "%d stalls ===", args.overlay, elapsed, loop_frames,
                 summary["mean_fps"], len(stalls))
        for s in stalls:
            log.info("  stall t=%.1fs f=%d skipped=%d (~%.0f ms)",
                     s["t"], s["frame"], s["skipped"], s["stall_ms"])
        shutdown_logging()

        print(f"\noverlay={args.overlay}  {elapsed:.0f}s  {loop_frames} frames  "
              f"{summary['mean_fps']:.1f} fps  ->  {len(stalls)} stalls")
        print(f"summary: {os.path.join(run_folder, 'summary.json')}")


if __name__ == "__main__":
    main()
