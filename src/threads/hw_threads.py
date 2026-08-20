"""Background threads that feed the control loop, plus the perf monitor.

Each of these owns one piece of hardware and publishes its latest reading behind a
lock, so the control loop never blocks on I2C, SPI or the camera.
"""

import re
import subprocess
import threading
import time
from collections import deque

import numpy as np

from src.motors import motor
from src.sensors import distance
from src.logs.setup import (
    Throttle, _fmt, clog, hlog, ilog, perflog, slog,
)
from src.obstacle_challenge.tuning import (
    HEALTH_POLL_PERIOD, HEALTH_TEMP_WARN_C, PERF_REPORT_PERIOD,
    WATCHDOG_POLL_S, WATCHDOG_TIMEOUT_S,
)

# ---------------------------------------------------------------------------
# Sensor / camera threads
# ---------------------------------------------------------------------------

class CameraThread(threading.Thread):
    def __init__(self, camera_instance):
        super().__init__(name="camera")
        self.camera = camera_instance
        self.latest_frame = None
        self.cond = threading.Condition()
        self.stop_event = threading.Event()
        self.daemon = True
        self.frame_counter = 0
        self.capture_time = 0.0     # monotonic timestamp of the latest frame

    def run(self):
        clog.info("Camera thread running.")
        while not self.stop_event.is_set():
            try:
                frame = self.camera.capture_frame()
            except Exception:
                clog.exception("capture_frame failed")
                time.sleep(0.05)
                continue
            now = time.monotonic()
            with self.cond:
                self.frame_counter += 1
                self.latest_frame = frame
                self.capture_time = now
                self.cond.notify_all()
        clog.info("Camera thread stopped.")

    def get_frame(self):
        # Zero-copy handoff -- see get_next_frame() for the immutability contract.
        with self.cond:
            if self.latest_frame is not None:
                return self.latest_frame, self.frame_counter
            return None, self.frame_counter

    def get_next_frame(self, last_counter, timeout=1.0):
        """Block until a frame newer than `last_counter` arrives.

        This is the only way the control loop should fetch frames. The old loop
        called get_frame() and `continue`d when the counter hadn't moved, which
        spun a core flat out and starved this thread -- the main reason the run
        sat at 40-50 fps instead of the camera's 56.

        The returned frame is a ZERO-COPY reference, not a copy: capture_frame()
        (picamera2 capture_array) allocates a fresh array per capture and this
        thread only ever swaps the reference, never writes into an old array, so
        handing the reference out is safe as long as consumers treat frames as
        read-only. They do: process_video_frame() only reads,
        annotate_video_frame() copies before drawing, and
        VideoEncoderProcess.write() copies into its shared-memory slot before
        returning. Anything new that wants to draw on a frame must .copy() first
        (tests/test_vision_pipeline.py pins the annotate invariant). Copying here
        cost a 0.7 MB memcpy per fetch INSIDE the lock, stalling the capture
        thread's notify path.
        """
        with self.cond:
            self.cond.wait_for(
                lambda: self.frame_counter != last_counter or self.stop_event.is_set(),
                timeout=timeout,
            )
            if self.latest_frame is not None:
                return self.latest_frame, self.frame_counter, self.capture_time
            return None, self.frame_counter, 0.0

    def stop(self):
        with self.cond:
            self.stop_event.set()
            self.cond.notify_all()


class ImuThread(threading.Thread):
    def __init__(self, bno, init_event):
        super().__init__(name="imu")
        self.bno = bno
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.heading = None

    def run(self):
        try:
            self.bno.initialize()
            ilog.info("IMU initialized.")
            self.initialization_complete.set()
            while not self.stop_event.is_set():
                heading = self.bno.get_heading()
                if heading is not None:
                    with self.lock:
                        self.heading = heading
                time.sleep(0.01)
        except Exception:
            ilog.exception("ERROR during IMU initialization/operation")
            self.initialization_complete.set()
        finally:
            self.bno.cleanup()
            ilog.info("IMU cleanup complete.")

    def get_heading(self):
        with self.lock:
            return self.heading

    def tare(self, target_angle=0.0):
        return self.bno.tare(target_angle)

    def reset_tare(self):
        return self.bno.reset_tare()

    def lock_calibration(self):
        return self.bno.lock_calibration()

    def unlock_calibration(self):
        return self.bno.unlock_calibration()

    def stop(self):
        self.stop_event.set()


class SensorThread(threading.Thread):
    def __init__(self, dist, init_event):
        super().__init__(name="sensors")
        self.dist = dist
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.distance_left = None
        self.distance_right = None
        self.distance_back = None
        self.distance_center = None
        # Channels are the sensors' XSHUT GPIOs: 17=front, 16=back (both wired),
        # 22=left, 27=right (not wired yet — get_distance() returns None for them
        # until they are, so polling them is harmless).
        self.channels = [
            distance.FRONT_CHANNEL,
            distance.BACK_CHANNEL,
            distance.LEFT_CHANNEL,
            distance.RIGHT_CHANNEL,
        ]
        self.history = {ch: deque(maxlen=4) for ch in self.channels}

    def run(self):
        try:
            for attempt in range(3):
                try:
                    slog.info("Initializing distance sensors (attempt %d/3)...", attempt + 1)
                    self.dist.initialise()
                    slog.info("Distance sensors initialized.")
                    break
                except Exception:
                    slog.exception("ERROR during distance sensor initialization")
            self.initialization_complete.set()
            slog.debug("Initialization complete flag set.")

            consecutive_none = {ch: 0 for ch in self.channels}
            reinit_threshold = 30
            while not self.stop_event.is_set():
                try:
                    readings = {}
                    for ch in list(consecutive_none.keys()):
                        val = self.dist.get_distance(ch)
                        if val is not None and val < 50.0:
                            val = None
                        with self.lock:
                            if val is not None:
                                self.history[ch].append(val)
                                consecutive_none[ch] = 0
                            else:
                                consecutive_none[ch] = consecutive_none.get(ch, 0) + 1

                            if self.history[ch]:
                                sorted_vals = sorted(self.history[ch])
                                if len(sorted_vals) >= 3:
                                    readings[ch] = sum(sorted_vals[1:-1]) / (len(sorted_vals) - 2)
                                else:
                                    readings[ch] = sum(sorted_vals) / len(sorted_vals)
                            else:
                                readings[ch] = None

                    with self.lock:
                        self.distance_center = readings.get(distance.FRONT_CHANNEL)
                        self.distance_back = readings.get(distance.BACK_CHANNEL)
                        self.distance_left = readings.get(distance.LEFT_CHANNEL)
                        self.distance_right = readings.get(distance.RIGHT_CHANNEL)

                    for ch, count in list(consecutive_none.items()):
                        if count >= reinit_threshold:
                            #ok = distance.reinit_sensor(ch)
                            #consecutive_none[ch] = 0 if ok else count
                            pass

                    time.sleep(0.02)  # account for timing budget
                except Exception:
                    slog.exception("ERROR during sensor reading")
                    time.sleep(0.1)

        except Exception:
            slog.exception("ERROR during sensor thread initialization/operation")
            # Still set the event so main thread doesn't hang forever
            self.initialization_complete.set()

        finally:
            slog.info("Cleaning up distance sensors...")
            self.dist.cleanup()
            slog.info("Distance sensor cleanup complete.")

    def get_readings(self):
        with self.lock:
            return {
                'distance_left': self.distance_left,
                'distance_center': self.distance_center,
                'distance_right': self.distance_right,
                'distance_back': self.distance_back
            }

    def get_history(self, ch):
        with self.lock:
            return list(self.history[ch])

    def stop(self):
        self.stop_event.set()

class Heartbeat:
    """A timestamp the control loop pets and a watchdog thread reads.

    Plain attribute get/set on a CPython object is atomic under the GIL, so this
    needs no lock: one thread writes `.ts`, another only ever reads it.
    """

    def __init__(self):
        self.ts = time.monotonic()

    def pet(self):
        self.ts = time.monotonic()

    def age(self):
        return time.monotonic() - self.ts


class WatchdogThread(threading.Thread):
    """Dead-man's switch: cuts motor power if the control loop stops petting.

    This is the piece that can act *during* a stall, not just clean up after one --
    it runs in its own thread, so it keeps polling even while the main loop is stuck
    inside a single blocking call (CPython releases the GIL around blocking I/O, so
    this thread still gets scheduled). A same-thread check in the control loop
    (`skipped >= LOOP_STALL_SKIP_THRESHOLD` in main.py) cannot do this: it only runs
    once the stuck call has already returned, by which point the stall is over.

    Added after the 2026-08-19_20-50-33 run stalled ~1.3s (74 frames) for a cause the
    per-frame logs couldn't pin down.
    """

    def __init__(self, heartbeat, timeout=WATCHDOG_TIMEOUT_S, poll=WATCHDOG_POLL_S):
        super().__init__(name="watchdog")
        self.heartbeat = heartbeat
        self.timeout = timeout
        self.poll = poll
        self.stop_event = threading.Event()
        self.daemon = True
        self.tripped = False

    def run(self):
        hlog.info("Watchdog thread running (timeout %.2fs, poll %.2fs).",
                   self.timeout, self.poll)
        while not self.stop_event.wait(self.poll):
            age = self.heartbeat.age()
            if age > self.timeout:
                if not self.tripped:
                    hlog.critical(
                        "WATCHDOG: control loop heartbeat stale for %.2fs (> %.2fs) -- "
                        "cutting motor power now, main loop is unresponsive.",
                        age, self.timeout)
                    try:
                        motor.stop_rpm_control()
                    except Exception:
                        hlog.exception("Watchdog failed to stop motor")
                    self.tripped = True
            elif self.tripped:
                hlog.warning("WATCHDOG: heartbeat resumed after %.2fs stale.", age)
                self.tripped = False

    def stop(self):
        self.stop_event.set()


_THROTTLE_BITS = {
    0: "under-voltage now", 1: "arm-freq-capped now", 2: "throttled now", 3: "soft-temp-limit now",
    16: "under-voltage occurred", 17: "arm-freq-capped occurred",
    18: "throttled occurred", 19: "soft-temp-limit occurred",
}


class SystemHealthThread(threading.Thread):
    """Polls `vcgencmd` for under-voltage / thermal throttling, off the control loop.

    Added after a run (2026-08-19_20-50-33) showed a ~1.3s control-loop stall with no
    clear cause in the per-frame logs. Runs `vcgencmd` in a subprocess with a timeout
    so a hung/slow vcgencmd call can never itself block steering -- exactly the
    mistake this thread exists to help rule out for other subsystems.
    """

    def __init__(self, period=HEALTH_POLL_PERIOD, temp_warn_c=HEALTH_TEMP_WARN_C):
        super().__init__(name="health")
        self.period = period
        self.temp_warn_c = temp_warn_c
        self.stop_event = threading.Event()
        self.daemon = True
        self.last_throttled = 0
        self._clear_log_throttle = Throttle(60.0)

    @staticmethod
    def _vcgencmd(*args, timeout=1.0):
        try:
            out = subprocess.run(["vcgencmd", *args], capture_output=True,
                                  text=True, timeout=timeout, check=False).stdout
            return out.strip()
        except Exception:
            return None

    def run(self):
        hlog.info("Health thread running (poll every %.1fs).", self.period)
        while not self.stop_event.wait(self.period):
            raw = self._vcgencmd("get_throttled")
            temp_raw = self._vcgencmd("measure_temp")
            throttled = None
            if raw and "=" in raw:
                try:
                    throttled = int(raw.split("=", 1)[1], 0)
                except ValueError:
                    pass
            temp_c = None
            if temp_raw:
                m = re.search(r"[\d.]+", temp_raw)
                if m:
                    temp_c = float(m.group())

            if throttled is not None and throttled != 0:
                active = [msg for bit, msg in _THROTTLE_BITS.items() if throttled & (1 << bit)]
                hlog.warning("vcgencmd throttled=0x%x (%s) temp=%s", throttled,
                              ", ".join(active) or "unknown bits", temp_raw or "?")
            elif temp_c is not None and temp_c >= self.temp_warn_c:
                hlog.warning("CPU temp %.1fC (throttled=0x%s)", temp_c,
                              format(throttled, "x") if throttled is not None else "?")
            elif self._clear_log_throttle:
                hlog.debug("throttled=0x%s temp=%s", format(throttled, "x") if throttled is not None else "?",
                           temp_raw or "?")
            self.last_throttled = throttled or 0

    def stop(self):
        self.stop_event.set()


class PerfMonitor:
    """Rolling FPS and latency stats, reported at INFO every PERF_REPORT_PERIOD.

    `latency` is the number that actually matters: camera capture -> servo command.
    """

    def __init__(self, period=PERF_REPORT_PERIOD):
        self.period = period
        self.reset()
        self._next = time.monotonic() + period

    def reset(self):
        self.lat = []
        self.proc = []
        self.frames = 0
        self.skipped = 0
        self._t0 = time.monotonic()

    def add(self, latency_ms, proc_ms, skipped=0):
        self.lat.append(latency_ms)
        self.proc.append(proc_ms)
        self.frames += 1
        self.skipped += skipped

    def maybe_report(self, extra=""):
        now = time.monotonic()
        if now < self._next or not self.lat:
            return
        dt = now - self._t0
        lat = np.array(self.lat)
        proc = np.array(self.proc)
        perflog.info(
            "%5.1f fps | capture->servo p50 %5.2f p95 %5.2f max %5.2f ms | "
            "vision p50 %4.2f ms | skipped %d%s",
            self.frames / dt,
            np.percentile(lat, 50), np.percentile(lat, 95), lat.max(),
            np.percentile(proc, 50), self.skipped, extra,
        )
        self.reset()
        self._next = now + self.period
