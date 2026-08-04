"""Background threads that feed the control loop, plus the perf monitor.

Each of these owns one piece of hardware and publishes its latest reading behind a
lock, so the control loop never blocks on I2C, SPI or the camera.
"""

import threading
import time
from collections import deque

import numpy as np

from src.sensors import distance
from src.obstacle_challenge.logsetup import (
    Throttle, _fmt, clog, ilog, perflog, slog,
)
from src.obstacle_challenge.tuning import PERF_REPORT_PERIOD

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
        with self.cond:
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.frame_counter
            return None, self.frame_counter

    def get_next_frame(self, last_counter, timeout=1.0):
        """Block until a frame newer than `last_counter` arrives.

        This is the only way the control loop should fetch frames. The old loop
        called get_frame() and `continue`d when the counter hadn't moved, which
        spun a core flat out and starved this thread -- the main reason the run
        sat at 40-50 fps instead of the camera's 56.
        """
        with self.cond:
            self.cond.wait_for(
                lambda: self.frame_counter != last_counter or self.stop_event.is_set(),
                timeout=timeout,
            )
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.frame_counter, self.capture_time
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
        self.history = {ch: deque(maxlen=5) for ch in self.channels}

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
                time.sleep(0.3)
            time.sleep(0.3)
            self.initialization_complete.set()
            slog.debug("Initialization complete flag set.")

            consecutive_none = {ch: 0 for ch in self.channels}
            reinit_threshold = 30
            throttle = Throttle(1.0)
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

                    if throttle:
                        slog.debug("dist front=%s back=%s left=%s right=%s",
                                   _fmt(self.distance_center), _fmt(self.distance_back),
                                   _fmt(self.distance_left), _fmt(self.distance_right))

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
