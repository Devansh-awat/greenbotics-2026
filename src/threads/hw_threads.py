"""Background threads that feed the control loop, plus the perf monitor.

Each of these owns one piece of hardware and publishes its latest reading behind a
lock, so the control loop never blocks on I2C, SPI or the camera.
"""

import threading
import time
from collections import deque

import numpy as np

from src.sensors import distance
from src.logs.setup import (
    Throttle, _fmt, clog, ilog, perflog, slog,
)
from src.obstacle_challenge.tuning import CAMERA_STALE_TIMEOUT_S, PERF_REPORT_PERIOD

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

    def get_next_frame(self, last_counter, timeout=CAMERA_STALE_TIMEOUT_S):
        """Block until a frame newer than `last_counter` arrives.

        Returns (frame, counter, capture_time, fresh). `fresh` is False when the
        wait TIMED OUT (or the thread is stopping) -- and then `frame` is the same
        one the caller already acted on, with a `capture_time` that is already
        `timeout` old. Steering on it means steering on stale pixels while the
        robot keeps moving at up to ~1 m/s, so every caller that actuates must
        check `fresh` and cut power rather than reuse the frame. The old signature
        returned no such signal: on timeout it handed back the stale frame with an
        unchanged counter, which the control loop scored as `skipped = 0` and
        processed as if it were new.

        `timeout` is 50 ms by default, not a round 1 s, because that is what the
        camera actually does: over 308k frame intervals from 245 archived runs the
        median is 17.9 ms (the IMX708's 56 fps), p99.9 is 31 ms and p99.99 is
        38.6 ms. Only 5 intervals in 308k exceeded 50 ms and the largest ever seen
        was 110 ms, so 50 ms is ~2.8 frame times and leaves a 30% margin over the
        worst routine jitter while capping blind travel at ~5 cm. Callers that are
        not steering can pass a longer one.

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
            # wait_for() returns the predicate's final value: True if a new frame
            # (or a stop) arrived, False only if the timeout expired.
            fresh = bool(self.cond.wait_for(
                lambda: self.frame_counter != last_counter or self.stop_event.is_set(),
                timeout=timeout,
            ))
            if self.stop_event.is_set():
                fresh = False
            if self.latest_frame is not None:
                return self.latest_frame, self.frame_counter, self.capture_time, fresh
            return None, self.frame_counter, 0.0, False

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
