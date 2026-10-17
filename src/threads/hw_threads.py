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
    clog, ilog, perflog, slog,
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
        self.gyro = None
        # Ring buffer for gyro samples: (timestamp, gx, gy, gz) in rad/s
        self.gyro_history = deque(maxlen=100)

    def run(self):
        try:
            self.bno.initialize()
            ilog.info("IMU initialized.")
            self.initialization_complete.set()
            while not self.stop_event.is_set():
                now = time.monotonic()
                if hasattr(self.bno, "get_readings"):
                    heading, gyro = self.bno.get_readings()
                else:
                    heading = self.bno.get_heading()
                    gyro = getattr(self.bno, "get_gyro", lambda: None)()

                with self.lock:
                    if heading is not None:
                        self.heading = heading
                    if gyro is not None:
                        self.gyro = gyro
                        self.gyro_history.append((now, gyro[0], gyro[1], gyro[2]))
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

    def get_gyro(self):
        with self.lock:
            return self.gyro

    def get_vibration(self, window_sec=0.5):
        """Compute angular vibration statistics over recent gyro readings.

        Vibration is measured as the standard deviation of angular velocity (rad/s
        and deg/s) around the mean rotation rate over the time window. This isolates
        high-frequency mechanical jitter / vibration from steady turns or motion.

        Returns:
            dict with total_deg_s, total_rad_s, axes_deg_s (x, y, z), peak_deg_s,
            samples count, or None if insufficient samples.
        """
        now = time.monotonic()
        cutoff = now - window_sec
        with self.lock:
            samples = [
                (gx, gy, gz) for ts, gx, gy, gz in self.gyro_history if ts >= cutoff
            ]
        if len(samples) < 3:
            return None

        arr = np.array(samples)  # shape (N, 3), units: rad/s
        std_rad = np.std(arr, axis=0)
        std_deg = np.degrees(std_rad)
        total_rad = float(np.linalg.norm(std_rad))
        total_deg = float(np.degrees(total_rad))

        mean_rate = np.mean(arr, axis=0)
        deviations = arr - mean_rate
        peak_rad = float(np.max(np.linalg.norm(deviations, axis=1)))
        peak_deg = float(np.degrees(peak_rad))

        # Pitch (X) and Roll (Y) lie in the plane of the flat arena floor.
        # Since the robot does not intentionally rotate around X or Y, all motion on X & Y
        # is mechanical wobble / surface vibration.
        pitch_roll_rad = float(np.linalg.norm(std_rad[:2]))
        pitch_roll_deg = float(np.degrees(pitch_roll_rad))
        yaw_jitter_deg = float(std_deg[2])

        return {
            "total_deg_s": total_deg,
            "total_rad_s": total_rad,
            "pitch_roll_deg_s": pitch_roll_deg,
            "yaw_jitter_deg_s": yaw_jitter_deg,
            "axes_deg_s": (float(std_deg[0]), float(std_deg[1]), float(std_deg[2])),
            "peak_deg_s": peak_deg,
            "samples": len(samples),
        }

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

            while not self.stop_event.is_set():
                try:
                    raw_readings = {}
                    for ch in self.channels:
                        val = self.dist.get_distance(ch)
                        if val is not None and val < 50.0:
                            val = None
                        raw_readings[ch] = val

                    readings = {}
                    with self.lock:
                        for ch, val in raw_readings.items():
                            if val is not None:
                                self.history[ch].append(val)

                            if self.history[ch]:
                                sorted_vals = sorted(self.history[ch])
                                if len(sorted_vals) >= 3:
                                    readings[ch] = sum(sorted_vals[1:-1]) / (len(sorted_vals) - 2)
                                else:
                                    readings[ch] = sum(sorted_vals) / len(sorted_vals)
                            else:
                                readings[ch] = None

                        self.distance_center = readings.get(distance.FRONT_CHANNEL)
                        self.distance_back = readings.get(distance.BACK_CHANNEL)
                        self.distance_left = readings.get(distance.LEFT_CHANNEL)
                        self.distance_right = readings.get(distance.RIGHT_CHANNEL)

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
