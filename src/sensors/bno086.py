# -*- coding: utf-8 -*-
"""
Driver for the BNO086 (BNO08x family) IMU.

Drop-in replacement for ``bno055.py``: same public API
(``initialize``, ``get_heading``, ``get_initial_heading``, ``cleanup``) so the
rest of the code can ``from src.sensors import bno086 as bno055`` (or swap the
import) and use it identically.

The BNO08x exposes a fused absolute-orientation "rotation vector" as a
quaternion rather than Euler angles, so ``get_heading()`` converts the
quaternion yaw to a 0-360° heading to match the BNO055's ``euler[0]``.
"""

import math
import time
import traceback

import numpy as np
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

from src.obstacle_challenge import config
from src.sensors import i2c_bus

# --- Workaround for a bug in adafruit_bno08x ----------------------------------
# `_process_available_packets()` feeds EVERY incoming packet to `_handle_packet`,
# including channel-0 (SHTP_COMMAND) and channel-1 (EXE) control/advertisement
# responses. `_separate_batch` then misinterprets those as sensor reports and
# raises `RuntimeError("Unprocessable Batch bytes", ...)`, which makes
# `enable_feature` fail every time on this BNO086 over I2C on the Pi. The
# library's own `_wait_for_packet_type` already skips these channels; we make
# `_handle_packet` do the same so the noisy control packets are ignored.
_orig_handle_packet = adafruit_bno08x.BNO08X._handle_packet


def _safe_handle_packet(self, packet):
    if packet.channel_number in (
        adafruit_bno08x.BNO_CHANNEL_SHTP_COMMAND,
        adafruit_bno08x.BNO_CHANNEL_EXE,
    ):
        return
    return _orig_handle_packet(self, packet)


adafruit_bno08x.BNO08X._handle_packet = _safe_handle_packet
# ------------------------------------------------------------------------------

# BNO08x default I2C address is 0x4A (0x4B if DI/ADR is pulled high).
# This board's ADR pin is pulled high -> 0x4B (confirmed via i2cdetect).
BNO086_ADDRESS = 0x4B

sensor = None


def initialize():
    global sensor
    if not config.GYRO_ENABLED:
        print("INFO: Gyro is disabled in config.")
        return True

    for attempt in range(3):
        try:
            with i2c_bus.LOCK:
                i2c = i2c_bus.get_bus()
                sensor = BNO08X_I2C(i2c, address=BNO086_ADDRESS)
                # Enable the fused absolute-orientation report (magnetometer-free).
                sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
                time.sleep(0.05)
            print("INFO: Gyro (BNO086) Initialized (using Game Rotation Vector).")
            return True
        except Exception as e:
            print(f"bno086.py: ERROR during Gyro initialisation: {e}")
            traceback.print_exc()
            time.sleep(0.2)
            sensor = None
    return False


_tare_offset = 0.0


def _quaternion_to_heading(qi, qj, qk, qr):
    """Yaw (heading) in degrees [0, 360) from a unit quaternion."""
    # Standard quaternion -> yaw (rotation about Z).
    siny_cosp = 2.0 * (qr * qk + qi * qj)
    cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return yaw % 360.0


def _get_raw_heading():
    if sensor:
        try:
            with i2c_bus.LOCK:
                qi, qj, qk, qr = sensor.game_quaternion
            if None not in (qi, qj, qk, qr):
                heading = _quaternion_to_heading(qi, qj, qk, qr)
                if getattr(config, "INVERT_GYRO", False):
                    heading = (360.0 - heading) % 360.0
                return heading
        except Exception:
            return None
    return None


def get_heading(raw=False):
    """Get current heading in degrees [0, 360). Applies tare offset unless raw=True."""
    h = _get_raw_heading()
    if h is None:
        return None
    if raw:
        return h
    return (h - _tare_offset) % 360.0


def tare(target_angle=0.0):
    """Tare (zero) the current heading to target_angle (default 0.0°)."""
    global _tare_offset
    raw = _get_raw_heading()
    if raw is not None:
        _tare_offset = (raw - target_angle) % 360.0
        print(f"INFO: BNO086 tared (offset: {_tare_offset:.2f}°, heading: {get_heading():.2f}°)")
        return True
    return False


def reset_tare():
    """Reset tare offset back to 0.0°."""
    global _tare_offset
    _tare_offset = 0.0


def lock_calibration():
    """Lock/freeze calibration so the BNO086 does not dynamically alter gyro bias while driving."""
    global sensor
    if sensor is None:
        return False
    try:
        with i2c_bus.LOCK:
            # ME command: 0 for accel, 0 for gyro, 0 for mag disables dynamic background calibration
            sensor._send_me_command([0, 0, 0, 0, 0, 0, 0, 0, 0])
        print("INFO: BNO086 calibration locked (dynamic calibration disabled).")
        return True
    except Exception as e:
        print(f"WARNING: Could not lock BNO086 calibration: {e}")
        return False


def unlock_calibration():
    """Re-enable auto-calibration routine."""
    global sensor
    if sensor is None:
        return False
    try:
        with i2c_bus.LOCK:
            sensor.begin_calibration()
        print("INFO: BNO086 auto-calibration enabled.")
        return True
    except Exception as e:
        print(f"WARNING: Could not enable BNO086 calibration: {e}")
        return False


def get_initial_heading(num_readings=20):
    if not sensor:
        return 0.0

    print("INFO: Acquiring initial heading for gyro zero point...")
    readings = []
    for _ in range(num_readings):
        yaw = get_heading()
        if yaw is not None:
            readings.append(yaw)
        time.sleep(0.05)

    if readings:
        initial_heading = np.mean(readings)
        print(f"INFO: Gyro zero point set to: {initial_heading:.2f} degrees.")
        return initial_heading
    else:
        print("WARNING: Could not get initial gyro heading.")
        return 0.0


def cleanup():
    print("--- Cleaning up Gyro (BNO086) ---")
    pass


if __name__ == "__main__":
    print("--- BNO086 Heading Reader ---")
    if not initialize() or sensor is None:
        print("Could not initialize sensor.")
    else:
        try:
            # Average a few readings to find the startup direction, then zero it
            print("Zeroing heading to current orientation...")
            readings = []
            for _ in range(20):
                h = get_heading()
                if h is not None:
                    readings.append(h)
                time.sleep(0.05)
            offset = np.mean(readings) if readings else 0.0
            print(f"Raw heading at start: {offset:.1f}°  →  will display as 0°")

            print("Reading heading (zeroed). Press Ctrl+C to exit.")
            while True:
                h = get_heading()
                if h is not None:
                    heading = (h - offset) % 360
                    print(f"Heading: {heading:.1f}°")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            cleanup()
