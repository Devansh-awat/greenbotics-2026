import adafruit_bno055
import time
import numpy as np
import traceback
from src.obstacle_challenge import config
from src.sensors import i2c_bus

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
                sensor = adafruit_bno055.BNO055_I2C(i2c)
                sensor.mode = adafruit_bno055.NDOF_MODE
                time.sleep(1)
                temp = sensor.temperature
            print(f"INFO: Gyro (BNO055) Initialized. Temp: {temp}°C")
            return True
        except Exception as e:
            print(f"bno055.py: ERROR during Gyro initialisation: {e}")
            traceback.print_exc()
            time.sleep(0.2)
            sensor = None
    return False


def get_heading():
    if sensor:
        try:
            with i2c_bus.LOCK:
                heading, _, _ = sensor.euler
            if heading is not None:
                if getattr(config, "INVERT_GYRO", False):
                    heading = (360.0 - heading) % 360.0
                return heading
        except Exception:
            return None
    return None


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
    print("--- Cleaning up Gyro (BNO055) ---")
    pass


if __name__ == "__main__":
    print("--- BNO055 Euler Angle Reader ---")
    if not initialize() or sensor is None:
        print("Could not initialize sensor.")
    else:
        try:
            # Average a few readings to find the startup direction, then zero it
            print("Zeroing heading to current orientation...")
            readings = []
            for _ in range(20):
                h, _, _ = sensor.euler
                if h is not None:
                    readings.append(h)
                time.sleep(0.05)
            offset = np.mean(readings) if readings else 0.0
            print(f"Raw heading at start: {offset:.1f}°  →  will display as 0°")

            print("Reading euler angles (zeroed). Press Ctrl+C to exit.")
            while True:
                euler = sensor.euler
                heading = ((euler[0] or 0) - offset) % 360
                print(f"Euler: heading={heading:.1f}°, roll={euler[1]}, pitch={euler[2]}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            cleanup()
