# -*- coding: utf-8 -*-
"""
Distance sensing with two VL53L4CD sensors on I2C bus 3:

- Front: one VL53L4CD on GPIO 17 (Address 0x2A).
- Back:  one VL53L4CD on GPIO 27 (Address 0x2B).

Callers address sensors by a logical "channel" number, kept consistent with the
old mux-only layout so the rest of the codebase is unchanged:

    channel 0 -> Front (VL53L4CD, address 0x2A, XSHUT GPIO17)
    channel 3 -> Back  (VL53L4CD, address 0x2B, XSHUT GPIO27)
"""

import time
import traceback
import threading
import busio
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as _LinuxI2C
from gpiozero import DigitalOutputDevice
import adafruit_vl53l4cd

# Patch adafruit_vl53l4cd.VL53L4CD._write_register to disable 1MHz Fast Plus mode
# (writing 0x00 instead of 0x12 to register 0x002D). This keeps standard I2C filters
# active on the sensor, avoiding clock stretching/timeout issues on the Pi 5.
_original_write_register = adafruit_vl53l4cd.VL53L4CD._write_register

def _custom_write_register(self, address, data, length=None):
    import struct
    if length is None:
        length = len(data)
    if address == 0x002D and length > 0:
        data_list = list(data[:length])
        if data_list[0] == 0x12:
            data_list[0] = 0x00
        data = bytes(data_list)
    with self.i2c_device as i2c:
        i2c.write(struct.pack(">H", address) + data[:length])

adafruit_vl53l4cd.VL53L4CD._write_register = _custom_write_register

# --- Configuration ---
I2C_BUS = 3

# Channels (XSHUT Pin Numbers)
FRONT_CHANNEL = 17   # Forward VL53L4CD (GPIO17)
BACK_CHANNEL = 27    # Backward VL53L4CD (GPIO27)

# Keep compatibility variables so callers don't break
VL53L1X_CHANNELS = [0]
VL53L8CX_CHANNELS = [3]
SENSOR_CHANNELS = [0, 3, FRONT_CHANNEL, BACK_CHANNEL]

# Pins and Addresses (matching test_vl53l4cd_dual.py)
XSHUT_A_PIN = 17
XSHUT_B_PIN = 27
ADDR_A = 0x2A
ADDR_B = 0x2B

# Timing settings
TIMING_BUDGET_MS = 33
BOOT_DELAY_S = 0.2
BRING_UP_RETRIES = 6

# range_status values we accept as a usable distance
_VALID_STATUSES = (
    adafruit_vl53l4cd.RANGE_VALID,
    adafruit_vl53l4cd.RANGE_WARN_SIGMA_ABOVE,
    adafruit_vl53l4cd.RANGE_WARN_SIGMA_BELOW,
)

# --- Global Variables ---
_i2c = None
_sensors = {}
_xshut_devices = {}
_lock = threading.Lock()
_diag = {}


class _ExtendedI2C(busio.I2C):
    """Minimal busio.I2C that targets an arbitrary /dev/i2c-N by bus number.

    busio.I2C delegates every transaction to ``self._i2c``; we just point that
    at Blinka's Linux smbus wrapper for the chosen bus.
    """

    def __init__(self, bus_num):
        self._bus_num = bus_num
        self.init(bus_num)

    def init(self, bus_num):
        self.deinit()
        self._i2c = _LinuxI2C(bus_num, mode=_LinuxI2C.MASTER)

    def deinit(self):
        try:
            del self._i2c
        except AttributeError:
            pass


def _bring_up(channel, new_address, label):
    """Wake one sensor (others must be in reset), move it to new_address.

    Each attempt does a full XSHUT reset (low -> high) so the chip boots fresh:
    a chip left ranging / killed mid-transaction by a previous run gets wedged
    (ACKs its address but stalls every data transfer), and only a reset edge
    clears it — re-probing alone won't. A reset also returns the chip to the
    default 0x29, so re-probing 0x29 is correct on every attempt.
    """
    global _sensors, _xshut_devices, _i2c

    xshut = _xshut_devices.get(channel)
    if xshut is None:
        raise RuntimeError(f"XSHUT pin for channel {channel} not initialized.")

    last_err = None
    for attempt in range(BRING_UP_RETRIES):
        xshut.off()
        time.sleep(0.05)
        xshut.on()              # clean reset edge -> fresh boot at 0x29
        time.sleep(BOOT_DELAY_S)
        try:
            with _lock:
                s = adafruit_vl53l4cd.VL53L4CD(_i2c)  # responds at default 0x29
                s.set_address(new_address)
                s.timing_budget = TIMING_BUDGET_MS
                s.inter_measurement = 0
                s.start_ranging()
            print(f"  - {label}: up at 0x{new_address:02X} (attempt {attempt + 1})")
            _sensors[channel] = s
            return s
        except (OSError, ValueError, RuntimeError) as e:
            last_err = e
            time.sleep(0.05)

    raise RuntimeError(f"{label}: bring-up failed after retries: {last_err}")


def initialise(i2c_bus_num=I2C_BUS, **_ignored):
    """
    Initializes the I2C bus and brings up the two VL53L4CD sensors
    as front (channel 0) and back (channel 3) sensors.
    """
    global _i2c, _sensors, _xshut_devices

    # Clean up any previously opened objects
    cleanup()

    try:
        _i2c = _ExtendedI2C(i2c_bus_num)
    except Exception as e:
        print(f"FATAL: Could not initialize I2C bus {i2c_bus_num}. Error: {e}")
        return False

    try:
        # Initial value False: hold both low (in reset) from the start
        _xshut_devices[FRONT_CHANNEL] = DigitalOutputDevice(XSHUT_A_PIN, initial_value=False)
        _xshut_devices[BACK_CHANNEL] = DigitalOutputDevice(XSHUT_B_PIN, initial_value=False)
        time.sleep(0.05)
    except Exception as e:
        print(f"FATAL: Could not initialize GPIO XSHUT devices. Error: {e}")
        cleanup()
        return False

    ok = True

    # Front VL53L4CD (GPIO17 -> 0x2A)
    try:
        _bring_up(FRONT_CHANNEL, ADDR_A, "Front VL53L4CD (GPIO17)")
    except Exception as e:
        print(f"distance.py: ERROR initializing Front VL53L4CD: {e}")
        traceback.print_exc()
        ok = False

    # Back VL53L4CD (GPIO27 -> 0x2B)
    try:
        _bring_up(BACK_CHANNEL, ADDR_B, "Back VL53L4CD (GPIO27)")
    except Exception as e:
        print(f"distance.py: ERROR initializing Back VL53L4CD: {e}")
        traceback.print_exc()
        ok = False

    print(f"INFO: Sensor initialization complete. Status: {ok}")
    return ok


def reinit_sensor(channel, **_ignored):
    """Reinitialize the sensor on the specified channel."""
    global _sensors, _xshut_devices
    print("Reinitializing sensor on channel", channel)
    
    # Map backward compatibility channels
    if channel == 0:
        channel = FRONT_CHANNEL
    elif channel == 3:
        channel = BACK_CHANNEL

    if channel not in SENSOR_CHANNELS:
        return False

    old = _sensors.pop(channel, None)
    if old is not None:
        try:
            with _lock:
                old.stop_ranging()
        except Exception as e:
            print(f"Warning: Could not stop existing sensor on channel {channel}: {e}")

    try:
        addr = ADDR_A if channel == FRONT_CHANNEL else ADDR_B
        label = "Front VL53L4CD (GPIO17)" if channel == FRONT_CHANNEL else "Back VL53L4CD (GPIO27)"
        _bring_up(channel, addr, label)
        return True
    except Exception as e:
        print(f"Warning: Error during reinit on channel {channel}: {e}")
        return False


def get_diag(reset=True):
    """Return (and optionally reset) per-channel None-reason counters."""
    snap = {ch: dict(c) for ch, c in _diag.items()}
    if reset:
        for c in _diag.values():
            for k in c:
                c[k] = 0
    return snap


def _bump(channel, key):
    c = _diag.setdefault(channel,
                         {'ok': 0, 'not_ready': 0, 'no_target': 0,
                          'absent': 0, 'err': 0})
    c[key] += 1


def get_distance(channel):
    """
    Returns distance in mm (float) for a configured channel, or None if the
    channel is not configured or no new frame is ready (non-blocking).
    """
    # Map backward compatibility channels
    actual_channel = channel
    if channel == 0:
        actual_channel = FRONT_CHANNEL
    elif channel == 3:
        actual_channel = BACK_CHANNEL

    if actual_channel not in _sensors:
        _bump(channel, 'absent')
        return None

    sensor = _sensors[actual_channel]
    try:
        with _lock:
            if not sensor.data_ready:
                _bump(channel, 'not_ready')
                return None
            distance_cm = sensor.distance
            status = sensor.range_status
            sensor.clear_interrupt()

        if status not in _VALID_STATUSES or distance_cm is None:
            _bump(channel, 'no_target')
            return None

        _bump(channel, 'ok')
        return float(distance_cm * 10.0)
    except (OSError, IOError) as e:
        _bump(channel, 'err')
        print(f"\nI/O Error on channel {channel}. Error: {e}")
        return None


def cleanup():
    """Stops ranging on all initialized sensors and releases GPIOs."""
    print("\n--- Cleaning up Sensors ---")
    for channel, sensor in list(_sensors.items()):
        try:
            with _lock:
                sensor.stop_ranging()
        except (OSError, AttributeError):
            print(f"Warning: Error during cleanup of sensor on channel {channel}.")

    _sensors.clear()

    # Release XSHUT devices
    global _xshut_devices
    for channel, dev in list(_xshut_devices.items()):
        try:
            dev.close()
        except Exception:
            pass
    _xshut_devices.clear()

    # Close I2C bus
    global _i2c
    if _i2c is not None:
        try:
            _i2c.deinit()
        except Exception:
            pass
        _i2c = None

    print("Cleanup complete.")


if __name__ == "__main__":
    print("--- Testing Distance Sensor Library (Dual VL53L4CD on I2C3) ---")
    if not initialise():
        print("Test failed during initialization.")
    elif not _sensors:
        print("No sensors were detected.")
    else:
        try:
            print("\nReading data from all detected sensors. Press Ctrl+C to stop.")
            print(list(_sensors.keys()))
            while True:
                output_line_parts = []
                for i in sorted(_sensors.keys()):
                    dist_mm = get_distance(i)
                    if dist_mm is not None:
                        output_line_parts.append(f"Ch{i}: {dist_mm:6.0f} mm")
                    else:
                        output_line_parts.append(f"Ch{i}:   ----   ")
                print(f"\r{(' | '.join(output_line_parts))}", end="", flush=True)
                time.sleep(1 / 30)  # ~30 Hz
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
