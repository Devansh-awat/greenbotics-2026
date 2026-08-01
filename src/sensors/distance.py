# -*- coding: utf-8 -*-
"""
Distance sensing with up to four VL53L4CD sensors on I2C bus 3.

All four chips power up at the default address 0x29, so they are brought up one
at a time: every XSHUT is held low (in reset), then each sensor is released,
probed at 0x29 and moved to its own address before the next one is woken.

Callers address sensors by a logical "channel", which is simply the sensor's
XSHUT GPIO number:

    channel 17 -> Front (0x2A)   [wired]
    channel 16 -> Back  (0x2B)   [wired]
    channel 22 -> Left  (0x2C)   [not wired yet — optional]
    channel 27 -> Right (0x2D)   [not wired yet — optional]

Left/Right are declared optional: if they aren't physically present, bring-up
fails fast, initialise() still returns True, and get_distance() on those
channels just returns None. Wire them up and they start working with no code
change.

The legacy mux channel numbers still work: 0 maps to Front, 3 maps to Back.
"""

import struct
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
_REG_RANGE_OFFSET_MM = 0x001E  # signed 16-bit register for range offset (mm * 4)
I2C_BUS = 3

# Channels == XSHUT GPIO numbers.
FRONT_CHANNEL = 17
BACK_CHANNEL = 16
LEFT_CHANNEL = 22
RIGHT_CHANNEL = 27

# Sensor table: channel -> (i2c address, label, required)
# "required" sensors make initialise() return False if they fail; the optional
# ones (not yet wired) are allowed to be absent.
SENSORS = {
    FRONT_CHANNEL: (0x2A, "Front VL53L4CD (GPIO17)", True),
    BACK_CHANNEL:  (0x2B, "Back VL53L4CD (GPIO16)",  True),
    LEFT_CHANNEL:  (0x2C, "Left VL53L4CD (GPIO22)",  False),
    RIGHT_CHANNEL: (0x2D, "Right VL53L4CD (GPIO27)", False),
}

# Bring-up order matters only in that each sensor must be alone at 0x29 when it
# is probed; _bring_up() guarantees that by leaving the rest in reset.
SENSOR_CHANNELS = list(SENSORS.keys())

# Legacy mux channel numbers used by older call sites.
_CHANNEL_ALIASES = {0: FRONT_CHANNEL, 3: BACK_CHANNEL}

# Timing settings
TIMING_BUDGET_MS = 33
BOOT_DELAY_S = 0.2
BRING_UP_RETRIES = 6           # for required sensors
OPTIONAL_BRING_UP_RETRIES = 2  # unwired sensors: fail fast, don't stall startup

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


def _resolve_channel(channel):
    """Map a legacy mux channel number onto its XSHUT-GPIO channel."""
    return _CHANNEL_ALIASES.get(channel, channel)


def _bring_up(channel, retries=None):
    """Wake one sensor (all others stay in reset), move it off 0x29.

    Each attempt does a full XSHUT reset (low -> high) so the chip boots fresh:
    a chip left ranging / killed mid-transaction by a previous run gets wedged
    (ACKs its address but stalls every data transfer), and only a reset edge
    clears it — re-probing alone won't. A reset also returns the chip to the
    default 0x29, so re-probing 0x29 is correct on every attempt.

    Sensors brought up earlier already sit at their own address, so they don't
    answer at 0x29 and can stay awake here.
    """
    global _sensors

    new_address, label, required = SENSORS[channel]
    if retries is None:
        retries = BRING_UP_RETRIES if required else OPTIONAL_BRING_UP_RETRIES

    xshut = _xshut_devices.get(channel)
    if xshut is None:
        raise RuntimeError(f"XSHUT pin for channel {channel} not initialized.")

    last_err = None
    for attempt in range(retries):
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
                # Set range offset to 0 mm
                s._write_register(_REG_RANGE_OFFSET_MM, struct.pack(">h", 0))
                s.start_ranging()
            print(f"  - {label}: up at 0x{new_address:02X} (attempt {attempt + 1})")
            _sensors[channel] = s
            return s
        except (OSError, ValueError, RuntimeError) as e:
            last_err = e
            time.sleep(0.05)

    # Leave a failed sensor in reset so it can't sit on 0x29 and collide with a
    # later bring-up.
    xshut.off()
    raise RuntimeError(f"{label}: bring-up failed after {retries} tries: {last_err}")


def initialise(i2c_bus_num=I2C_BUS, **_ignored):
    """Initialize the I2C bus and bring up every configured VL53L4CD.

    Returns True if all *required* sensors came up. Optional (not-yet-wired)
    sensors are reported but don't fail initialisation.
    """
    global _i2c, _sensors, _xshut_devices

    # Clean up any previously opened objects
    cleanup(silent=True)

    try:
        _i2c = _ExtendedI2C(i2c_bus_num)
    except Exception as e:
        print(f"FATAL: Could not initialize I2C bus {i2c_bus_num}. Error: {e}")
        return False

    try:
        # Initial value False: hold every sensor low (in reset) from the start so
        # only one at a time answers at the default 0x29.
        for channel in SENSOR_CHANNELS:
            _xshut_devices[channel] = DigitalOutputDevice(channel, initial_value=False)
        time.sleep(0.05)
    except Exception as e:
        print(f"FATAL: Could not initialize GPIO XSHUT devices. Error: {e}")
        cleanup(silent=True)
        return False

    ok = True
    for channel in SENSOR_CHANNELS:
        _, label, required = SENSORS[channel]
        try:
            _bring_up(channel)
        except Exception as e:
            if required:
                print(f"distance.py: ERROR initializing {label}: {e}")
                traceback.print_exc()
                ok = False
            else:
                print(f"distance.py: {label} not present (optional) — skipping.")

    print(f"INFO: Sensor initialization complete. Status: {ok}. "
          f"Live channels: {sorted(_sensors.keys())}")
    return ok


def reinit_sensor(channel, **_ignored):
    """Reinitialize the sensor on the specified channel."""
    global _sensors

    channel = _resolve_channel(channel)
    print("Reinitializing sensor on channel", channel)

    if channel not in SENSORS:
        return False

    old = _sensors.pop(channel, None)
    if old is not None:
        try:
            with _lock:
                old.stop_ranging()
        except Exception as e:
            print(f"Warning: Could not stop existing sensor on channel {channel}: {e}")

    try:
        _bring_up(channel)
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
    channel is not configured / not wired up, or no new frame is ready
    (non-blocking).
    """
    actual_channel = _resolve_channel(channel)

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


def cleanup(silent=False):
    """Stops ranging on all initialized sensors and releases GPIOs."""
    if not silent:
        print("\n--- Cleaning up Sensors ---")
    for channel, sensor in list(_sensors.items()):
        try:
            with _lock:
                sensor.stop_ranging()
        except (OSError, AttributeError):
            if not silent:
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

    if not silent:
        print("Cleanup complete.")


if __name__ == "__main__":
    print("--- Testing Distance Sensor Library (VL53L4CD x4 on I2C3) ---")
    if not initialise():
        print("Test failed: a required sensor did not come up.")
    elif not _sensors:
        print("No sensors were detected.")
    else:
        try:
            print("\nReading data from all detected sensors. Press Ctrl+C to stop.")
            while True:
                output_line_parts = []
                for ch in sorted(_sensors.keys()):
                    name = SENSORS[ch][1].split()[0]
                    dist_mm = get_distance(ch)
                    if dist_mm is not None:
                        output_line_parts.append(f"{name}(Ch{ch}): {dist_mm:6.0f} mm")
                    else:
                        output_line_parts.append(f"{name}(Ch{ch}):   ----   ")
                print(f"\r{(' | '.join(output_line_parts))}", end="", flush=True)
                time.sleep(1 / 30)  # ~30 Hz
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
