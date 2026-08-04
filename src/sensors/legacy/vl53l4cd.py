# -*- coding: utf-8 -*-
"""
VL53L4CD driver — single ToF sensor on the main I2C bus (i2c-1, GPIO2/3).

History / why this looked broken: the VL53L4CD originally failed on i2c-1 — it
ACKed its address (showed in `i2cdetect -y 1` at 0x29) but every data-phase
read returned OSError [Errno 121]. Root cause was too-strong combined I2C
pull-ups: the fixed 1.8 kOhm board pull-ups on GPIO2/3 in parallel with the
sensor breakout's own pull-ups pulled the bus impedance too low to reach a
valid logic low during the data phase. It was NOT clock stretching and NOT a
DesignWare controller limitation (lowering the baudrate didn't help).

Fix: REMOVE the breakout's pull-up resistors. The sensor then reads perfectly on
i2c-1 (confirmed: model id 0xEBAA, full ranging). Keep the breakout pull-ups
off — re-adding them (or adding a second pulled-up breakout) brings the failure
back.

This sensor shares i2c-1 with the BNO086 IMU (0x4B), so it uses the shared
``i2c_bus`` bus object + process-wide lock to serialise transactions across
threads (the ToF read loop vs the IMU thread). See i2c_bus.py.

WIRING:
    SDA -> GPIO2 (pin 3), SCL -> GPIO3 (pin 5)    [Pi board has 1.8k pull-ups]
    XSHUT -> 3.3V (must be high to enable)
    VIN -> 3.3V, GND -> GND
    Breakout pull-up resistors REMOVED (required — see above).

Address note: 0x29 collides with a VL53L1X if one is muxed onto this bus. The
plan is to remove the 1X and the mux, leaving the 4CD + BNO086 alone on i2c-1.
"""

import struct
import time
import busio
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as _LinuxI2C
from gpiozero import DigitalOutputDevice
import adafruit_vl53l4cd

from src.sensors import i2c_bus

# Registers the Adafruit driver doesn't expose. Values per the ST VL53L4CD ULD.
_REG_RANGE_OFFSET_MM = 0x001E  # signed 16-bit, stored as offset_mm * 4
_REG_XTALK_KCPS = 0x0016       # signed 16-bit crosstalk compensation

# I2C bus the sensor lives on. Bus 1 = the main shared bus (GPIO2/3, uses the
# locked i2c_bus shared with the IMU). Any other bus (e.g. 3 = i2c3-pi5 on
# GPIO14/15) is opened with a private _ExtendedI2C handle.
#
# NOTE: the Pi 5 RP1 hardware controllers (bus 1 AND bus 3) intermittently choke
# on this chip's heavy init sequence because it clock-stretches harder than the
# controller tolerates — see the retry-on-reset loop in initialize(). The only
# fully reliable bus found was the bit-banged i2c-gpio software bus.
I2C_BUS = 3

# XSHUT (chip enable) GPIO. The breakout pull-ups were removed, so XSHUT has
# nothing holding it high — left floating the chip drops into reset and vanishes
# off the bus (intermittent OSError 121 / device disappears). initialize() drives
# this sensor's XSHUT high and holds it.
#
# This rig has a SECOND VL53L4CD whose XSHUT is on GPIO27; it also defaults to
# 0x29, so we hold it in reset (low) to keep 0x29 unambiguous for this single-
# sensor driver. To read BOTH sensors, use test_vl53l4cd_dual.py instead, which
# sequences XSHUT and reassigns the two to 0x2A/0x2B.
XSHUT_PIN = 17           # this sensor's XSHUT -> driven high
XSHUT_HOLD_LOW = (27,)   # other sensors' XSHUT -> held in reset
XSHUT_BOOT_DELAY_S = 0.2  # settle time after XSHUT rises before talking to chip

# Range offset (mm) found by calibrate_offset() against a 100 mm target. The
# offset register is volatile (resets every power cycle), so initialize()
# reapplies this. Re-run `... calibrate 100` and update this if the mounting or
# sensor changes.
CALIBRATED_OFFSET_MM = 3.5

# range_status values we accept as a usable distance. 0 is fully valid; the two
# "sigma" warnings still carry a real distance (just flagged noisy). Everything
# else (signal too weak, out-of-bounds/invalid phase, crosstalk, hw fail, ...)
# means the distance field is garbage and must be rejected.
_VALID_STATUSES = (
    adafruit_vl53l4cd.RANGE_VALID,
    adafruit_vl53l4cd.RANGE_WARN_SIGMA_ABOVE,
    adafruit_vl53l4cd.RANGE_WARN_SIGMA_BELOW,
)

sensor = None
last_known_distance = None
_xshut_devices = []
_i2c_handle = None


def _open_bus():
    """Return an I2C bus object for I2C_BUS. Bus 1 uses the shared locked
    i2c_bus (coexists with the IMU); other buses get a cached private handle."""
    global _i2c_handle
    if I2C_BUS == 1:
        return i2c_bus.get_bus()
    if _i2c_handle is None:
        _i2c_handle = _ExtendedI2C(I2C_BUS)
    return _i2c_handle


def _enable_xshut():
    """Reset this sensor with a clean XSHUT low->high edge and hold it high.

    A bare low->high pulse matters: if a previous run left the chip ranging or
    was killed mid-transaction, it gets wedged (ACKs its address but stalls every
    data transfer). Just driving XSHUT high (no falling edge) leaves it wedged;
    a real reset edge forces a fresh boot and clears it. Other sensors on the bus
    are held in reset (low). Handles are held for the process lifetime (released
    in cleanup()); letting them go floats XSHUT and the chip resets.
    """
    global _xshut_devices
    if _xshut_devices:
        return
    holds = [DigitalOutputDevice(p, initial_value=False) for p in XSHUT_HOLD_LOW]
    this = DigitalOutputDevice(XSHUT_PIN, initial_value=False)  # start in reset
    time.sleep(0.05)
    this.on()                                                   # reset edge -> boot
    time.sleep(XSHUT_BOOT_DELAY_S)
    _xshut_devices = [this] + holds


def _reset_sensor():
    """Pulse this sensor's XSHUT low->high for a fresh boot (clears a wedged chip)."""
    if not _xshut_devices:
        _enable_xshut()
        return
    dev = _xshut_devices[0]  # this sensor's XSHUT (holds come after)
    dev.off()
    time.sleep(0.15)
    dev.on()
    time.sleep(XSHUT_BOOT_DELAY_S)


def _release_xshut():
    global _xshut_devices
    for dev in _xshut_devices:
        try:
            dev.close()
        except Exception:
            pass
    _xshut_devices = []


class _ExtendedI2C(busio.I2C):
    """Minimal busio.I2C that targets an arbitrary /dev/i2c-N by bus number.

    busio.I2C delegates every transaction to ``self._i2c``; we just point that
    at Blinka's Linux smbus wrapper for the chosen bus. This is the same trick
    adafruit_extended_bus.ExtendedI2C uses, inlined so we don't need the extra
    package (which isn't installed / fetchable on this Pi). Used by the
    standalone multi-sensor test util; the main driver uses the shared i2c_bus.
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


def initialize(timing_budget_ms=50, inter_measurement_ms=0, retries=12):
    """Initialize the VL53L4CD on the shared i2c-1 bus. Returns True/False.

    The chip's heavy init sequence (config blob + VHV calibration) makes it
    clock-stretch hard, and on the Pi 5 RP1 hardware controller that intermittently
    trips the I2C transfer timeout (OSError 110/121) — even though plain reads are
    100% reliable. The cure is a clean XSHUT reset on EVERY attempt (a wedged chip
    only clears on a fresh boot) plus generous retries.
    """
    global sensor, last_known_distance

    _enable_xshut()  # claim pins, hold the other sensor in reset

    last_err = None
    for attempt in range(retries):
        _reset_sensor()  # fresh boot each try — a wedged chip won't init otherwise
        try:
            with i2c_bus.LOCK:
                i2c = _open_bus()
                sensor = adafruit_vl53l4cd.VL53L4CD(i2c)
                sensor.timing_budget = timing_budget_ms
                sensor.inter_measurement = inter_measurement_ms
                set_offset(CALIBRATED_OFFSET_MM)  # offset reg is volatile; reapply
                sensor.start_ranging()
            last_known_distance = None
            print(
                f"INFO: VL53L4CD initialized at 0x29 on i2c-{I2C_BUS} "
                f"(offset {CALIBRATED_OFFSET_MM:+.1f} mm, attempt {attempt + 1})."
            )
            return True
        except (ValueError, OSError, RuntimeError) as e:
            last_err = e
            sensor = None

    print(
        f"ERROR: No VL53L4CD on i2c-{I2C_BUS} after {retries} tries "
        f"(XSHUT high, bus pull-ups present). Last error: {last_err}"
    )
    return False


def get_distance():
    """
    NON-BLOCKING read. Returns distance in mm, or last known value if busy.
    Returns None on invalid reading.
    """
    if sensor is None:
        return None

    global last_known_distance
    try:
        with i2c_bus.LOCK:
            if not sensor.data_ready:
                return last_known_distance
            distance_cm = sensor.distance
            status = sensor.range_status
            sensor.clear_interrupt()

        if status not in _VALID_STATUSES or distance_cm is None:
            last_known_distance = None
            return None

        last_known_distance = distance_cm * 10.0
        return last_known_distance
    except OSError:
        return last_known_distance


def get_new_distance(timeout=1.0):
    """BLOCKING read; waits for a fresh value. Returns distance in mm or None."""
    if sensor is None:
        return None

    start_time = time.monotonic()
    while True:
        try:
            with i2c_bus.LOCK:
                ready = sensor.data_ready
                if ready:
                    distance_cm = sensor.distance
                    status = sensor.range_status
                    sensor.clear_interrupt()
            if ready:
                break
        except OSError:
            return None
        if time.monotonic() - start_time > timeout:
            return None
        time.sleep(0.005)

    if status not in _VALID_STATUSES or distance_cm is None:
        return None
    return distance_cm * 10.0


def get_offset():
    """Return the current part-to-part range offset in mm (signed)."""
    with i2c_bus.LOCK:
        raw = struct.unpack(">h", sensor._read_register(_REG_RANGE_OFFSET_MM, 2))[0]
    return raw / 4.0


def set_offset(offset_mm):
    """Write the part-to-part range offset (mm, signed). Added to every reading."""
    with i2c_bus.LOCK:
        sensor._write_register(
            _REG_RANGE_OFFSET_MM, struct.pack(">h", int(offset_mm * 4))
        )


def calibrate_offset(target_mm, samples=20):
    """
    ST offset-calibration routine the Adafruit driver omits.

    Place a flat, matte target squarely facing the sensor at a known distance
    (ST recommends ~100 mm). This zeroes the offset, averages `samples` valid
    readings, and programs offset = target_mm - measured_avg so the sensor
    reports true distance. Returns the offset it programmed (mm), or None if it
    couldn't get enough valid samples.

    NOTE: calibrate at a real working distance, NOT at touch (~0 mm) — at
    contact the return saturates and the reading isn't representative.
    """
    was_ranging = True
    try:
        with i2c_bus.LOCK:
            sensor.stop_ranging()
    except OSError:
        was_ranging = False

    # Zero offset and crosstalk so we measure raw error.
    set_offset(0)
    with i2c_bus.LOCK:
        sensor._write_register(_REG_XTALK_KCPS, struct.pack(">h", 0))
        sensor.start_ranging()

    readings = []
    deadline = time.monotonic() + max(2.0, samples * 0.2)
    while len(readings) < samples and time.monotonic() < deadline:
        with i2c_bus.LOCK:
            ready = sensor.data_ready
            if ready:
                d_mm = sensor.distance * 10.0
                st = sensor.range_status
                sensor.clear_interrupt()
        if ready and st in _VALID_STATUSES:
            readings.append(d_mm)
        time.sleep(0.01)

    with i2c_bus.LOCK:
        sensor.stop_ranging()

    if len(readings) < max(5, samples // 2):
        print(f"WARN: only {len(readings)} valid samples; offset cal aborted.")
        if was_ranging:
            with i2c_bus.LOCK:
                sensor.start_ranging()
        return None

    avg = sum(readings) / len(readings)
    offset = target_mm - avg
    set_offset(offset)
    print(
        f"INFO: measured avg {avg:.1f} mm at target {target_mm} mm -> "
        f"offset {offset:+.1f} mm programmed."
    )
    if was_ranging:
        with i2c_bus.LOCK:
            sensor.start_ranging()
    return offset


def cleanup():
    """Stop ranging and release XSHUT."""
    print("--- Cleaning up ToF Sensor (VL53L4CD) ---")
    if sensor is not None:
        try:
            with i2c_bus.LOCK:
                sensor.stop_ranging()
        except OSError:
            print("Warning: I/O error during VL53L4CD cleanup. Ignoring.")
    _release_xshut()


if __name__ == "__main__":
    import sys

    print("--- Testing VL53L4CD (i2c-1, GPIO2/3, breakout pull-ups removed) ---")
    if not initialize():
        print("VL53L4CD test failed during initialization.")
        sys.exit(1)

    # `python3 -m src.sensors.vl53l4cd calibrate [target_mm]`
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        target = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
        print(f"Current offset: {get_offset():+.1f} mm")
        input(
            f"Place a flat matte target squarely at {target:.0f} mm, then press Enter..."
        )
        calibrate_offset(target)
        print(f"New offset: {get_offset():+.1f} mm. Re-run without 'calibrate' to test.")
        cleanup()
        sys.exit(0)

    try:
        print(f"\nCurrent offset: {get_offset():+.1f} mm")
        print("Read (distance + status). Press Ctrl+C to stop.")
        while True:
            d = get_new_distance()
            if d is not None:
                print(f"\rdist={d / 10.0:6.1f} cm  [VALID]      ", end="")
            else:
                print("\rdist=  ----    [no valid target] ", end="")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        cleanup()
