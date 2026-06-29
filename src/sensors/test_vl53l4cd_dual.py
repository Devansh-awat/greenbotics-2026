# -*- coding: utf-8 -*-
"""
Bring up TWO VL53L4CD sensors on the same i2c bus (bus = vl53l4cd.I2C_BUS).

Both chips power up at the default address 0x29, so they can't coexist as-is.
Standard fix: hold both in reset via their XSHUT pins, then enable them one at a
time and reassign each to a unique address before enabling the next.

    SDA/SCL shared -> the I2C_BUS pins (i2c-3 = GPIO14/15; needs pull-ups!)
    XSHUT A -> GPIO17      XSHUT B -> GPIO27

Sequence:
  1. Drive both XSHUT low  -> both chips in reset, bus is clear.
  2. Raise XSHUT A         -> only A is awake at 0x29; reassign it to 0x2A.
  3. Raise XSHUT B         -> only B is now new at 0x29; reassign it to 0x2B.
  4. Range both and print distances side by side.

The bring-up (constructor + set_address) is retried because the bus is marginal
when a breakout's pull-ups aren't removed (see vl53l4cd.py header). Remove the
pull-ups from BOTH breakouts for reliability — each one loads the bus even while
its sensor is held in XSHUT reset.

Run from repo root:  python3 -m src.sensors.test_vl53l4cd_dual
"""

import time
from gpiozero import DigitalOutputDevice
import adafruit_vl53l4cd

from src.sensors.vl53l4cd import _ExtendedI2C, I2C_BUS, _VALID_STATUSES

XSHUT_A_PIN = 17
XSHUT_B_PIN = 27
ADDR_A = 0x2A
ADDR_B = 0x2B

BOOT_DELAY_S = 0.2  # settle time for a chip to boot after XSHUT goes high
BRING_UP_RETRIES = 6


def _bring_up(i2c, xshut, new_address, label):
    """Wake one sensor (others must be in reset), move it to new_address.

    Each attempt does a full XSHUT reset (low -> high) so the chip boots fresh:
    a chip left ranging / killed mid-transaction by a previous run gets wedged
    (ACKs its address but stalls every data transfer), and only a reset edge
    clears it — re-probing alone won't. A reset also returns the chip to the
    default 0x29, so re-probing 0x29 is correct on every attempt.

    Catches BOTH ValueError (the constructor's probe failure) and OSError (a
    later register access failing); the Adafruit driver raises either.
    """
    last_err = None
    for _ in range(BRING_UP_RETRIES):
        xshut.off()
        time.sleep(0.05)
        xshut.on()              # clean reset edge -> fresh boot at 0x29
        time.sleep(BOOT_DELAY_S)
        try:
            s = adafruit_vl53l4cd.VL53L4CD(i2c)  # responds at default 0x29
            s.set_address(new_address)
            s.timing_budget = 50
            s.inter_measurement = 0
            s.start_ranging()
            print(f"  - {label}: up at 0x{new_address:02X}")
            return s
        except (OSError, ValueError) as e:
            last_err = e
    raise RuntimeError(f"{label}: bring-up failed after retries: {last_err}")


def main():
    i2c = _ExtendedI2C(I2C_BUS)

    # initial_value=False -> both held low (in reset) from the start.
    xshut_a = DigitalOutputDevice(XSHUT_A_PIN, initial_value=False)
    xshut_b = DigitalOutputDevice(XSHUT_B_PIN, initial_value=False)
    time.sleep(0.05)

    sensor_a = sensor_b = None
    try:
        print(f"Bringing up two VL53L4CD on i2c bus {I2C_BUS}...")
        sensor_a = _bring_up(i2c, xshut_a, ADDR_A, "Sensor A (XSHUT GPIO17)")
        sensor_b = _bring_up(i2c, xshut_b, ADDR_B, "Sensor B (XSHUT GPIO27)")

        print("\nReading both. Press Ctrl+C to stop.")
        while True:
            out = ""
            for name, s in (("A", sensor_a), ("B", sensor_b)):
                try:
                    if s.data_ready:
                        d = s.distance
                        st = s.range_status
                        s.clear_interrupt()
                        if st in _VALID_STATUSES:
                            out += f"{name}: {d:6.1f} cm  "
                        else:
                            out += f"{name}:  --(st{st})  "
                    else:
                        out += f"{name}:  ....    "
                except OSError:
                    # transient i2c glitch — skip this frame, keep running
                    out += f"{name}:  err     "
            print(f"\r{out}", end="")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        for s in (sensor_a, sensor_b):
            if s is not None:
                try:
                    s.stop_ranging()
                except OSError:
                    pass
        xshut_a.close()
        xshut_b.close()


if __name__ == "__main__":
    main()
