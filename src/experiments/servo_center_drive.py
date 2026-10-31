#!/usr/bin/env python3
"""
servo_center_drive.py

Hardware experiment for tuning SERVO_CENTER_OFFSET: drive forward with the steering
at 0 for a fixed time, then brake and report how far the gyro heading drifted. A
centered servo drifts ~0 deg. After each run you can type a new offset and run again.

Heading decreases when the robot turns left (see obstacle runs: CCW turns go 360 ->
305), so a negative drift means it veered left -> raise the offset; positive drift
-> lower it.

Run from the repo root:  python3 -m src.experiments.servo_center_drive [--offset 10]
"""

import argparse
import sys
import time

from src.motors import motor, servo
from src.obstacle_challenge import config
from src.sensors import bno086

# --- Parameters ---
DRIVE_RPM = 150.0          # Wheel rpm target, closed loop
DRIVE_TIME_S = 2.5         # Drive this long per run
LOOP_HZ = 200.0            # Heading poll rate
BRAKE_HOLD_S = 0.5         # Keep the motor shorted this long after each run
SERVO_SETTLE_S = 0.5       # Time for the servo to reach its angle before driving


def heading_change(current, start):
    """Signed shortest heading change from start to current, in (-180, 180]."""
    return (current - start + 180.0) % 360.0 - 180.0


def read_heading(timeout_s=2.0):
    """Wait for a valid IMU heading, or return None after timeout_s."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        h = bno086.get_heading()
        if h is not None:
            return h
        time.sleep(0.02)
    return None


def drive_once(rpm, drive_time_s):
    """Drive straight at steering 0; return the signed heading drift in degrees, or None."""
    servo.set_angle(0)
    time.sleep(SERVO_SETTLE_S)

    start = read_heading()
    if start is None:
        print("FATAL: no IMU heading.")
        return None

    t0 = time.monotonic()
    if not motor.start_rpm_control(rpm, direction="forward"):
        print("FATAL: RPM control did not start (encoder not initialized?).")
        return None
    drift = 0.0
    next_print = 0.0
    try:
        while time.monotonic() - t0 < drive_time_s:
            h = bno086.get_heading()
            if h is not None:
                drift = heading_change(h, start)
            t = time.monotonic() - t0
            if t >= next_print:
                print(f"t={t:4.2f}s drift={drift:+6.1f} rpm={motor.get_measured_rpm():5.0f}")
                next_print = t + 0.25
            time.sleep(1.0 / LOOP_HZ)
    finally:
        motor.stop_rpm_control()   # brakes
        time.sleep(BRAKE_HOLD_S)
    return drift


def main():
    parser = argparse.ArgumentParser(description="Drive straight at steering 0 to tune SERVO_CENTER_OFFSET")
    parser.add_argument("--offset", type=float, default=config.SERVO_CENTER_OFFSET,
                        help=f"servo center offset to start with (default: config value {config.SERVO_CENTER_OFFSET})")
    parser.add_argument("--rpm", type=float, default=DRIVE_RPM, help=f"wheel rpm (default {DRIVE_RPM:.0f})")
    parser.add_argument("--time", type=float, default=DRIVE_TIME_S, help=f"drive seconds per run (default {DRIVE_TIME_S})")
    args = parser.parse_args()

    config.SERVO_CENTER_OFFSET = args.offset

    bno086.initialize()
    motor.initialize()
    if servo.initialize() is False:
        print("FATAL: servo failed to initialize.")
        return 1

    try:
        while True:
            cmd = input(f"\nOffset {config.SERVO_CENTER_OFFSET:+.1f}. Enter = drive, <num> = new offset, q = quit: ").strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd:
                try:
                    config.SERVO_CENTER_OFFSET = float(cmd)
                except ValueError:
                    print("  ? unrecognized input")
                    continue
                servo.set_angle(0)
                print(f"  offset set to {config.SERVO_CENTER_OFFSET:+.1f}; place the robot and press Enter to drive")
                continue

            drift = drive_once(args.rpm, args.time)
            if drift is None:
                return 1
            if abs(drift) < 1.0:
                hint = "looks centered"
            elif drift < 0:
                hint = "veered LEFT -> raise the offset"
            else:
                hint = "veered RIGHT -> lower the offset"
            print(f"Offset {config.SERVO_CENTER_OFFSET:+.1f}: drift {drift:+.1f} deg over {args.time:.1f}s ({hint})")

        print(f"\nFinal: set SERVO_CENTER_OFFSET = {config.SERVO_CENTER_OFFSET:g} in src/obstacle_challenge/config.py")
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nInterrupted.")
        return 0
    finally:
        motor.stop_rpm_control()
        motor.brake()
        servo.set_angle(0)
        time.sleep(BRAKE_HOLD_S)
        motor.cleanup()
        servo.cleanup()
        bno086.cleanup()


if __name__ == "__main__":
    sys.exit(main())
