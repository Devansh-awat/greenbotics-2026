#!/usr/bin/env python3
"""
left_turn_forward_90deg.py

Hardware experiment: steer to -50 deg (left), drive forward at 80 wheel rpm until the
gyro says the robot has turned 90 degrees from where it started, then brake and
straighten the wheels.

The reverse variant (left_turn_reverse_90deg.py) calls run_experiment("reverse") and
does the same motion in the other direction.

Run from the repo root:  python3 -m src.experiments.left_turn_forward_90deg
"""

import sys
import time

from src.motors import motor, servo
from src.sensors import bno086

# --- Parameters ---
STEER_ANGLE = -50.0        # Servo input angle; negative is full left
TURN_TARGET_DEG = 80.0     # Stop once the heading has changed by this much (either sign)
DRIVE_RPM = 60.0           # Wheel rpm target, closed loop
LOOP_HZ = 400.0            # Heading poll rate
BRAKE_HOLD_S = 1.0         # Keep the motor shorted this long; motor.cleanup() releases it to coast
SERVO_SETTLE_S = 0.5       # Time for the servo to reach its angle before driving
MAX_RUN_TIME_S = 10.0      # Safety timeout in case the heading never reaches the target


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


def run_experiment(direction="forward"):
    print(f"Left -50 deg turn, {direction}, {DRIVE_RPM:.0f} rpm, until {TURN_TARGET_DEG:.0f} deg")

    bno086.initialize()
    motor.initialize()
    if servo.initialize() is False:
        print("FATAL: servo failed to initialize.")
        return 1

    try:
        start = read_heading()
        if start is None:
            print("FATAL: no IMU heading.")
            return 1
        print(f"Start heading: {start:.1f} deg")

        servo.set_angle(STEER_ANGLE)
        time.sleep(SERVO_SETTLE_S)

        t0 = time.monotonic()
        if not motor.start_rpm_control(DRIVE_RPM, direction=direction):
            print("FATAL: RPM control did not start (encoder not initialized?).")
            return 1
        turned = 0.0
        reason = "timeout"
        next_print = 0.0
        while time.monotonic() - t0 < MAX_RUN_TIME_S:
            h = bno086.get_heading()
            if h is not None:
                turned = heading_change(h, start)
                # Absolute value: reversing with the same steering turns the other way.
                if abs(turned) >= TURN_TARGET_DEG:
                    reason = "target reached"
                    break
            t = time.monotonic() - t0
            if t >= next_print:
                print(f"t={t:4.2f}s heading={h if h is None else round(h, 1)} "
                      f"turned={turned:+6.1f} rpm={motor.get_measured_rpm():5.0f}")
                next_print = t + 0.25
            time.sleep(1.0 / LOOP_HZ)
        elapsed = time.monotonic() - t0

        motor.stop_rpm_control()   # brakes
        print(f"Stopped ({reason}) after {elapsed:.2f}s, turned {turned:+.1f} deg")
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
    sys.exit(run_experiment("forward"))
