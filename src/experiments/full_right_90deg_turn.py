#!/usr/bin/env python3
"""
full_right_90deg_turn.py

Hardware experiment: steer fully right, drive forward at ~max wheel rpm until the gyro
says the robot has turned 90 degrees from where it started, then brake and
straighten the wheels. Handy for measuring the minimum turning circle.

Run from the repo root:  python3 -m src.experiments.full_right_90deg_turn
"""

import sys
import time

from src.motors import motor, servo
from src.sensors import bno086
from src.obstacle_challenge import config

# --- Parameters ---
TURN_TARGET_DEG = 90.0     # Stop once the heading has changed by this much
DRIVE_RPM = 615            # Wheel rpm target (max is ~616), closed loop
LOOP_HZ = 100.0            # Heading poll rate
BRAKE_HOLD_S = 1.0         # Keep the motor shorted this long; motor.cleanup() releases it to coast
SERVO_SETTLE_S = 0.5       # Time for the servo to reach full lock before driving
MAX_RUN_TIME_S = 8.0       # Safety timeout in case the heading never reaches the target


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


def run_experiment():
    print("Full-right 90 degree turn at max rpm")

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

        servo.set_angle(config.INPUT_ANGLE_MAX_SERVO)   # clamped to full right
        time.sleep(SERVO_SETTLE_S)

        t0 = time.monotonic()
        if not motor.start_rpm_control(DRIVE_RPM):
            print("FATAL: RPM control did not start (encoder not initialized?).")
            return 1
        turned = 0.0
        reason = "timeout"
        next_print = 0.0
        while time.monotonic() - t0 < MAX_RUN_TIME_S:
            h = bno086.get_heading()
            if h is not None:
                turned = heading_change(h, start)
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
    sys.exit(run_experiment())
