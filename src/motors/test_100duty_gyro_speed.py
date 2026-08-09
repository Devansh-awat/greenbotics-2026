#!/usr/bin/env python3
"""
test_100duty_gyro_speed.py

Hardware Test Script for Greenbotics 2025 WRO Future Engineers Robot.

Behavior:
1. Initializes motor, servo, encoder, and BNO086 IMU (gyro).
2. Captures initial heading as reference target for driving straight.
3. Drives FORWARD at 100% duty cycle for 1.5 seconds while holding heading using gyro P-control.
4. Continuously measures encoder counts, wheel/motor RPM, and linear velocity (m/s, cm/s).
5. Applies a 5-sample (100 ms) rolling moving average for smoothed real-time telemetry.
6. Displays real-time metrics during run and prints a comprehensive summary upon completion.
7. Safely brakes the motor, centers the servo, and cleans up all GPIO/hardware drivers.

Run from repo root:
    python3 -m src.motors.test_100duty_gyro_speed
"""

import sys
import time
import math
import signal
import argparse
import numpy as np

# Import hardware drivers from src
try:
    from src.motors import motor, servo
    from src.sensors import bno086
    from src.obstacle_challenge import config
except ImportError as err:
    print(f"FATAL: Failed to import hardware drivers from src: {err}")
    print("Ensure you are running this script from the greenbotics repository root.")
    sys.exit(1)


# --- Default Parameters ---
TARGET_DUTY = 100.0       # 100% PWM duty cycle
RUN_DURATION_SEC = 1.5    # Default drive duration in seconds (1.5s)
LOOP_HZ = 50.0            # 50 Hz control / measurement loop (20 ms interval)
MA_WINDOW_SIZE = 5        # Moving average window size (5 samples = 100 ms smoothing)
KP_GYRO = 0.85            # Proportional gain for gyro straight steering
SERVO_MIN_ANGLE = -45.0   # Servo left limit
SERVO_MAX_ANGLE = 45.0    # Servo right limit

# External gear ratio: 13T motor pinion to 38T spur gear on wheel
MOTOR_TO_WHEEL_RATIO = 38.0 / 13.0  # ~2.923


def normalize_angle_error(error: float) -> float:
    """Normalizes angle error to [-180, +180] degrees."""
    while error <= -180.0:
        error += 360.0
    while error > 180.0:
        error -= 360.0
    return error


def steer_with_gyro(current_heading: float, target_heading: float, kp: float = KP_GYRO) -> float:
    """Calculates servo angle to hold target heading using proportional control."""
    if current_heading is None or target_heading is None:
        return 0.0
    error = normalize_angle_error(target_heading - current_heading)
    steer_angle = kp * error
    return float(np.clip(steer_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE))


def run_test(duty: float = TARGET_DUTY, duration: float = RUN_DURATION_SEC):
    print("==========================================================")
    print("  100% DUTY + GYRO (BNO086) STRAIGHT + RPM & SPEED TEST   ")
    print("==========================================================")
    print(f"Target Duty Cycle: {duty:.1f}%")
    print(f"Target Duration  : {duration:.2f} seconds")
    print(f"Control Loop Freq: {LOOP_HZ:.0f} Hz")
    print(f"MA Smoothing     : {MA_WINDOW_SIZE} samples ({MA_WINDOW_SIZE/LOOP_HZ*1000:.0f} ms)")
    print("----------------------------------------------------------")

    # 1. Initialize Motor & Encoder
    print("[1/4] Initializing Motor & Encoder...")
    if not motor.initialize():
        print("FATAL: Failed to initialize motor driver.")
        return False

    if not motor.encoder:
        print("ERROR: Encoder not available on motor! Speed/RPM cannot be measured.")
        motor.cleanup()
        return False

    # 2. Initialize Steering Servo
    print("[2/4] Initializing Steering Servo...")
    if not servo.initialize():
        print("FATAL: Failed to initialize servo driver.")
        motor.cleanup()
        return False

    # 3. Initialize Gyro (BNO086 IMU)
    print("[3/4] Initializing Gyro (BNO086)...")
    if not bno086.initialize():
        print("WARNING: BNO086 Gyro failed to initialize or disabled. Will maintain 0° servo angle.")
        gyro_available = False
        target_heading = 0.0
    else:
        gyro_available = True
        target_heading = bno086.get_initial_heading(num_readings=15)
        print(f"INFO: Baseline Target Heading set to {target_heading:.2f}°")

    # Encoder specifications
    cpr = motor.encoder.counts_per_rev                    # ~540.18 counts/wheel rev
    wheel_diameter_mm = motor.encoder.wheel_diameter_mm  # 62.4 mm
    wheel_circ_m = (math.pi * wheel_diameter_mm) / 1000.0  # ~0.196 m

    print("\n[4/4] Starting Drive Phase in 1 second...")
    time.sleep(1.0)

    # Variables for tracking run metrics
    samples = []
    
    # Rolling buffers for moving average calculation
    win_wheel_rpm = []
    win_motor_rpm = []
    win_speed_mps = []

    start_time = time.perf_counter()
    last_time = start_time
    start_pos = motor.encoder.position
    last_pos = start_pos
    start_dist_mm = motor.encoder.distance
    last_dist_mm = start_dist_mm

    # Engage motor forward at 100% duty
    motor.forward(duty)

    print("\n" + "-" * 85)
    print(f"{'Time(s)':<8} | {'Heading(°)':<10} | {'Steer(°)':<9} | {'Wheel RPM (MA)':<15} | {'Motor RPM (MA)':<15} | {'Speed m/s (MA)':<15} | {'Dist(cm)':<8}")
    print("-" * 85)

    loop_interval = 1.0 / LOOP_HZ

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start_time
            if elapsed >= duration:
                break

            dt = now - last_time
            if dt < 0.015:
                time.sleep(0.002)
                continue

            # Read current BNO086 Heading & update steering
            if gyro_available:
                curr_heading = bno086.get_heading()
                if curr_heading is None:
                    curr_heading = target_heading
            else:
                curr_heading = target_heading

            steer_angle = steer_with_gyro(curr_heading, target_heading)
            servo.set_angle(steer_angle)

            # Read Encoder Position and Distance
            curr_pos = motor.encoder.position
            curr_dist_mm = motor.encoder.distance

            delta_counts = curr_pos - last_pos
            delta_dist_m = (curr_dist_mm - last_dist_mm) / 1000.0

            # Calculate Instantaneous Metrics
            inst_wheel_rpm = (delta_counts / cpr) * (60.0 / dt)
            inst_motor_rpm = inst_wheel_rpm * MOTOR_TO_WHEEL_RATIO
            inst_speed_mps = delta_dist_m / dt
            inst_speed_cmps = inst_speed_mps * 100.0
            cum_dist_cm = (curr_dist_mm - start_dist_mm) / 10.0

            # Update Rolling Window for Moving Average
            win_wheel_rpm.append(inst_wheel_rpm)
            win_motor_rpm.append(inst_motor_rpm)
            win_speed_mps.append(inst_speed_mps)

            if len(win_wheel_rpm) > MA_WINDOW_SIZE:
                win_wheel_rpm.pop(0)
                win_motor_rpm.pop(0)
                win_speed_mps.pop(0)

            ma_wheel_rpm = sum(win_wheel_rpm) / len(win_wheel_rpm)
            ma_motor_rpm = sum(win_motor_rpm) / len(win_motor_rpm)
            ma_speed_mps = sum(win_speed_mps) / len(win_speed_mps)

            # Record sample
            samples.append({
                "time": elapsed,
                "dt": dt,
                "heading": curr_heading,
                "steer": steer_angle,
                "pos": curr_pos,
                "wheel_rpm_raw": inst_wheel_rpm,
                "motor_rpm_raw": inst_motor_rpm,
                "speed_mps_raw": inst_speed_mps,
                "wheel_rpm_ma": ma_wheel_rpm,
                "motor_rpm_ma": ma_motor_rpm,
                "speed_mps_ma": ma_speed_mps,
                "speed_cmps": ma_speed_mps * 100.0,
                "dist_cm": cum_dist_cm,
            })

            # Print realtime output with moving average values
            print(f"{elapsed:6.3f}s  | {curr_heading:8.2f}°  | {steer_angle:7.2f}°  | {ma_wheel_rpm:14.1f}  | {ma_motor_rpm:14.1f}  | {ma_speed_mps:14.3f}   | {cum_dist_cm:7.2f}")

            last_time = now
            last_pos = curr_pos
            last_dist_mm = curr_dist_mm

            # Sleep remaining loop time
            computation_time = time.perf_counter() - now
            sleep_time = max(0.001, loop_interval - computation_time)
            time.sleep(sleep_time)

    finally:
        end_time = time.perf_counter()
        actual_duration = end_time - start_time

        # Immediate Brake and Cleanup
        motor.brake()
        servo.set_angle(0.0)
        time.sleep(0.1)

        final_pos = motor.encoder.position if motor.encoder else last_pos
        final_dist_mm = motor.encoder.distance if motor.encoder else last_dist_mm
        
        motor.cleanup()
        servo.cleanup()
        if gyro_available:
            bno086.cleanup()

    print("-" * 85)
    print("\n==========================================================")
    print("                    TEST SUMMARY REPORT                   ")
    print("==========================================================")

    total_counts = final_pos - start_pos
    total_revs = total_counts / cpr
    total_dist_m = (final_dist_mm - start_dist_mm) / 1000.0
    total_dist_cm = total_dist_m * 100.0

    avg_wheel_rpm = (total_revs / actual_duration) * 60.0 if actual_duration > 0 else 0.0
    avg_motor_rpm = avg_wheel_rpm * MOTOR_TO_WHEEL_RATIO
    avg_speed_mps = total_dist_m / actual_duration if actual_duration > 0 else 0.0
    avg_speed_cmps = avg_speed_mps * 100.0

    wheel_rpms_ma = [s["wheel_rpm_ma"] for s in samples if s["wheel_rpm_ma"] > 0]
    motor_rpms_ma = [s["motor_rpm_ma"] for s in samples if s["motor_rpm_ma"] > 0]
    speeds_mps_ma = [s["speed_mps_ma"] for s in samples if s["speed_mps_ma"] > 0]

    peak_wheel_rpm_ma = max(wheel_rpms_ma) if wheel_rpms_ma else 0.0
    peak_motor_rpm_ma = max(motor_rpms_ma) if motor_rpms_ma else 0.0
    peak_speed_mps_ma = max(speeds_mps_ma) if speeds_mps_ma else 0.0
    peak_speed_cmps_ma = peak_speed_mps_ma * 100.0

    final_heading = samples[-1]["heading"] if samples else target_heading
    heading_drift = normalize_angle_error(final_heading - target_heading)

    print(f"Actual Duration     : {actual_duration:.3f} s")
    print(f"Total Encoder Counts: {total_counts} counts")
    print(f"Total Wheel Revs    : {total_revs:.3f} revs")
    print(f"Total Distance      : {total_dist_cm:.2f} cm ({total_dist_m:.3f} m)")
    print("----------------------------------------------------------")
    print(f"Average Wheel Speed : {avg_wheel_rpm:.1f} RPM")
    print(f"Peak Wheel Speed(MA): {peak_wheel_rpm_ma:.1f} RPM")
    print(f"Average Motor Speed : {avg_motor_rpm:.1f} RPM")
    print(f"Peak Motor Speed(MA): {peak_motor_rpm_ma:.1f} RPM")
    print("----------------------------------------------------------")
    print(f"Average Speed       : {avg_speed_mps:.3f} m/s ({avg_speed_cmps:.1f} cm/s)")
    print(f"Peak Speed (MA)     : {peak_speed_mps_ma:.3f} m/s ({peak_speed_cmps_ma:.1f} cm/s)")
    print("----------------------------------------------------------")
    if gyro_available:
        print(f"Target Heading      : {target_heading:.2f}°")
        print(f"Final Heading       : {final_heading:.2f}°")
        print(f"Heading Drift Error : {heading_drift:+.2f}°")
    print("==========================================================\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="100% Duty Gyro-Straight & Speed Test")
    parser.add_argument("-d", "--duration", type=float, default=RUN_DURATION_SEC, help="Run duration in seconds (default: 1.5s)")
    parser.add_argument("-p", "--duty", type=float, default=TARGET_DUTY, help="Motor duty cycle % (default: 100.0%)")
    args = parser.parse_args()

    # Setup Ctrl+C handler
    def sigint_handler(sig, frame):
        print("\n\n[!] KeyboardInterrupt caught. Emergency stopping motor and servo...")
        try:
            motor.brake()
            servo.set_angle(0.0)
            motor.cleanup()
            servo.cleanup()
            bno086.cleanup()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    run_test(duty=args.duty, duration=args.duration)


if __name__ == "__main__":
    main()
