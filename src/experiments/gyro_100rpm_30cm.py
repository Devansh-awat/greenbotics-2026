#!/usr/bin/env python3
"""
gyro_100rpm_30cm.py

Hardware Experiment Script for Greenbotics 2025 WRO Future Engineers Robot.
"""

import sys
import time
import math
import numpy as np

# Import hardware drivers from src
try:
    from src.motors import motor, servo
    from src.sensors import distance
    from src.obstacle_challenge import config
    
    # Try importing BNO086, fall back to BNO055 if needed
    try:
        from src.sensors import bno086 as imu_module
    except ImportError:
        from src.sensors import bno055 as imu_module
except ImportError as err:
    print(f"FATAL: Failed to import hardware drivers from src: {err}")
    print("Ensure you are running this script from the greenbotics repository root.")
    sys.exit(1)


# --- Parameters ---
TARGET_RPM = 100.0         # Target wheel RPM (100 RPM)
TARGET_DIST_CM = 50.0      # Drive target distance: 50 cm
LOOP_HZ = 50.0             # Control loop frequency (50 Hz = 20 ms)
BRAKE_MONITOR_HZ = 100.0   # High-freq monitoring during coast-down (100 Hz = 10 ms)
KP_GYRO = getattr(config, 'GYRO_KP', 2.5)  # Proportional gain for gyro straight steering (2.5 - 3.0)
KD_GYRO = 0.10             # Derivative gain for gyro straight steering
SERVO_MIN_ANGLE = -45.0    # Servo left limit
SERVO_MAX_ANGLE = 45.0     # Servo right limit
MAX_RUN_TIME_SEC = 10.0    # Safety timeout limit (seconds)


def normalize_angle_error(error: float) -> float:
    """Normalizes angle error to [-180, +180] degrees."""
    while error <= -180.0:
        error += 360.0
    while error > 180.0:
        error -= 360.0
    return error


def compute_gyro_steer(current_heading: float, target_heading: float, prev_error: float) -> tuple[float, float]:
    """Calculates servo angle to hold target heading using PD control."""
    if current_heading is None or target_heading is None:
        return 0.0, prev_error
    error = normalize_angle_error(target_heading - current_heading)
    derivative = error - prev_error
    steer_angle = (KP_GYRO * error) + (KD_GYRO * derivative)
    clamped_angle = float(np.clip(steer_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE))
    return clamped_angle, error


def get_safe_heading(target_heading: float, last_valid_heading: float) -> float:
    """Reads IMU heading and filters out uninitialized 0.0° or transient jump glitches (>40°)."""
    raw_h = imu_module.get_heading()
    if raw_h is None:
        return last_valid_heading if last_valid_heading is not None else target_heading

    ref = last_valid_heading if last_valid_heading is not None else target_heading
    delta = abs(normalize_angle_error(raw_h - ref))

    if delta > 40.0:
        return ref

    return raw_h


def run_experiment():
    print("==========================================================")
    print("      GYRO 100 RPM DRIVE + 50CM DISTANCE EXPERIMENT       ")
    print("==========================================================")
    print(f"Target Wheel Speed: {TARGET_RPM:.1f} RPM")
    print(f"Target Distance   : {TARGET_DIST_CM:.1f} cm")
    print(f"Control Frequency : {LOOP_HZ:.0f} Hz")
    print(f"Safety Timeout    : {MAX_RUN_TIME_SEC:.1f} s")
    print("----------------------------------------------------------")

    # 1. Initialize Motor & Encoder
    print("[1/4] Initializing Motor & Encoder...")
    if not motor.initialize():
        print("FATAL: Failed to initialize motor driver.")
        return False

    if not motor.encoder:
        print("ERROR: Encoder not available on motor! Cannot measure distance/speed.")
        motor.cleanup()
        return False

    # 2. Initialize Steering Servo
    print("[2/4] Initializing Steering Servo...")
    if not servo.initialize():
        print("FATAL: Failed to initialize servo driver.")
        motor.cleanup()
        return False

    # 3. Initialize Gyro (IMU)
    print("[3/4] Initializing Gyro IMU...")
    if not imu_module.initialize():
        print("WARNING: IMU failed to initialize or disabled. Servo will remain centered.")
        gyro_available = False
        target_heading = 0.0
    else:
        gyro_available = True
        time.sleep(0.3)
        target_heading = imu_module.get_initial_heading(num_readings=20)
        print(f"INFO: Baseline Target Heading set to {target_heading:.2f}°")

    # 4. Initialize Distance Sensors
    print("[4/4] Initializing Distance Sensors...")
    if not distance.initialise():
        print("WARNING: Distance sensor initialization reported errors.")

    cpr = motor.encoder.counts_per_rev                    # counts/wheel rev (~540-617)
    wheel_diameter_mm = motor.encoder.wheel_diameter_mm  # 62.4 mm
    wheel_circ_m = (math.pi * wheel_diameter_mm) / 1000.0  # ~0.196 m per rev

    initial_front_mm = distance.get_distance(distance.FRONT_CHANNEL)
    initial_front_cm = (initial_front_mm / 10.0) if initial_front_mm is not None else None
    print(f"INFO: Initial Front Distance: {initial_front_cm:.1f} cm" if initial_front_cm is not None else "INFO: Initial Front Distance: None")

    print("\nStarting Drive Phase in 1 second... (Press Ctrl+C to abort)")
    time.sleep(1.0)

    samples = []
    start_time = time.perf_counter()
    last_time = start_time
    start_pos = motor.encoder.position
    last_pos = start_pos

    prev_error = 0.0
    last_valid_heading = target_heading
    stop_reason = "Unknown"
    
    motor.start_rpm_control(TARGET_RPM, direction="forward")

    print("\n" + "-" * 90)
    print(f"{'Time(s)':<8} | {'Heading(°)':<10} | {'Steer(°)':<9} | {'Wheel RPM':<10} | {'Speed m/s':<10} | {'Travel(cm)':<11} | {'Front Dist(cm)':<15}")
    print("-" * 90)

    loop_interval = 1.0 / LOOP_HZ
    step_index = 0

    t_stop_signal = None
    pos_at_stop_signal = None
    dist_at_stop_signal_cm = None
    speed_at_stop_signal_mps = 0.0

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start_time
            if elapsed >= MAX_RUN_TIME_SEC:
                stop_reason = f"Safety timeout ({MAX_RUN_TIME_SEC:.1f}s)"
                t_stop_signal = now
                pos_at_stop_signal = motor.encoder.position
                break

            dt = now - last_time
            if dt < (loop_interval * 0.75):
                time.sleep(0.002)
                continue
            last_time = now

            if gyro_available:
                curr_heading = get_safe_heading(target_heading, last_valid_heading)
                last_valid_heading = curr_heading
            else:
                curr_heading = target_heading

            steer_angle, prev_error = compute_gyro_steer(curr_heading, target_heading, prev_error)
            servo.set_angle(steer_angle)

            curr_pos = motor.encoder.position
            delta_counts = curr_pos - last_pos
            last_pos = curr_pos

            total_dist_cm = motor.counts_to_cm(curr_pos - start_pos, "forward") or 0.0

            inst_wheel_rpm = (delta_counts / cpr) * (60.0 / dt) if dt > 0 else 0.0
            inst_speed_mps = (delta_counts / cpr) * wheel_circ_m / dt if dt > 0 else 0.0

            front_mm = distance.get_distance(distance.FRONT_CHANNEL)
            front_cm = (front_mm / 10.0) if front_mm is not None else None

            samples.append({
                "time": elapsed,
                "dt": dt,
                "heading": curr_heading,
                "steer": steer_angle,
                "pos": curr_pos,
                "wheel_rpm": inst_wheel_rpm,
                "speed_mps": inst_speed_mps,
                "front_cm": front_cm,
                "travel_cm": total_dist_cm
            })

            step_index += 1
            if step_index % 2 == 0:
                front_str = f"{front_cm:15.1f}" if front_cm is not None else f"{'None':>15}"
                print(f"{elapsed:<8.2f} | {curr_heading:<10.2f} | {steer_angle:<9.2f} | {inst_wheel_rpm:<10.1f} | {inst_speed_mps:<10.3f} | {total_dist_cm:<11.2f} | {front_str}")

            if total_dist_cm >= TARGET_DIST_CM:
                stop_reason = f"Target distance reached ({total_dist_cm:.1f} cm >= {TARGET_DIST_CM:.1f} cm)"
                t_stop_signal = now
                pos_at_stop_signal = curr_pos
                dist_at_stop_signal_cm = total_dist_cm
                speed_at_stop_signal_mps = inst_speed_mps
                break

    except KeyboardInterrupt:
        print("\n[!] Ctrl+C Interrupted by user.")
        stop_reason = "Interrupted by user (Ctrl+C)"

    finally:
        if t_stop_signal is None:
            t_stop_signal = time.perf_counter()
            pos_at_stop_signal = motor.encoder.position if motor.encoder else start_pos
            speed_at_stop_signal_mps = samples[-1]["speed_mps"] if samples else 0.0

        motor.stop_rpm_control()
        motor.brake()
        servo.set_angle(0.0)

    print("\n" + "=" * 90)
    print(f"STOP TRIGGERED: {stop_reason}")
    print(f"Speed at Stop Signal: {speed_at_stop_signal_mps:.3f} m/s ({speed_at_stop_signal_mps * 3.6:.2f} km/h)")
    if dist_at_stop_signal_cm is not None:
        print(f"Encoder Distance at Stop Signal: {dist_at_stop_signal_cm:.2f} cm")
    print("=" * 90)

    brake_start_time = t_stop_signal
    last_brake_time = brake_start_time
    last_brake_pos = pos_at_stop_signal
    zero_motion_count = 0

    brake_interval = 1.0 / BRAKE_MONITOR_HZ
    max_brake_timeout = 2.0

    while True:
        now_brake = time.perf_counter()
        t_elapsed_brake = now_brake - brake_start_time
        if t_elapsed_brake >= max_brake_timeout:
            break

        dt_brake = now_brake - last_brake_time
        if dt_brake < (brake_interval * 0.75):
            time.sleep(0.001)
            continue
        last_brake_time = now_brake

        curr_brake_pos = motor.encoder.position
        delta_brake_counts = curr_brake_pos - last_brake_pos
        last_brake_pos = curr_brake_pos

        if delta_brake_counts == 0:
            zero_motion_count += 1
        else:
            zero_motion_count = 0

        if zero_motion_count >= 5:
            break

    t_stop_complete = time.perf_counter()
    stop_duration_sec = t_stop_complete - t_stop_signal

    final_pos = motor.encoder.position
    coast_counts = abs(final_pos - pos_at_stop_signal)
    coast_dist_cm = motor.counts_to_cm(coast_counts, "forward") or 0.0
    total_run_dist_cm = motor.counts_to_cm(final_pos - start_pos, "forward") or 0.0

    time.sleep(0.1)
    final_front_mm = distance.get_distance(distance.FRONT_CHANNEL)
    final_front_cm = (final_front_mm / 10.0) if final_front_mm is not None else None

    if samples:
        avg_speed_mps = sum(s["speed_mps"] for s in samples) / len(samples)
        max_speed_mps = max(s["speed_mps"] for s in samples)
        avg_rpm = sum(s["wheel_rpm"] for s in samples) / len(samples)
    else:
        avg_speed_mps = max_speed_mps = avg_rpm = 0.0

    print("\n" + "█" * 60)
    print("            EXPERIMENT SUMMARY & TELEMETRY REPORT           ")
    print("█" * 60)
    print(f"Stop Trigger Reason        : {stop_reason}")
    print(f"Total Run Duration         : {t_stop_complete - start_time:.3f} s")
    print(f"Average Drive Speed        : {avg_speed_mps:.3f} m/s ({avg_rpm:.1f} Wheel RPM)")
    print(f"Max Drive Speed            : {max_speed_mps:.3f} m/s")
    print(f"Speed at Brake Trigger     : {speed_at_stop_signal_mps:.3f} m/s")
    print("-" * 60)
    print(f"Target Distance            : {TARGET_DIST_CM:.1f} cm")
    print(f"Distance @ Stop Signal     : {dist_at_stop_signal_cm:.2f} cm" if dist_at_stop_signal_cm is not None else "Distance @ Stop Signal     : N/A")
    print(f"STOPPING TIME (to 0 RPM)   : {stop_duration_sec:.4f} seconds ({stop_duration_sec * 1000.0:.1f} ms)")
    print(f"COASTING DISTANCE (braking): {coast_dist_cm:.2f} cm ({coast_counts} encoder ticks)")
    print(f"TOTAL TRAVEL DISTANCE      : {total_run_dist_cm:.2f} cm")
    print("-" * 60)
    print(f"Initial Front Distance     : {initial_front_cm:.1f} cm" if initial_front_cm is not None else "Initial Front Distance     : N/A")
    print(f"Final Front Dist @ Stopped : {final_front_cm:.1f} cm" if final_front_cm is not None else "Final Front Dist @ Stopped : N/A")
    print("█" * 60 + "\n")

    distance.cleanup() if hasattr(distance, "cleanup") else None
    if gyro_available and hasattr(imu_module, "cleanup"):
        imu_module.cleanup()
    servo.cleanup()
    motor.cleanup()

    print("Experiment completed cleanly.")
    return True


if __name__ == "__main__":
    try:
        run_experiment()
    except Exception as e:
        print(f"Unhandled exception during experiment: {e}")
        import traceback
        traceback.print_exc()
        try:
            servo.cleanup()
            motor.cleanup()
        except Exception:
            pass
        sys.exit(1)
