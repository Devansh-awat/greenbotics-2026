#!/usr/bin/env python3
"""
speed_sweep_dist_stop.py

Hardware Speed Sweep Experiment Script for Greenbotics 2025 WRO Future Engineers Robot.

Fixes applied:
1. Reversing Steering Inversion:
   - Inverted steering correction during reverse mode (`direction="reverse"`) so gyro corrections
     actively straighten the car instead of amplifying drift into curves.
2. Exact Return to Start Position (Encoder + ToF Double Check):
   - Uses encoder position (`start_pos`) captured before forward move to guarantee the vehicle
     reverses back to the exact physical starting point every run.
   - Requires 3 consecutive valid ToF readings above threshold to prevent single-frame ToF glitches
     from stopping reverse prematurely.
3. Robust Gyro Heading Lock & KeyboardInterrupt safety.
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


# --- Sweep Parameters ---
START_RPM = 60.0           # Initial sweep speed (60 RPM)
STEP_RPM = 20.0            # Speed increment step per run (+20 RPM)
MAX_TARGET_RPM = round(0.80 * motor.MAX_WHEEL_RPM) # 80% of max RPM (~490 RPM)
STOP_TRIGGER_DIST_CM = 20.0 # Stop when front distance <= 20 cm
PAUSE_AFTER_STOP_SEC = 2.0 # Pause 2 seconds after stopping
REVERSE_RPM = 75.0         # Speed when reversing back between runs

LOOP_HZ = 50.0             # Control loop frequency (50 Hz = 20 ms)
BRAKE_MONITOR_HZ = 100.0   # High-freq monitoring during coast-down (100 Hz = 10 ms)
KP_GYRO = getattr(config, 'GYRO_KP', 2.5)  # Proportional gain for gyro straight steering (2.5 - 3.0)
KD_GYRO = 0.10             # Derivative gain for gyro straight steering
SERVO_MIN_ANGLE = -45.0    # Servo left limit
SERVO_MAX_ANGLE = 45.0     # Servo right limit
MAX_RUN_TIME_SEC = 15.0    # Safety timeout limit per run (seconds)


def normalize_angle_error(error: float) -> float:
    """Normalizes angle error to [-180, +180] degrees."""
    while error <= -180.0:
        error += 360.0
    while error > 180.0:
        error -= 360.0
    return error


def compute_gyro_steer(current_heading: float, target_heading: float, prev_error: float, direction: str = "forward") -> tuple[float, float]:
    """Calculates servo angle to hold target heading using PD control."""
    if current_heading is None or target_heading is None:
        return 0.0, prev_error
    error = normalize_angle_error(target_heading - current_heading)
    derivative = error - prev_error
    steer_angle = (KP_GYRO * error) + (KD_GYRO * derivative)

    # In reverse, inverting steering angle is required for Ackermann steering geometry
    # to turn back toward target heading rather than amplifying drift into curves.
    if direction == "reverse":
        steer_angle = -steer_angle

    clamped_angle = float(np.clip(steer_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE))
    return clamped_angle, error


def get_safe_heading(target_heading: float, last_valid_heading: float) -> float:
    """Reads IMU heading and filters out uninitialized 0.0° or transient jump glitches (>40°)."""
    raw_h = imu_module.get_heading()
    if raw_h is None:
        return last_valid_heading if last_valid_heading is not None else target_heading

    ref = last_valid_heading if last_valid_heading is not None else target_heading
    delta = abs(normalize_angle_error(raw_h - ref))

    # Reject sudden single-frame spikes (>40° jump)
    if delta > 40.0:
        return ref

    return raw_h


def run_single_speed_test(run_idx: int, target_rpm: float, gyro_available: bool, target_heading: float, initial_start_front_cm: float):
    print("\n" + "=" * 90)
    print(f"  RUN #{run_idx}: TARGET SPEED = {target_rpm:.1f} RPM  (Stop Trigger = {STOP_TRIGGER_DIST_CM:.1f} cm)")
    print("=" * 90)

    cpr = motor.encoder.counts_per_rev
    wheel_diameter_mm = motor.encoder.wheel_diameter_mm
    wheel_circ_m = (math.pi * wheel_diameter_mm) / 1000.0

    # Record starting front distance & start encoder position for this run
    start_front_mm = distance.get_distance(distance.FRONT_CHANNEL)
    start_front_cm = (start_front_mm / 10.0) if start_front_mm is not None else initial_start_front_cm
    start_pos = motor.encoder.position
    print(f"INFO: Run #{run_idx} Starting Front Distance: {start_front_cm:.1f} cm (Start Pos: {start_pos})" if start_front_cm is not None else f"INFO: Run #{run_idx} Starting Front Distance: N/A (Start Pos: {start_pos})")

    samples = []
    start_time = time.perf_counter()
    last_time = start_time
    last_pos = start_pos

    prev_error = 0.0
    last_valid_heading = target_heading
    stop_reason = "Unknown"

    t_stop_signal = None
    pos_at_stop_signal = None
    dist_at_stop_signal_cm = None
    speed_at_stop_signal_mps = 0.0

    # Start closed-loop speed control
    motor.start_rpm_control(target_rpm, direction="forward")

    print("-" * 90)
    print(f"{'Time(s)':<8} | {'Heading(°)':<10} | {'Steer(°)':<9} | {'Front Dist(cm)':<15} | {'Wheel RPM':<10} | {'Speed m/s':<10} | {'Travel(cm)':<10}")
    print("-" * 90)

    loop_interval = 1.0 / LOOP_HZ
    step_index = 0

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

            # Read IMU with glitch filter & update steering
            if gyro_available:
                curr_heading = get_safe_heading(target_heading, last_valid_heading)
                last_valid_heading = curr_heading
            else:
                curr_heading = target_heading

            steer_angle, prev_error = compute_gyro_steer(curr_heading, target_heading, prev_error, direction="forward")
            servo.set_angle(steer_angle)

            # Read Encoder Position
            curr_pos = motor.encoder.position
            delta_counts = curr_pos - last_pos
            last_pos = curr_pos

            inst_wheel_rpm = (delta_counts / cpr) * (60.0 / dt) if dt > 0 else 0.0
            inst_speed_mps = (delta_counts / cpr) * wheel_circ_m / dt if dt > 0 else 0.0
            total_dist_cm = motor.counts_to_cm(curr_pos - start_pos, "forward") or 0.0

            # Read Front Distance Sensor
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
                print(f"{elapsed:<8.2f} | {curr_heading:<10.2f} | {steer_angle:<9.2f} | {front_str} | {inst_wheel_rpm:<10.1f} | {inst_speed_mps:<10.3f} | {total_dist_cm:<10.2f}")

            # Trigger condition (<= 20 cm)
            if front_cm is not None and front_cm <= STOP_TRIGGER_DIST_CM:
                stop_reason = f"Front sensor threshold reached ({front_cm:.1f} cm <= {STOP_TRIGGER_DIST_CM:.1f} cm)"
                t_stop_signal = now
                pos_at_stop_signal = curr_pos
                dist_at_stop_signal_cm = front_cm
                speed_at_stop_signal_mps = inst_speed_mps
                break

    finally:
        if t_stop_signal is None:
            t_stop_signal = time.perf_counter()
            pos_at_stop_signal = motor.encoder.position if motor.encoder else start_pos
            speed_at_stop_signal_mps = samples[-1]["speed_mps"] if samples else 0.0

        motor.stop_rpm_control()
        motor.brake()
        servo.set_angle(0.0)

    print("\n" + "-" * 90)
    print(f"STOP SIGNAL TRIGGERED: {stop_reason}")
    print(f"Speed @ Brake Signal: {speed_at_stop_signal_mps:.3f} m/s ({speed_at_stop_signal_mps * 3.6:.2f} km/h)")
    if dist_at_stop_signal_cm is not None:
        print(f"Front ToF Distance @ Brake Signal: {dist_at_stop_signal_cm:.1f} cm")

    # High-freq monitoring of braking coast-down
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
    stop_duration_ms = stop_duration_sec * 1000.0

    # Final encoder position & coasting distance
    final_pos = motor.encoder.position
    coast_counts = abs(final_pos - pos_at_stop_signal)
    coast_dist_cm = motor.counts_to_cm(coast_counts, "forward") or 0.0

    # Read final ToF front distance when stopped
    time.sleep(0.1)
    final_front_mm = distance.get_distance(distance.FRONT_CHANNEL)
    final_front_cm = (final_front_mm / 10.0) if final_front_mm is not None else None

    # Calculate ToF Distance Delta
    if dist_at_stop_signal_cm is not None and final_front_cm is not None:
        tof_delta_cm = dist_at_stop_signal_cm - final_front_cm
    else:
        tof_delta_cm = None

    # Calculate average pre-brake speed
    if samples:
        avg_speed_mps = sum(s["speed_mps"] for s in samples) / len(samples)
        max_speed_mps = max(s["speed_mps"] for s in samples)
        avg_rpm = sum(s["wheel_rpm"] for s in samples) / len(samples)
    else:
        avg_speed_mps = max_speed_mps = avg_rpm = 0.0

    # Single Run Report
    print("-" * 90)
    print(f"RUN #{run_idx} RESULTS:")
    print(f"  - Target RPM                  : {target_rpm:.1f} RPM")
    print(f"  - Measured Avg Speed          : {avg_speed_mps:.3f} m/s ({avg_rpm:.1f} RPM)")
    print(f"  - Speed @ Brake Signal        : {speed_at_stop_signal_mps:.3f} m/s")
    print(f"  - STOPPING TIME (to 0 RPM)    : {stop_duration_ms:.1f} ms ({stop_duration_sec:.4f} s)")
    print(f"  - Encoder Coast Distance Delta: {coast_dist_cm:.2f} cm ({coast_counts} ticks)")
    if tof_delta_cm is not None:
        print(f"  - ToF Sensor Distance Delta   : {tof_delta_cm:.2f} cm (from {dist_at_stop_signal_cm:.1f} cm -> {final_front_cm:.1f} cm)")
    else:
        print(f"  - ToF Sensor Distance Delta   : N/A")
    print("-" * 90)

    # Return structured metrics dictionary
    return {
        "run_idx": run_idx,
        "target_rpm": target_rpm,
        "avg_rpm": avg_rpm,
        "avg_speed_mps": avg_speed_mps,
        "max_speed_mps": max_speed_mps,
        "speed_at_brake_mps": speed_at_stop_signal_mps,
        "stop_duration_ms": stop_duration_ms,
        "encoder_delta_cm": coast_dist_cm,
        "tof_delta_cm": tof_delta_cm,
        "front_at_brake_cm": dist_at_stop_signal_cm,
        "final_front_cm": final_front_cm,
        "start_front_cm": start_front_cm,
        "start_pos": start_pos
    }


def reverse_back_to_start(gyro_available: bool, target_heading: float, start_pos: int, target_start_cm: float):
    """Reverses back using gyro straight reverse until encoder returns to start_pos (double-checked with ToF)."""
    print(f"\n[<--] Reversing back to starting position (Start Encoder Pos: {start_pos}, ~{target_start_cm:.1f} cm ToF)...")
    motor.start_rpm_control(REVERSE_RPM, direction="reverse")

    start_time = time.perf_counter()
    prev_error = 0.0
    last_valid_heading = target_heading
    valid_tof_count = 0

    try:
        while True:
            now = time.perf_counter()
            if now - start_time > 15.0:
                print("WARNING: Reverse timeout reached (15s). Stopping reverse move.")
                break

            # Read IMU with glitch filter & update reverse steering (inverted sign)
            if gyro_available:
                curr_heading = get_safe_heading(target_heading, last_valid_heading)
                last_valid_heading = curr_heading
            else:
                curr_heading = target_heading

            steer_angle, prev_error = compute_gyro_steer(curr_heading, target_heading, prev_error, direction="reverse")
            servo.set_angle(steer_angle)

            # Check Encoder position: Primary stop condition when wheel returns to starting encoder position
            curr_pos = motor.encoder.position if motor.encoder else None
            encoder_reached = (curr_pos is not None and curr_pos <= (start_pos + 15))

            # Check ToF distance: Require 3 consecutive valid samples above target threshold to avoid noise glitches
            front_mm = distance.get_distance(distance.FRONT_CHANNEL)
            if front_mm is not None:
                front_cm = front_mm / 10.0
                if front_cm >= (target_start_cm - 3.0):
                    valid_tof_count += 1
                else:
                    valid_tof_count = 0
            else:
                valid_tof_count = 0

            tof_reached = (valid_tof_count >= 3)

            # Stop reverse when encoder target is reached OR 3 consecutive valid ToF readings confirm start position
            if encoder_reached or tof_reached:
                reason = "Encoder start position reached" if encoder_reached else f"ToF distance confirmed ({valid_tof_count} samples)"
                print(f"INFO: Reached starting position ({reason}). Current Encoder Pos: {curr_pos}, Target: {start_pos}")
                break

            time.sleep(0.02)
    finally:
        motor.stop_rpm_control()
        motor.brake()
        servo.set_angle(0.0)
        print("[<--] Reverse complete. Holding position.")


def run_experiment():
    print("==========================================================")
    print("  MULTI-SPEED SWEEP DRIVE & BRAKING PERFORMANCE EXPERIMENT")
    print("==========================================================")
    print(f"Start Speed       : {START_RPM:.1f} RPM")
    print(f"Speed Step        : +{STEP_RPM:.1f} RPM per run")
    print(f"Max Target Speed  : {MAX_TARGET_RPM:.1f} RPM (80% max)")
    print(f"Front Stop Trigger: {STOP_TRIGGER_DIST_CM:.1f} cm")
    print(f"Pause After Stop  : {PAUSE_AFTER_STOP_SEC:.1f} seconds")
    print("----------------------------------------------------------")

    # Initializations
    print("[1/4] Initializing Motor & Encoder...")
    if not motor.initialize() or not motor.encoder:
        print("FATAL: Failed to initialize motor or encoder.")
        return False

    print("[2/4] Initializing Steering Servo...")
    if not servo.initialize():
        print("FATAL: Failed to initialize servo.")
        motor.cleanup()
        return False

    print("[3/4] Initializing Gyro IMU...")
    if not imu_module.initialize():
        print("WARNING: IMU failed to initialize or disabled. Servo will remain centered.")
        gyro_available = False
        target_heading = 0.0
    else:
        gyro_available = True
        time.sleep(0.3)
        target_heading = imu_module.get_initial_heading(num_readings=20)
        print(f"INFO: Baseline Target Heading locked at {target_heading:.2f}°")

    print("[4/4] Initializing Distance Sensors...")
    if not distance.initialise():
        print("WARNING: Distance sensor initialization reported errors.")

    initial_front_mm = distance.get_distance(distance.FRONT_CHANNEL)
    initial_front_cm = (initial_front_mm / 10.0) if initial_front_mm is not None else 115.0
    print(f"INFO: Baseline Starting Front Distance: {initial_front_cm:.1f} cm")

    print("\nStarting Automated Speed Sweep in 2 seconds... (Press Ctrl+C to abort)")
    time.sleep(2.0)

    all_runs_metrics = []
    target_rpm = START_RPM
    run_idx = 1

    try:
        while target_rpm <= MAX_TARGET_RPM:
            # Execute single speed run
            metrics = run_single_speed_test(run_idx, target_rpm, gyro_available, target_heading, initial_front_cm)
            all_runs_metrics.append(metrics)

            # Pause 2 seconds while stopped at wall
            print(f"\n[||] Pausing for {PAUSE_AFTER_STOP_SEC:.1f} seconds at front wall...")
            time.sleep(PAUSE_AFTER_STOP_SEC)

            # Reverse back to start if there are more RPM steps remaining
            next_rpm = target_rpm + STEP_RPM
            if next_rpm <= MAX_TARGET_RPM:
                reverse_target_cm = metrics["start_front_cm"] if metrics["start_front_cm"] is not None else initial_front_cm
                start_pos = metrics["start_pos"]
                reverse_back_to_start(gyro_available, target_heading, start_pos, reverse_target_cm)
                print("Pausing 1 second before next speed run...")
                time.sleep(1.0)

            target_rpm = next_rpm
            run_idx += 1

    except KeyboardInterrupt:
        print("\n[!] Ctrl+C Interrupted by user. Cleaning up and printing partial summary...")

    finally:
        motor.stop_rpm_control()
        motor.brake()
        servo.set_angle(0.0)

    # Master Comparative Telemetry Summary
    if all_runs_metrics:
        print("\n" + "█" * 105)
        print("                      MULTI-SPEED SWEEP BRAKING PERFORMANCE COMPARISON REPORT                   ")
        print("█" * 105)
        print(f"{'Run#':<5} | {'Target RPM':<10} | {'Avg Speed':<11} | {'Speed @ Brake':<15} | {'Stopping Time':<15} | {'Encoder Coast':<15} | {'ToF Dist Delta':<15}")
        print(f"{'':<5} | {'(RPM)':<10} | {'(m/s)':<11} | {'(m/s)':<15} | {'(ms)':<15} | {'(cm)':<15} | {'(cm)':<15}")
        print("-" * 105)

        for m in all_runs_metrics:
            tof_str = f"{m['tof_delta_cm']:15.2f}" if m['tof_delta_cm'] is not None else f"{'N/A':>15}"
            print(f"{m['run_idx']:<5} | {m['target_rpm']:<10.1f} | {m['avg_speed_mps']:<11.3f} | {m['speed_at_brake_mps']:<15.3f} | {m['stop_duration_ms']:<15.1f} | {m['encoder_delta_cm']:<15.2f} | {tof_str}")

        print("█" * 105 + "\n")

    # Teardown
    distance.cleanup() if hasattr(distance, "cleanup") else None
    if gyro_available and hasattr(imu_module, "cleanup"):
        imu_module.cleanup()
    servo.cleanup()
    motor.cleanup()

    print("Multi-speed sweep experiment completed cleanly.")
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
