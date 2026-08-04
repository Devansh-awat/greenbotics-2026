"""Heading maths and the gyro-stabilised drive primitives.

`steer_with_gyro()` is a pure function -- give it a current and target heading and
it returns a servo angle. The `drive_straight_*` helpers are blocking: they own the
motor for the duration of the move and brake when done.
"""

import time

import numpy as np

from src.motors import motor, servo
from src.obstacle_challenge.logsetup import log
from src.obstacle_challenge.tuning import GYRO_KD, GYRO_KP

# Set by main_v5 via bind(); see maneuvers.py for why this is injected rather than
# imported.
imu_thread = None


def bind(**kwargs):
    """Inject the run-level objects this module reads (imu_thread)."""
    globals().update(kwargs)

def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)

def _heading_hold_step(target_heading, prev_error, kp, kd, limit=45):
    """One PD step of the heading lock. Returns (servo_angle, error).

    Returns (None, prev_error) when the IMU has no reading yet, so the caller can
    hold the current steering rather than snapping the wheels to centre.
    """
    current_heading = imu_thread.get_heading()
    if current_heading is None:
        return None, prev_error

    error = target_heading - current_heading
    while error <= -180: error += 360
    while error > 180: error -= 360

    derivative = error - prev_error
    steer_angle = (kp * error) + (kd * derivative)
    return float(np.clip(steer_angle, -limit, limit)), error


def drive_straight_with_gyro(target_heading, duration, speed, direction='forward', kp=GYRO_KP, kd=GYRO_KD):
    """Drive for a fixed TIME at a fixed DUTY, holding `target_heading`.

    BUG: open-loop on both axes -- `speed` is a raw PWM duty, and the move ends on
    a stopwatch. How far the robot actually goes then depends on battery voltage,
    ramp/carpet, and how hard the wheels are cranked over, which is why the same
    `duration` gives a different distance run to run. Migrate callers to
    drive_distance_with_gyro() below, which closes the loop on the encoder for
    distance and on rpm for speed. Kept for now so existing tuning still runs.
    """
    log.info("Driving %s with gyro stabilization for %.2fs (target %.1f°, speed %s)",
             direction, duration, target_heading, speed)

    start_time = time.monotonic()

    # BUG: direct duty. Replace with motor.start_rpm_control(rpm, direction).
    if direction == 'forward':
        motor.forward(speed)
    else:
        motor.reverse(speed)

    prev_error = 0.0

    while time.monotonic() - start_time < duration:
        steer_angle, prev_error = _heading_hold_step(target_heading, prev_error, kp, kd)
        if steer_angle is None:
            time.sleep(0.01)
            continue
        servo.set_angle(steer_angle)
        time.sleep(0.01)

    motor.brake()
    servo.set_angle(0)
    log.debug("Gyro-stabilized drive complete.")


def drive_distance_with_gyro(target_heading, distance_cm, rpm=60.0, direction='forward',
                             kp=GYRO_KP, kd=GYRO_KD, max_servo_angle=45,
                             timeout=None, stop_check=None):
    """Drive a measured DISTANCE while holding `target_heading`. Encoder-closed.

    The distance-based replacement for drive_straight_with_gyro(): the encoder
    says when to stop instead of a stopwatch, and motor's closed-loop rpm
    controller holds the speed instead of a fixed PWM duty. Battery sag, carpet
    and steering load therefore change how LONG the move takes, not how FAR it
    goes -- which is the property the maneuvers actually want.

        target_heading: heading (deg) to hold for the whole move.
        distance_cm:    how far to travel. Always positive; `direction` picks the way.
        rpm:            target WHEEL rpm. Keep at/above ~40 (motor.MAX_WHEEL_RPM is
                        455) -- below the stall floor the controller pulses and the
                        motion is stop-start.
        direction:      'forward' or 'reverse'.
        kp, kd:         heading-lock PD gains.
        max_servo_angle: steering clamp, same convention as steer_with_gyro().
        timeout:        seconds before giving up. Defaults to 3x the time the move
                        should take, plus 2 s of startup slack. A slipping or
                        stalled wheel must not be able to loop forever.
        stop_check:     optional callable; the move aborts early when it returns
                        True (e.g. a ToF reading going below a limit).

    Returns the distance actually travelled (cm), which is <= distance_cm if the
    move timed out or was stopped early. Returns None if the encoder is missing --
    check for that rather than assuming the move happened.
    """
    start_pos = motor.encoder_position()
    if start_pos is None:
        log.error("No encoder: drive_distance_with_gyro(%.1f cm) refused. "
                  "Not falling back to a timed move -- that would drive blind.", distance_cm)
        return None

    target_cm = abs(distance_cm)
    if timeout is None:
        speed_cm_per_s = (motor.cm_per_minute_at(rpm) or 0.0) / 60.0
        timeout = (3.0 * target_cm / speed_cm_per_s + 2.0) if speed_cm_per_s > 0 else 10.0

    log.info("Driving %s %.1f cm @ %.0f rpm with gyro lock on %.1f° (timeout %.1fs)",
             direction, target_cm, rpm, target_heading, timeout)

    motor.stop_rpm_control()
    motor.start_rpm_control(rpm, direction)

    prev_error = 0.0
    travelled = 0.0
    start_time = time.monotonic()
    reason = "distance reached"
    try:
        while True:
            travelled = motor.counts_to_cm(motor.encoder_position() - start_pos, direction)
            if travelled >= target_cm:
                break
            if time.monotonic() - start_time > timeout:
                reason = "TIMEOUT"
                break
            if stop_check is not None and stop_check():
                reason = "stop_check"
                break

            steer_angle, prev_error = _heading_hold_step(
                target_heading, prev_error, kp, kd, limit=max_servo_angle)
            if steer_angle is not None:
                servo.set_angle(steer_angle)
            time.sleep(0.005)
    finally:
        motor.stop_rpm_control()
        motor.brake()
        servo.set_angle(0)

    # Read once more after braking: coast-down is real and the caller wants the
    # distance the robot ENDED at, not the one it decided to stop at.
    final = motor.counts_to_cm(motor.encoder_position() - start_pos, direction)
    level = log.warning if reason == "TIMEOUT" else log.info
    level("Distance drive done (%s): asked %.1f cm, travelled %.1f cm in %.2fs",
          reason, target_cm, final, time.monotonic() - start_time)
    return final


def steer_with_gyro(current_heading: float,
                    target_heading: float,
                    kp: float = GYRO_KP,
                    kd: float = GYRO_KD,
                    prev_error: float = 0.0,
                    min_servo_angle: int = -45,
                    max_servo_angle: int = 45) -> float:
    """
    Calculates the servo steering angle to maintain a target heading using a gyro PD controller.

    This function uses a Proportional-Derivative (PD) controller to correct the robot's
    trajectory. It correctly handles the "wrap-around" issue where headings
    jump between 359 and 0 degrees.

    Args:
        current_heading (float): The current direction of the robot in degrees (0-359).
        target_heading (float): The desired direction in degrees (0-359).
        kp (float): The proportional gain constant.
        kd (float): The derivative gain constant.
        prev_error (float): The previous heading error.
        min_servo_angle (int): The minimum allowable angle for the servo (e.g., full left).
        max_servo_angle (int): The maximum allowable angle for the servo (e.g., full right).

    Returns:
        float: The calculated servo angle, clamped within the min/max limits.
    """
    if current_heading is None or target_heading is None:
        return 0.0

    # 1. Calculate the error between target and current heading.
    error = target_heading - current_heading

    # 2. Handle the wrap-around issue (e.g., turning from 350 degrees to 10 degrees).
    # The shortest path is not -340 degrees, but +20 degrees.
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    # 3. Calculate derivative and steering correction (PD controller).
    derivative = error - prev_error
    steer_angle = (kp * error) + (kd * derivative)

    # 4. Clamp the steering angle to the servo's physical limits.
    clamped_steer_angle = np.clip(steer_angle, min_servo_angle, max_servo_angle)

    return clamped_steer_angle
