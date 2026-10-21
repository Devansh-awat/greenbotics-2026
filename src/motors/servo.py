from rpi_hardware_pwm import HardwarePWM
import time
import numpy as np
from src.obstacle_challenge import config


servo_pwm = None
_MAX_PWM_RETRIES = 5
_PWM_RETRY_DELAY = 0.1

def initialize():
    """Initializes the servo motor PWM with retries in case the kernel is busy."""
    global servo_pwm

    last_error = None
    for attempt in range(1, _MAX_PWM_RETRIES + 1):
        try:
            servo_pwm = HardwarePWM(
                pwm_channel=config.SERVO_PWM_CHANNEL,
                hz=config.SERVO_PWM_FREQ,
                chip=config.SERVO_PWM_CHIP,
            )
            servo_pwm.start(0)
            set_angle(0.0)
            print("INFO: Servo Initialized.")
            return True
        except PermissionError as err:
            last_error = err
            print(
                f"WARNING: Servo PWM permission denied (attempt {attempt}/{_MAX_PWM_RETRIES}): {err}"
            )
            time.sleep(_PWM_RETRY_DELAY * attempt)
        except Exception as err:
            last_error = err
            print(f"FATAL: Servo failed to initialize: {err}")
            time.sleep(_PWM_RETRY_DELAY * attempt)

    print(f"FATAL: Servo failed to initialize after {_MAX_PWM_RETRIES} attempts: {last_error}")
    return False


def set_angle(input_angle: float):
    """
    Sets the servo to a specific angle, RESPECTING the software steering limits
    (+-45 degrees either side of center; applied before the center offset so
    left and right travel stay equal).
    """
    if servo_pwm is None:
        return

    if getattr(config, "SERVO_DIR_INVERT", False):
        input_angle = -input_angle

    clamped_input = max(-45.0, min(45.0, input_angle)) + config.SERVO_CENTER_OFFSET
    input_range = config.INPUT_ANGLE_MAX_SERVO - config.INPUT_ANGLE_MIN_SERVO
    output_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    target_output_angle = (
        config.CALIBRATED_ANGLE_MIN
        + ((clamped_input - config.INPUT_ANGLE_MIN_SERVO) / input_range) * output_range
    )

    cal_angle_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    cal_pulse_range = config.CALIBRATED_MAX_PW_S - config.CALIBRATED_MIN_PW_S
    target_pulse_s = (
        config.CALIBRATED_MIN_PW_S
        + ((target_output_angle - config.CALIBRATED_ANGLE_MIN) / cal_angle_range)
        * cal_pulse_range
    )

    clamped_pw_s = max(
        config.SAFETY_MIN_PW_S, min(config.SAFETY_MAX_PW_S, target_pulse_s)
    )
    duty_cycle = (clamped_pw_s / config.SERVO_PWM_PERIOD_S) * 100.0
    servo_pwm.change_duty_cycle(max(0.0, min(100.0, duty_cycle)))


def set_angle_unlimited(input_angle: float):
    """
    Sets the servo to a specific angle, BYPASSING the software limits.
    This is for special maneuvers like parking that need sharper turns.
    It is still protected by the hardware safety pulse width limits.
    """
    if servo_pwm is None:
        return

    if getattr(config, "SERVO_DIR_INVERT", False):
        input_angle = -input_angle

    # +-50 degrees either side of center, applied before the offset so the
    # limits stay symmetric about the true center.
    clamped_input = float(np.clip(input_angle, -50.0, 50.0)) + config.SERVO_CENTER_OFFSET

    input_range = config.INPUT_ANGLE_MAX_SERVO - config.INPUT_ANGLE_MIN_SERVO
    output_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    target_output_angle = (
        config.CALIBRATED_ANGLE_MIN
        + ((clamped_input - config.INPUT_ANGLE_MIN_SERVO) / input_range)
        * output_range
    )

    cal_angle_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    cal_pulse_range = config.CALIBRATED_MAX_PW_S - config.CALIBRATED_MIN_PW_S
    target_pulse_s = (
        config.CALIBRATED_MIN_PW_S
        + ((target_output_angle - config.CALIBRATED_ANGLE_MIN) / cal_angle_range)
        * cal_pulse_range
    )

    clamped_pw_s = max(
        config.SAFETY_MIN_PW_S, min(config.SAFETY_MAX_PW_S, target_pulse_s)
    )
    duty_cycle = (clamped_pw_s / config.SERVO_PWM_PERIOD_S) * 100.0
    servo_pwm.change_duty_cycle(max(0.0, min(100.0, duty_cycle)))


def cleanup():
    """Centers the servo and stops PWM."""
    print("--- Cleaning up Servo ---")
    if servo_pwm:
        set_angle(0.0)
        time.sleep(0.5)
        servo_pwm.change_duty_cycle(0)
        time.sleep(4.0 / config.SERVO_PWM_FREQ)
        servo_pwm.stop()


def _center_tool():
    print("--- Servo Center-Finding Tool ---")
    if not initialize():
        print("Servo test failed during initialization.")
    else:
        # Smallest step the servo can meaningfully resolve. Toggle between
        # 0.5 and 1.0 with 's' to home in on the true mechanical center.
        STEP_OPTIONS = [0.5, 1.0]
        step_idx = 0
        step = STEP_OPTIONS[step_idx]
        angle = 0.0

        # Angles here are relative to SERVO_CENTER_OFFSET (both set_angle and
        # set_angle_unlimited add it). If center is found at angle X, the new
        # offset is SERVO_CENTER_OFFSET + X.
        set_angle_unlimited(angle)

        print(
            "Controls:\n"
            "  a / <- : nudge LEFT by current step\n"
            "  d / -> : nudge RIGHT by current step\n"
            "  s      : toggle step size (0.5 <-> 1.0)\n"
            "  0      : jump to 0\n"
            "  <num>  : jump to an absolute angle (e.g. -2.5)\n"
            "  q      : quit\n"
        )
        print(f"angle = {angle:+.1f}   step = {step}")

        try:
            while True:
                cmd = input("> ").strip().lower()
                if cmd in ("q", "quit", "exit"):
                    break
                elif cmd in ("a", "left"):
                    angle -= step
                elif cmd in ("d", "right"):
                    angle += step
                elif cmd == "s":
                    step_idx = (step_idx + 1) % len(STEP_OPTIONS)
                    step = STEP_OPTIONS[step_idx]
                elif cmd == "":
                    pass  # repeat / refresh
                else:
                    try:
                        angle = float(cmd)
                    except ValueError:
                        print("  ? unrecognized input")
                        continue
                set_angle_unlimited(angle)
                print(f"angle = {angle:+.1f}   step = {step}")
        except (KeyboardInterrupt, EOFError):
            print("\nInterrupted.")
        finally:
            set_angle_unlimited(0)
            time.sleep(0.5)
            cleanup()


def _sweep(limit_deg: float, speed_deg_s: float):
    """Sweep smoothly between -limit and +limit until Ctrl+C."""
    print(f"--- Servo Sweep: +-{limit_deg:.1f} deg at {speed_deg_s:.0f} deg/s (Ctrl+C to stop) ---")
    if not initialize():
        print("Servo test failed during initialization.")
        return

    # One command per PWM frame; faster writes would just be dropped.
    period = 1.0 / config.SERVO_PWM_FREQ
    step = speed_deg_s * period
    angle = 0.0
    direction = 1.0
    try:
        set_angle_unlimited(angle)
        next_t = time.monotonic()
        while True:
            angle += direction * step
            if angle >= limit_deg:
                angle, direction = limit_deg, -1.0
                print(f"  reached +{limit_deg:.1f}")
            elif angle <= -limit_deg:
                angle, direction = -limit_deg, 1.0
                print(f"  reached -{limit_deg:.1f}")
            set_angle_unlimited(angle)
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        set_angle_unlimited(0)
        time.sleep(0.5)
        cleanup()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Servo center-finding / sweep tool")
    parser.add_argument("-s", "--sweep", action="store_true",
                        help="sweep back and forth instead of the interactive center tool")
    parser.add_argument("--limit", type=float, default=45.0,
                        help="sweep limit in degrees either side of center (default 45)")
    parser.add_argument("--speed", type=float, default=90.0,
                        help="sweep speed in deg/s (default 90)")
    args = parser.parse_args()

    if args.sweep:
        _sweep(args.limit, args.speed)
    else:
        _center_tool()
