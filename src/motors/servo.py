from rpi_hardware_pwm import HardwarePWM
import time
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
            time.sleep(0.5)
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
    Sets the servo to a specific angle, RESPECTING the software limits
    in the config file (e.g., -45 to +45 degrees).
    """
    if servo_pwm is None:
        return

    adjusted_angle = input_angle + config.SERVO_CENTER_OFFSET
    clamped_input = max(
        config.INPUT_ANGLE_MIN_SERVO, min(config.INPUT_ANGLE_MAX_SERVO, adjusted_angle)
    )
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

    unclamped_input = input_angle + config.SERVO_CENTER_OFFSET

    input_range = config.INPUT_ANGLE_MAX_SERVO - config.INPUT_ANGLE_MIN_SERVO
    output_range = config.CALIBRATED_ANGLE_MAX - config.CALIBRATED_ANGLE_MIN
    target_output_angle = (
        config.CALIBRATED_ANGLE_MIN
        + ((unclamped_input - config.INPUT_ANGLE_MIN_SERVO) / input_range)
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


if __name__ == "__main__":
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

        # Use set_angle_unlimited so the printed angle maps directly to the
        # commanded position (set_angle adds a +5 trim offset, hiding center).
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
