"""
Quick bench test for newly-wired servo + DC motor (no encoder).

Sweeps the servo through its software-limited range and pulses the motor
forward, then reverse, repeating until Ctrl+C or the stop button is pressed.

A momentary stop button is wired to BUTTON_PIN (GPIO23). The pin is held high
(3.3V) by the internal pull-up configured in /boot/firmware/config.txt, so it
idles at 1 and reads 0 when the button shorts it to ground.
"""

import time

import lgpio

from src.motors import motor, servo
from src.obstacle_challenge import config


SERVO_ANGLES = [0, 30, 0, -30, 0]
SERVO_DWELL_S = 0.6

MOTOR_SPEED = 50
MOTOR_RUN_S = 1.5
MOTOR_PAUSE_S = 0.5

# How often to poll the stop button while waiting.
BUTTON_POLL_S = 0.02


class StopRequested(Exception):
    """Raised when the stop button is pressed."""


_button_handle = None


def init_button():
    """Claims the stop button pin as an input. Pull-up comes from config.txt."""
    global _button_handle
    try:
        _button_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(_button_handle, config.BUTTON_PIN)
        print(f"INFO: Stop button ready on GPIO{config.BUTTON_PIN}.")
        return True
    except Exception as err:
        print(f"WARNING: Stop button failed to initialize: {err}")
        _button_handle = None
        return False


def button_pressed():
    """Returns True when the button is pressed (pin pulled low)."""
    if _button_handle is None:
        return False
    return lgpio.gpio_read(_button_handle, config.BUTTON_PIN) == 0


def check_stop():
    """Raises StopRequested if the button is pressed."""
    if button_pressed():
        raise StopRequested


def sleep_unless_stopped(duration_s):
    """Sleeps for duration_s, polling the button and stopping early if pressed."""
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        check_stop()
        time.sleep(min(BUTTON_POLL_S, max(0.0, end - time.monotonic())))
    check_stop()


def cleanup_button():
    """Releases the stop button GPIO."""
    if _button_handle is not None:
        lgpio.gpiochip_close(_button_handle)


def main():
    print("--- Servo + Motor Bench Test (no encoder) ---")

    init_button()
    servo_ok = servo.initialize()
    motor_ok = motor.initialize()

    if not (servo_ok and motor_ok):
        print("Initialization failed. Aborting.")
        if servo_ok:
            servo.cleanup()
        if motor_ok:
            motor.cleanup()
        cleanup_button()
        return

    try:
        while True:
            print("Sweeping servo...")
            for angle in SERVO_ANGLES:
                check_stop()
                print(f"  servo -> {angle:+d} deg")
                servo.set_angle(angle)
                sleep_unless_stopped(SERVO_DWELL_S)

            print(f"Motor forward @ {MOTOR_SPEED}% for {MOTOR_RUN_S}s")
            motor.forward(MOTOR_SPEED)
            sleep_unless_stopped(MOTOR_RUN_S)
            motor.brake()
            sleep_unless_stopped(MOTOR_PAUSE_S)

            print(f"Motor reverse @ {MOTOR_SPEED}% for {MOTOR_RUN_S}s")
            motor.reverse(MOTOR_SPEED)
            sleep_unless_stopped(MOTOR_RUN_S)
            motor.brake()
            sleep_unless_stopped(MOTOR_PAUSE_S)

    except StopRequested:
        print("\nStop button pressed. Shutting down.")
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        motor.cleanup()
        servo.cleanup()
        cleanup_button()


if __name__ == "__main__":
    main()
