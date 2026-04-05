import lgpio
from rpi_hardware_pwm import HardwarePWM
import time
from src.obstacle_challenge import config
from src.sensors.encoder import IncrementalEncoder
import board


gpio_handle = None
motor_pwm = None
encoder = None
_MAX_PWM_RETRIES = 5
_PWM_RETRY_DELAY = 0.1

def initialize():
    """Initializes the DC motor driver and PWM."""
    global gpio_handle, motor_pwm
    try:
        gpio_handle = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(gpio_handle, config.AIN1_PIN)
        lgpio.gpio_claim_output(gpio_handle, config.AIN2_PIN)
        lgpio.gpio_claim_output(gpio_handle, config.STBY_PIN)

        standby()
    except Exception as err:
        print(f"FATAL: Motor failed to initialize GPIO: {err}")
        return False

    last_error = None
    for attempt in range(1, _MAX_PWM_RETRIES + 1):
        try:
            motor_pwm = HardwarePWM(
                pwm_channel=config.MOTOR_PWM_CHANNEL,
                hz=config.MOTOR_PWM_FREQ,
                chip=config.MOTOR_PWM_CHIP,
            )
            motor_pwm.start(0)
            
            global encoder
            try:
                encoder = IncrementalEncoder(board.D20)
                print("INFO: Encoder Initialized.")
            except Exception as e:
                print(f"WARNING: Encoder failed to initialize: {e}")

            print("INFO: Motor Initialized.")
            return True
        except PermissionError as err:
            last_error = err
            print(
                f"WARNING: Motor PWM permission denied (attempt {attempt}/{_MAX_PWM_RETRIES}): {err}"
            )
            time.sleep(_PWM_RETRY_DELAY * attempt)
        except Exception as err:
            last_error = err
            print(f"FATAL: Motor failed to initialize: {err}")
            time.sleep(_PWM_RETRY_DELAY * attempt)

    print(f"FATAL: Motor failed to initialize after {_MAX_PWM_RETRIES} attempts: {last_error}")
    return False


def _set_speed(speed):
    """Internal function to set motor PWM duty cycle."""
    if motor_pwm:
        motor_pwm.change_duty_cycle(max(0, min(100, speed)))


def forward(speed):
    """Drives the motor forward at a given speed."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 0)
        _set_speed(speed)


def reverse(speed):
    """Drives the motor in reverse at a given speed."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 0)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 1)
        _set_speed(speed)


def standby():
    """Puts the motor driver in standby mode (low power, disengaged)."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 0)


def brake():
    """Brakes the motor by shorting its terminals."""
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, config.STBY_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN1_PIN, 1)
        lgpio.gpio_write(gpio_handle, config.AIN2_PIN, 1)
        _set_speed(0)


def move(distance_cm, max_speed=70, min_speed=40, accel_dist_cm=2, kp=3.0, ki=0.0, kd=0.1):
    """
    Moves the motor a specific distance in cm using the encoder.
    Uses PID control for accurate deceleration, while keeping acceleration ramp.
    """
    if not encoder:
        print("ERROR: Encoder not initialized. Cannot use move().")
        return

    target_mm = abs(distance_cm * 10)
    accel_mm = accel_dist_cm * 10
    
    start_dist = encoder.distance
    direction = 1 if distance_cm > 0 else -1
    motor_func = forward if direction == 1 else reverse
    
    brake()
    
    integral = 0.0
    previous_error = target_mm
    last_time = time.time()
    
    while True:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0:
            dt = 0.01
            
        current_travel_mm = abs(encoder.distance - start_dist)
        error = target_mm - current_travel_mm
        
        # Stop condition: reached target or within 0.5mm tolerance
        if error <= 0.5:
            break
            
        integral += error * dt
        derivative = (error - previous_error) / dt
        
        # PID evaluates the required speed based on distance error
        pid_output = (kp * error) + (ki * integral) + (kd * derivative)
        
        # Cap speed by max_speed
        speed = min(max_speed, abs(pid_output))
        
        # Acceleration phase (overrides PID output to slowly ramp up if needed)
        if current_travel_mm < accel_mm:
            speed_ratio = current_travel_mm / accel_mm if accel_mm > 0 else 1
            accel_speed = min_speed + (max_speed - min_speed) * speed_ratio
            speed = min(speed, accel_speed)
            
        # Ensure minimum speed so motor does not stall before reaching target
        speed = max(min_speed, speed)
        
        motor_func(speed)
        
        previous_error = error
        last_time = current_time
        
        time.sleep(0.01)

    brake()
    print(f"move({distance_cm} cm) complete.")


def cleanup():
    """Stops the motor and releases GPIO resources."""
    print("--- Cleaning up Motor ---")
    if motor_pwm:
        forward(0)
        standby()
        motor_pwm.stop()
    if gpio_handle:
        lgpio.gpiochip_close(gpio_handle)
    if encoder:
        encoder.deinit()


if __name__ == "__main__":
    print("--- Testing Motor Module ---")
    if not initialize():
        print("Motor test failed during initialization.")
    else:
        try:
            print("Moving forward 10cm using move()...")
            move(30)
            time.sleep(1)

            print("Moving backward 10cm using move()...")
            move(-30)
            time.sleep(1)

            print("Motor test complete.")

        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()