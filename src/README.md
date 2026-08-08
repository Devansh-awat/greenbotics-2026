# Software Code Walkthrough (`src`)

This is a code walkthrough for the Python code that runs on the Raspberry Pi 5. It covers hardware interfaces, program flow, and core algorithm logic.

See the main [Main Readme - 5.3](../README.md#53-software-setup--running-the-robot) for install steps and detailed design aspects in main [ - 3](Main Readme../README.md#3-software-architecture--obstacle-strategy)

## 1. Hardware Interfaces

Each hardware component has its own driver module. All of them talk to RP1 (GPIO, PWM, I2C, SPI) directly, so the robot needs no separate microcontroller.

**Servo steering** (`motors/servo.py`) uses hardware PWM via RP1. Every angle gets clamped to safe limits before it reaches the servo:

```python
servo_pwm = HardwarePWM(pwm_channel=config.SERVO_PWM_CHANNEL,
                         hz=config.SERVO_PWM_FREQ, chip=config.SERVO_PWM_CHIP)

def set_angle(input_angle: float):
    adjusted_angle = input_angle + config.SERVO_CENTER_OFFSET
    clamped_input = max(config.INPUT_ANGLE_MIN_SERVO,
                         min(config.INPUT_ANGLE_MAX_SERVO, adjusted_angle))
    # ...maps clamped_input to the calibrated PWM output range
```

**Drive motor** (`motors/motor.py`) uses hardware PWM for speed and GPIO pins for direction. The wheel encoder reads through RP1's PIO block:

```python
motor_pwm = HardwarePWM(pwm_channel=config.MOTOR_PWM_CHANNEL,
                         hz=config.MOTOR_PWM_FREQ, chip=config.MOTOR_PWM_CHIP)
encoder = IncrementalEncoder(board.D20)   # PIO-backed, counts pulses in hardware

def _set_speed(speed):
    motor_pwm.change_duty_cycle(max(0, min(100, speed)))
```

**Distance sensors** (`sensors/distance.py`) bring up 4 VL53L4CD sensors one at a time on the same I2C bus. Each sensor's XSHUT pin holds it in reset until its turn, so every sensor gets a unique address:

```python
for channel in SENSOR_CHANNELS:
    _xshut_devices[channel] = DigitalOutputDevice(channel, initial_value=False)  # hold all low
# then _bring_up() releases one XSHUT at a time and re-addresses that sensor
```

**Encoder PIO program** (`sensors/encoder.py`) loads a small assembly program onto RP1 that counts quadrature pulses in hardware. This keeps every pulse count accurate no matter how busy the CPU gets — see [Main Readme 3.7.1](../README.md#371-why-pio-for-the-encoder) for the full explanation.

## 2. Program Flow

Both challenges run a sense → decide → act loop. The main README has the flowcharts for this:
- Open Challenge flow: [Main Readme 3.3, Open Challenge section](../README.md#33-open-challenge--srcopen_challengemainpy)
- Obstacle Challenge state machine: [Main Readme 3.4 , Obstacle Challenge section](../README.md#34-obstacle-challenge--state-machine--algorithms)
- Threading model: [Main Readme 3.2.1 , System Architecture](../README.md#321-threading-model)

Here are some important code snippets explaining the core algorithms we have used: 

**Priority state machine** runs fresh every frame inside the main loop, and the highest priority match wins. The main README has the full state table in [Main Readme 3.4.1](../README.md#341-states--priority-order):

```python
if close_black_area > 3000:
    # P1: AVOID HEADON — hard steer away from wall dead ahead
elif detected_blocks:
    # P2: PASS TRAFFIC SIGN — target-line geometry steers around pillar
elif wall_inner_left < 100 or wall_inner_right < 100:
    # P3: CORNER TURN — amplify remaining wall area to force the turn
else:
    # P4: WALL FOLLOW — PD controller keeps robot centered
```

**Target-line steering** passes a pillar on the correct side using an angle-based law, tuned separately for red and green:

```python
current_angle = math.atan2(block_x - origin_x, origin_y - block_y)
steering_angle = (current_angle - IDEAL_ANGLE) * Kp
# IDEAL_ANGLE = +42.5° for red, -40.5° for green ; Kp = 1.5
```

**Gyro steering** (`steer_with_gyro`) holds a straight line or executes precise turns during parking:

```python
def steer_with_gyro(current_heading, target_heading, Kp=0.85):
    heading_error = get_angular_difference(target_heading, current_heading)
    return heading_error * Kp
```

**Turn counting** uses a debounced rising-edge check on the orange line detector, so one line crossing at high speed only counts once:

```python
orange_detection_history.append(orange_detected_this_frame)   # last 4 frames
if not orange_detection_history[-4] and all(list(orange_detection_history)[1:]):
    turn_counter += 1
    cooldown_frames = ORANGE_COOLDOWN_FRAMES   # 50-frame cooldown
```

[Main Readme 3.4.8](../README.md#348-parking-algorithm) covers the parking sequence, edge cases, and parameter tuning in full detail.


