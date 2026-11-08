# Software Code Walkthrough (`src`)

This is a code walkthrough for the Python code that runs on the Raspberry Pi 5. It covers hardware interfaces, program flow, and core algorithm logic.

See the main [Main README - 6.3](../README.md#63-software-setup-and-running-the-robot) for install steps and detailed design aspects in main [README - 4](../README.md#4-software-architecture-and-obstacle-strategy)

## 1. Hardware Interfaces

Each hardware component has its own driver module. All of them talk to RP1 (GPIO, PWM, I2C, PIO) directly, so the robot needs no separate microcontroller.

**Servo steering** (`motors/servo.py`) uses hardware PWM via RP1. Every angle gets clamped to safe limits before it reaches the servo:

```python
servo_pwm = HardwarePWM(pwm_channel=config.SERVO_PWM_CHANNEL,
                         hz=config.SERVO_PWM_FREQ, chip=config.SERVO_PWM_CHIP)

def set_angle(input_angle: float):
    clamped_input = max(-45.0, min(45.0, input_angle)) + config.SERVO_CENTER_OFFSET
    clamped_input = max(_MIN_SERVO_ANGLE, clamped_input)
    if config.SERVO_DIR_INVERT:
        clamped_input = -clamped_input
```

`set_angle_unlimited()` does the same with a +-50 deg clamp, for the sharper turns in parking.

**Drive motor** (`motors/motor.py`) uses hardware PWM for speed and GPIO pins for direction. The wheel encoder reads through RP1's PIO block:

```python
motor_pwm = HardwarePWM(pwm_channel=config.MOTOR_PWM_CHANNEL,
                         hz=config.MOTOR_PWM_FREQ, chip=config.MOTOR_PWM_CHIP)
encoder = IncrementalEncoder(board.D20)   # runs on PIO

def _set_speed(speed):
    motor_pwm.change_duty_cycle(max(0, min(100, speed)))
```

**Distance sensors** (`sensors/distance.py`) bring up the two wired VL53L4CD sensors (front and back) one at a time on I2C bus 3. Every XSHUT pin, including the unused left and right ones, starts low (in reset), so each sensor is alone at the default address when it is moved to its own:

```python
for channel in SENSOR_CHANNELS + UNUSED_CHANNELS:
    _xshut_devices[channel] = DigitalOutputDevice(channel, initial_value=False)  # hold all low
# then _bring_up() releases one XSHUT at a time and moves that sensor off 0x29
```

The left and right sensors stay in reset (low power).

**Encoder PIO program** (`sensors/encoder.py`) loads a small assembly program onto RP1 that counts quadrature pulses in hardware. This keeps every pulse count accurate no matter how busy the CPU gets — see [Main README 4.8.1](../README.md#481-why-pio-for-the-encoder) for the full explanation.

## 2. Program Flow

Both challenges run a sense → decide → act loop. The main README has the flowcharts for this:
- Open Challenge flow: [Main README 4.3, Open Challenge section](../README.md#43-open-challenge-flow-chart-and-algorithm)
- Obstacle Challenge state machine: [Main README 4.4, Obstacle Challenge section](../README.md#44-obstacle-challenge-state-machine-and-algorithms)
- Threading model: [Main README 4.2.1, System Architecture](../README.md#421-threading-model)

Here are some important code snippets explaining the core algorithms we have used: 

**Priority rule list** (`obstacle_challenge/rules.py`) runs fresh every frame. Each rule returns `None` ("not my situation") or a `Command`, and the first rule that answers wins. The filters then adjust the winning command. The main README has the state table in [Main README 4.4.1](../README.md#441-states-and-priority-order):

```python
RULES = [
    rule_close_block,   # reverse if pillar too close
    rule_red,           # pass red on its right
    rule_green,         # pass green on its left
    rule_wall_follow,   # wall follow
]

def decide(world, mem):
    for rule in RULES:
        cmd = rule(world, mem)
        if cmd is not None:
            break
    for f in FILTERS:
        f(cmd, world, mem)
    return cmd
```

**Pillar steering** (`rule_red` / `rule_green`) aims at the gap beside the pillar: between the pillar and the black wall, or the magenta parking wall. If no gap is found, it falls back to an angle-based target line:

```python
if aim_x is not None:   # midpoint
    error = aim_x - FRAME_MIDPOINT_X
    angle = error * TARGET_LINE_KP + (error - mem.prev_target_error) * TARGET_LINE_KD
else:                   # target line
    ideal   = math.degrees(math.atan2(other_x - ox, oy - other_y))
    current = math.degrees(math.atan2(bx - ox, oy - by))
    angle = (current - ideal) * BLOCK_ANGLE_GAIN
```

**Gyro steering** (`steer_with_gyro` in `obstacle_challenge/control.py`) holds a straight line or executes precise turns during the initial maneuver and parking:

```python
def steer_with_gyro(current_heading, target_heading, kp=GYRO_KP, kd=GYRO_KD,
                    prev_error=0.0, min_servo_angle=-45, max_servo_angle=45):
    error = (target_heading - current_heading + 180.0) % 360.0 - 180.0
    steer_angle = kp * error + kd * (error - prev_error)
    return max(min_servo_angle, min(max_servo_angle, steer_angle))
```

**Turn counting** uses a debounced rising-edge check on the orange line detector, so one line crossing at high speed only counts once:

```python
mem.orange_history.append(bool(world.orange))        # last 3 frames
if mem.turn_cooldown > 0:
    mem.turn_cooldown -= 1
elif not mem.orange_history[0] and sum(mem.orange_history) == ORANGE_DETECTION_HISTORY_LENGTH - 1:
    mem.turns += 1
    mem.turn_cooldown = ORANGE_COOLDOWN_FRAMES
```

Parking and the initial maneuver are in `obstacle_challenge/maneuvers.py`. [Main README 4.4.8](../README.md#448-parking-algorithm) covers the parking sequence, edge cases, and parameter tuning.

---

## Files in this folder
This is a high-level map of how files are organized within src folder.

```
src/
├── experiments/          # Standalone data-collection scripts for tuning/characterizing robot behavior
├── logs/                 # Logging setup/configuration for run logs
├── motors/               # Drive motor and steering servo control (PWM, RPM closed-loop)
├── obstacle_challenge/   # Obstacle Challenge entry point, tuning, rules, maneuvers and video recording
├── open_challenge/       # Open Challenge entry point, config, rules, speed schedule and frame log
├── sensors/              # Hardware-abstraction drivers for camera, IMU, distance sensors, and encoder
├── tests/                # Regression tests for code refactoring 
├── threads/              # Background hardware-sensor thread management
├── tools/                # Calibration and diagnostic utilities (HSV tuning, capture pipelines, sensor tests)
├── vision/               # Shared frame-processing pipeline and worker pool
├── requirements.txt      # Python dependencies (pip)
└── __init__.py           # Marks src/ as a Python package (empty)
```
