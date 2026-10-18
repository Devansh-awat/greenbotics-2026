TILT_THRESHOLD_DEGREES = 15.0

GYRO_ENABLED = True
INVERT_GYRO = True

BUTTON_PIN = 23
LED_PIN = 12

AIN1_PIN = 26
AIN2_PIN = 13
STBY_PIN = 6
# The motor's two leads are reversed relative to the TB6612's A01/A02 outputs,
# and they're soldered to the PCB so they can't be swapped back. Inverting here
# makes forward() drive the robot forwards.
# Changed to False to reverse the direction as requested (motor went backward when told forward).
MOTOR_DIR_INVERT = False
MOTOR_PWM_FREQ = 1000
MOTOR_PWM_CHIP = 0
MOTOR_PWM_CHANNEL = 3
# Raised from the standard 50Hz to cut command-update quantization (20ms->12.5ms).
# Pulse widths below are absolute (seconds) and SERVO_PWM_PERIOD_S is derived from
# this, so center/endpoint calibration is unaffected -- verified identical 1.489ms
# pulse across frequencies.
#
# The ES08A II is an ANALOG servo and EMAX publishes no max refresh rate. 80Hz is
# the highest rate with reported headroom: analog servos drive the motor with a
# pulse proportional to position error, and once that drive pulse approaches the
# update interval the internal loop goes unstable. Reported behaviour on analog
# servos is 80Hz good / 90Hz marginal / 100Hz oscillating -- and the instability
# shows up on LARGE steps (our 55deg close-block swing), not at rest, so it will
# not necessarily reveal itself on a stationary bench test.
# If steering hunts or buzzes during big swings, drop back to 50.
SERVO_PWM_FREQ = 80
SERVO_PWM_CHIP = 0
SERVO_PWM_CHANNEL = 2
CALIBRATED_MIN_PW_S = 0.001
CALIBRATED_MAX_PW_S = 0.002
CALIBRATED_ANGLE_MIN = 42.5
CALIBRATED_ANGLE_MAX = 137.5
INPUT_ANGLE_MIN_SERVO = -47.5
INPUT_ANGLE_MAX_SERVO = 47.5
SERVO_CENTER_OFFSET = -1.0
SAFETY_MIN_PW_S = 0.0007
SAFETY_MAX_PW_S = 0.0023
SERVO_PWM_PERIOD_S = 1.0 / SERVO_PWM_FREQ
