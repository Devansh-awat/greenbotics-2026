import numpy as np


TILT_THRESHOLD_DEGREES = 15.0


DRIVE_SPEED = 100.0


FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MAX_FPS = 30
CROP_TOP_FRAC = 0
CROP_BOTTOM_FRAC = 0
MIN_CONTOUR_AREA = 2500
MAX_BLOCK_AREA_FRACTION = 0.25
MIN_BLOCK_ROI_OVERLAP = 0.50


MIN_BLOCK_AREA_FOR_ACTION = 7500
MANEUVER_ANGLE_DEG = 45.0
MIN_FRAMES_IN_TURN = 30
TOTAL_TURNS = 12
MANEUVER_DRIVE_FRAMES_SIDEWAYS = 10
MANEUVER_DRIVE_FRAMES_STRAIGHT = 0
SPECIAL_TURN_SIDEWAYS_FRAMES = 0


CCW_TURN_ANGLE = 90.0
CCW_SCAN_ANGLE = 60.0
CCW_NORMAL_DRIVE_FRAMES = 25
CCW_REVERSE_FRAMES = 10
CCW_SHORT_DRIVE_FRAMES = 0


CW_DRIVE_FORWARD_RED_FRAMES = 25
CW_REVERSE_GREEN_FRAMES = 15
CW_DRIVE_FORWARD_NONE_FRAMES = 0

GYRO_ENABLED = True
GYRO_KP = 3.0
HEADING_LOCK_TOLERANCE = 5.0
INVERT_GYRO = True


TOF_ENABLED = True
TOF_CHANNELS_TO_USE = range(1,4)
TOF_FORWARD_SENSOR_CHANNEL = 1
TOF_CORNERING_THRESHOLD_MM = 240
TOF_OBSTACLE_THRESHOLD_MM = 100
WALL_FOLLOW_KP = 0.2


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
MOTOR_PWM_PIN = 19
MOTOR_PWM_FREQ = 1000
MOTOR_PWM_CHIP = 0
MOTOR_PWM_CHANNEL = 3
SERVO_GPIO = 18
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


BOX_COLOR_RED = (0, 0, 255)
BOX_COLOR_GREEN = (0, 255, 0)
BOX_COLOR_ROI = (255, 0, 255)

LOWER_RED_1 = np.array([0, 100, 55])
UPPER_RED_1 = np.array([5, 255, 255])
LOWER_RED_2 = np.array([174, 100, 55])
UPPER_RED_2 = np.array([180, 255, 255])

LOWER_GREEN = np.array([40, 60, 40])
UPPER_GREEN = np.array([80, 255, 180])

