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
# Savox SV-1232MG (digital, coreless, HV 6.0-8.4V). Published specs:
#   frame rate 50-360Hz, neutral 1500us, dead band 3-5us,
#   operating travel 1000-2000us = ~100deg, max travel 800-2200us (~165deg).
# Running at the 360Hz maximum so a new steering command reaches the servo within
# 2.78ms instead of up to 20ms at 50Hz (the period is still longer than the 2.2ms
# max pulse). Bench-tested 10-360Hz: the holding whine is identical at every rate,
# so frame rate is chosen purely for latency. Pulse widths below are absolute
# (seconds) and SERVO_PWM_PERIOD_S is derived from this, so center/endpoint
# calibration is unaffected by the frequency.
SERVO_PWM_FREQ = 360
SERVO_PWM_CHIP = 0
SERVO_PWM_CHANNEL = 2
SERVO_DIR_INVERT = True
# 1000-2000us spans 100deg of servo travel -> 10us/deg, so input angles map 1:1
# to physical degrees (+-50deg input == 1000/2000us).
CALIBRATED_MIN_PW_S = 0.001
CALIBRATED_MAX_PW_S = 0.002
CALIBRATED_ANGLE_MIN = 40.0
CALIBRATED_ANGLE_MAX = 140.0
INPUT_ANGLE_MIN_SERVO = -50.0
INPUT_ANGLE_MAX_SERVO = 50.0
SERVO_CENTER_OFFSET = -10
# Hard limits = the servo's rated max-travel pulse range.
SAFETY_MIN_PW_S = 0.0008
SAFETY_MAX_PW_S = 0.0022
SERVO_PWM_PERIOD_S = 1.0 / SERVO_PWM_FREQ
