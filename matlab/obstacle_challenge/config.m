% config.m  — mirrors config.py / camera.py (Greenbotics GitHub)

DRIVE_SPEED               = 85.0;     % real robot: 85-92 PWM
FRAME_WIDTH               = 640;
FRAME_HEIGHT              = 360;      % camera.py uses 640×360 low-res stream
CROP_TOP_FRAC             = 100/360;  % main_v3.py ROI: Y=100-290 out of 360
CROP_BOTTOM_FRAC          = 70/360;
MIN_CONTOUR_AREA          = 500;      % main_v3.py: 500 px²
MIN_BLOCK_AREA_FOR_ACTION = 1500;     % trigger avoidance steering (early)

% Centroid-based proportional steering (main_v3.py — NOT gyro heading)
% servo_angle = KP_BLOCK * (centroid_x - target_x), clamped ±45°
KP_BLOCK                  = 0.09;
RED_TARGET_X              = 320 - 300;   % =  20  push red block to far left
GREEN_TARGET_X            = 320 + 300;   % = 620  push green block to far right

CLEARANCE_FRAMES          = 45;
GYRO_KP                   = 3.0;
HEADING_LOCK_TOLERANCE    = 5.0;

LOWER_RED_1 = [0,   150, 100];   UPPER_RED_1 = [10,  255, 255];
LOWER_RED_2 = [170, 150, 100];   UPPER_RED_2 = [180, 255, 255];
LOWER_GREEN = [35,   60,  60];   UPPER_GREEN = [90,  255, 255];