% config.m — simulation-side constants, cross-checked against the CURRENT
% robot source (src/obstacle_challenge/{tuning,control,rules,main}.py).
%
% SYNC NOTE (2026-11): this file used to claim a line-by-line mirror of a file
% called main_v3.py. That file is retired (moved to src/obstacle_challenge/
% legacy/); the robot has since shipped main_v4.py and now main.py, with the
% block-steering law replaced by a rule engine (rules.py: rule_red/rule_green,
% pick_target_block) and detection generalised into src/vision/pipeline.py
% driven by tuning.py's OBJECTS table. That rule engine depends on live
% camera-handoff geometry and a white-balance colour split (tuning.py's
% COLOUR_SPLIT) that have no meaningful equivalent in this top-down physics
% model, so it is NOT ported line-by-line here — see matlab/README.md
% "Perception" and "Navigation" sections for exactly what is and isn't a
% literal port after this sync.

DRIVE_SPEED               = 85.0;     % real robot: 85-92 PWM
FRAME_WIDTH               = 640;
FRAME_HEIGHT              = 360;      % real camera stream is also 640x360
CROP_TOP_FRAC             = 100/360;  % legacy main_v3.py ROI: Y=100-290 out of 360
CROP_BOTTOM_FRAC          = 70/360;

% Block-detection area thresholds. Deliberately MORE PERMISSIVE than the
% robot's current BLOCK_MIN_AREA=250 (tuning.py): render_frame.m paints
% pillars as flat, idealized colour (see below), so the simulated camera
% needs its own thresholds, not the venue-lighting-calibrated real ones.
MIN_CONTOUR_AREA          = 500;
MIN_BLOCK_AREA_FOR_ACTION = 1500;     % trigger avoidance steering (early)

% Legacy centroid-based proportional steering law (main_v3.py-era). Current
% robot code has no KP_BLOCK/target_x at all — see rules.py rule_red/
% rule_green instead. Kept here ONLY for 'field' mode, which is an explicit
% simulation baseline, not a claimed port (see control_step.m header).
% servo_angle = KP_BLOCK * (centroid_x - target_x), clamped ±45°
KP_BLOCK                  = 0.09;
RED_TARGET_X              = 320 - 300;   % =  20  push red block to far left
GREEN_TARGET_X            = 320 + 300;   % = 620  push green block to far right

CLEARANCE_FRAMES          = 45;

% Gyro heading-lock PD gains — these DO match the real robot:
% src/obstacle_challenge/tuning.py: GYRO_KP = 0.85, GYRO_KD = 0.1, used by
% control.py's steer_with_gyro()/_heading_hold_step() (same error-wrap +
% kp*error + kd*derivative formula as gyro_steer.m below).
GYRO_KP                   = 0.85;
GYRO_KD                   = 0.1;
% Discrete corner-turn snap gain. The real robot executes a corner as a
% separate blocking turn routine (maneuvers.py), not this PD loop, so there
% is no real-robot constant to mirror here — this is a simulation-only gain,
% named apart from GYRO_KP/KD so it is never mistaken for one.
CORNER_SNAP_KP            = 3.0;
HEADING_LOCK_TOLERANCE    = 5.0;       % currently unused (reserved)

% Simulated-camera HSV thresholds for the idealized pillar colours drawn by
% render_frame.m (red [215,30,30], green [25,185,40] in RGB -> saturated,
% bright in HSV). For reference, the REAL robot's venue-calibrated ranges
% (src/obstacle_challenge/tuning.py HSV_RANGES, as of this sync) are:
%   red:   H 0-5/176-180, S 80-230, V 35-180
%   green: H 54-82,       S 49-255, V 35-255
% Those V-upper bounds (180/255 at the low end) are tuned for real room
% lighting and would clip the simulator's much brighter synthetic pillars,
% so this file keeps its own wider, idealized-colour thresholds below rather
% than copying the real numbers verbatim.
LOWER_RED_1 = [0,   150, 100];   UPPER_RED_1 = [10,  255, 255];
LOWER_RED_2 = [170, 150, 100];   UPPER_RED_2 = [180, 255, 255];
LOWER_GREEN = [35,   60,  60];   UPPER_GREEN = [90,  255, 255];

% Wall HSV threshold — UNLIKE red/green above, this one IS copied verbatim
% from the real robot (src/obstacle_challenge/tuning.py HSV_RANGES), because
% it actually works against the synthetic render: render_frame.m draws walls
% as flat grey (body V=12, edge V=60), which is zero-saturation at any hue,
% so it clears this threshold (S<=110, V<=85) the same way a real black wall
% under room lighting does. No idealized-vs-calibrated mismatch here.
LOWER_BLACK = [0, 0, 0];   UPPER_BLACK = [180, 110, 85];