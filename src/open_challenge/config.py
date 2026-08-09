# config.py
import cv2
import numpy as np

# --- Robot Parameters ---
MOTOR_SPEED = 90

# --- Frame & Video Processing ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

# --- Color Definitions (HSV) ---
# Ported from obstacle_challenge/tuning.py (v5). The black range is much tighter
# than the old open-challenge one (V<=70 vs V<=110) -- v5's wall areas, and
# therefore WALL_KP/WALL_KD below, are calibrated against THIS range. Loosening it
# inflates every area and effectively multiplies the gains.
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 95, 70])

# Orange + blue are only ever read inside the line ROI. Both ranges come from v5
# as a matched pair -- they decide the driving direction between them (whichever
# line is seen first), so they must have comparable sensitivity. If lap counting
# starts missing lines, lower LOWER_ORANGE[2] (currently 182) rather than swapping
# in a differently-tuned range for one colour only.
LOWER_ORANGE = np.array([6, 50, 182])
UPPER_ORANGE = np.array([15, 255, 255])
LOWER_BLUE = np.array([114, 50, 110])
UPPER_BLUE = np.array([123, 255, 255])

# --- Detection Parameters ---
WALL_MIN_AREA = 300
ORANGE_MIN_AREA = 20

# --- Wall-following gains (v5) -------------------------------------------
# PD on (left + inner_left) - (right + inner_right) black pixel area.
WALL_KP = 0.0006
WALL_KD = 0.0003

# Corner trigger: either a black band across the close ROI, or the line ROI
# filling up with wall.
CLOSE_BLACK_AREA_THRESHOLD = 3000
LINE_ROI_WALL_PCT_THRESHOLD = 50
CORNER_TURN_ANGLE = 35

# Edge-preserving filter instead of GaussianBlur (v5). Keeps the wall/mat boundary
# crisp where the blur smears it. d=5 only -- NEVER pass d=-1 (480 ms/frame).
USE_BILATERAL = True
BILATERAL_D = 5
BILATERAL_SIGMA_COLOR = 50
BILATERAL_SIGMA_SPACE = 50
HSV_BEFORE_BLUR = True

# --- Regions of Interest (ROI) ---
# ROIs for Wall Detection (v5 positions -- 10 px higher than the old open ones)
left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 130, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 130, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 155, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 155, 100, 100
close_x, close_y, close_w, close_h = 140, 110, 360, 10

# Jobs for wall detection
left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}

WALL_JOBS = [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]

# ROI for the floor line (orange/blue). v5 calls this the "line ROI"; the same
# patch doubles as the close-range wall check (line_roi_wall_pct).
line_roi_x, line_roi_y, line_roi_w, line_roi_h = 280, 190, 80, 40
# Back-compat aliases for anything still using the old names.
orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h = line_roi_x, line_roi_y, line_roi_w, line_roi_h

# --- ROI Masks (Pre-computed for performance) ---

# Mask for Wall ROIs
roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
for job in WALL_JOBS:
    x, y, w, h = job['roi']
    cv2.rectangle(roi_mask_walls, (x, y), (x + w, y + h), 255, -1)

# Mask for the line ROI
roi_mask_line = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_line, (line_roi_x, line_roi_y), (line_roi_x + line_roi_w, line_roi_y + line_roi_h), 255, -1)
roi_mask_orange = roi_mask_line

# Mask for "Close Black" Wall ROI (directly in front)
roi_mask_close_black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_close_black, (close_x, close_y), (close_x + close_w, close_y + close_h), 255, -1)

# --- Working slice -------------------------------------------------------
# Every ROI lives inside this vertical band, so colour conversion and filtering
# only ever run on these rows (v5 does the same).
GLOBAL_Y_OFFSET = min(left_roi_y, right_roi_y, inner_left_roi_y, inner_right_roi_y,
                      line_roi_y, close_y)
GLOBAL_Y_END = max(left_roi_y + left_roi_h, right_roi_y + right_roi_h,
                   inner_left_roi_y + inner_left_roi_h, inner_right_roi_y + inner_right_roi_h,
                   line_roi_y + line_roi_h, close_y + close_h)
SLICE_HEIGHT = GLOBAL_Y_END - GLOBAL_Y_OFFSET

# GPIO Pin Configuration
BUTTON_PIN = 23
LED_PIN = 12

del job, x, y, w, h
