# config.py
import cv2
import numpy as np

# --- Robot Parameters ---
TARGET_RPM = 450.0  # Closed-loop target wheel RPM
MOTOR_SPEED = TARGET_RPM  # Backwards-compatibility alias

# --- Frame & Video Processing ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

# --- Performance Switches & Video ---
USE_VISION_POOL = True
VISION_POOL_TIMEOUT = 0.050
VISION_WORKER_THREADS = 2
VIDEO_QUEUE_SLOTS = 8
VIDEO_FOURCC = 'avc1'
USE_LAB = False

# --- Turn Counting & Stop Parameters ---
ORANGE_COOLDOWN_FRAMES = 50
ORANGE_DETECTION_HISTORY_LENGTH = 3
FINAL_STOP_DISTANCE_CM = 36.8  # Distance in cm to drive after turn 12 (450 RPM * 0.25s = 36.8 cm)
FINAL_STOP_DELAY = 2  # Fallback timeout in seconds

# --- Color Definitions (Imported from Obstacle Challenge tuning) ---
from src.obstacle_challenge.tuning import (
    COLOR_RANGES,
    HSV_RANGES,
    LAB_RANGES,
    LUV_RANGES,
    LOWER_BLACK,
    UPPER_BLACK,
    LOWER_ORANGE,
    UPPER_ORANGE,
    LOWER_BLUE,
    UPPER_BLUE,
    LOWER_RED_1,
    UPPER_RED_1,
    LOWER_RED_2,
    UPPER_RED_2,
    LOWER_GREEN,
    UPPER_GREEN,
    LOWER_MAGENTA,
    UPPER_MAGENTA,
)

# --- Detection Parameters ---
WALL_MIN_AREA = 300
ORANGE_MIN_AREA = 20
BLOCK_MIN_AREA = 250
MAGENTA_MIN_AREA = 300
CLOSE_BLOCK_MIN_AREA = 15

# --- Wall-following gains (v5) -------------------------------------------
# PD on (left + inner_left) - (right + inner_right) black pixel area.
WALL_KP = 0.0012
WALL_KD = 0.0005

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
left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 130, 135, 160
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 130, 135, 160
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 170, 100, 110
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 170, 100, 110
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

# Block ROIs (unused on open track, empty/dummy masks)
full_frame_roi = (0, 0, 0, 0)
close_block_roi = (0, 0, 0, 0)
roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_magenta = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")

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
