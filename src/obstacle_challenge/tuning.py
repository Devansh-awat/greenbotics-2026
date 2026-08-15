"""Tuning constants for the obstacle challenge (v5).

These deliberately override the values in config.py -- when tuning a run, edit
here, not config.py. Imported with `from ...tuning import *` by the vision and
control modules, so every name defined here is part of the public surface.
"""

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------

# MOTOR_SPEED = 88
# ORANGE_COOLDOWN_FRAMES = 45

MOTOR_SPEED = 65
MAX_WHEEL_RPM = 1800.0 * 13.0 / 38.0  # Pololu 4861 @ 12V with 13/38 external gear (~615.8)
MIN_RPM = 0.5 * MAX_WHEEL_RPM
MAX_RPM = 0.65 * MAX_WHEEL_RPM
MAX_ACCEL_PER_FRAME = 10.0
MAX_DECEL_PER_FRAME = 200.0
INITIAL_RPM = 50.0
USE_VARIABLE_SPEED = True  # Set to True to re-enable variable speed based on steering & block height
BLOCK_TARGET_GRACE_FRAMES = 12

ORANGE_COOLDOWN_FRAMES = 50

#MOTOR_SPEED = 92
#ORANGE_COOLDOWN_FRAMES = 45

ORANGE_DETECTION_HISTORY_LENGTH = 3

WALL_THRESHOLD = 200
WALL_KP = 0.0006
WALL_KD = 0.0003

GYRO_KP = 0.85
GYRO_KD = 0.1


FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

USE_LAB = False

# --- Performance switches ---------------------------------------------------
# USE_VISION_POOL: run the arena mask and the colour thresholding in two worker
#   processes and join them. ~1 ms/frame. Set False to run everything inline --
#   the results are identical, so this is a safe thing to turn off when debugging.
# USE_BILATERAL: bilateral filtering preserves pillar/wall edges while killing mat
#   texture noise, which GaussianBlur smears. Measured on the working slice:
#   d=5 -> 1.6 ms, d=9 -> 5.5 ms. d=5 fits the budget comfortably; d=9 only really
#   fits with the pool on. NEVER pass d=-1: OpenCV then derives d from sigmaSpace
#   and it costs 480 ms/frame.
USE_VISION_POOL = True
# Bilateral costs about +1.5 ms of capture->steer latency: measured end-to-end against
# the live camera, 6.9 ms off vs 8.2 ms on. It runs in the colour worker, which is the
# shorter half of the fork-join, so some of it hides under the arena mask (3.3 ms) --
# but not all of it. In isolation the colour half is 1.6 ms -> 3.0 ms at d=5, which
# looks like it should hide completely; it doesn't, because under real contention
# (arena worker + 8 x264 threads + control loop) the worker doesn't get the two full
# cores that microbenchmark assumed. Do not trust the isolated number, measure e2e.
# d=9 is far worse (7.7 ms for the colour half even at 4 threads) -- don't raise d
# without re-measuring.
#
# WARNING: this is not a free-lunch accuracy win, it CHANGES DETECTIONS. Measured over
# 250 real frames, d=5 alters the detection set on 200 of them -- almost entirely small
# centroid/area shifts (net object count over the whole set: +1 block, +2 walls,
# +3 close_black), not objects appearing or disappearing. Centroids feed the steering
# angle directly, so the HSV ranges and the block-targeting constants may want a
# re-tune. Set False to get the old GaussianBlur behaviour back exactly.
USE_BILATERAL = True
BILATERAL_D = 5
BILATERAL_SIGMA_COLOR = 50
BILATERAL_SIGMA_SPACE = 50
HSV_BEFORE_BLUR = True

# Worker sync timeout. If a vision worker ever misses this the pool is disabled for
# the rest of the run and we fall back to inline processing -- a slow frame is
# survivable, a hung control loop on a moving robot is not.
VISION_POOL_TIMEOUT = 0.050

# OpenCV threads inside each vision worker. 1 is the obvious choice for a worker that
# owns a core, but bilateralFilter parallelises well: measured in isolation the colour
# half is 4.90 ms at 1 thread vs 2.96 ms at 2. End-to-end the gain is much smaller
# (8.5 -> 8.2 ms) because the extra thread competes with everything else, but it is
# not negative and there is ~60% idle CPU to pay for it. Going above 2 measured worse.
VISION_WORKER_THREADS = 2

VIDEO_QUEUE_SLOTS = 8      # ring-buffer depth for the encoder process
VIDEO_FOURCC = 'avc1'      # H.264. Opens in QuickTime; mp4v does not.

CAMERA_FPS = 56.0          # the sensor's hard ceiling, see the module docstring

# Record only every Nth frame. This is the single biggest performance lever in the
# whole program, and it is not obvious why.
#
# Encoding cost depends on CONTENT, not just resolution. Benchmarks against a parked
# robot looked fine because a static scene inter-frame-compresses to almost nothing.
# On a moving robot every frame is genuinely different, x264 does real work in all 8
# of its threads, and it starves the vision workers. Measured by replaying 2312 real
# moving frames at 56 fps through the full stack (IMU + ToF + encoder threads live):
#
#     every frame  49.9 fps | vision 15.43 ms | capture->steer 22.65 ms | 91 skipped
#     every 2nd    55.7 fps | vision  8.06 ms | capture->steer  8.38 ms |  6 skipped
#     every 3rd    56.0 fps | vision  6.74 ms | capture->steer  7.00 ms |  1 skipped
#     no encoder   55.9 fps | vision  7.58 ms | capture->steer  7.79 ms |  3 skipped
#
# That first row is exactly what the 2026-08-02_00-21-29 run recorded. At N=2 the
# recording is 28 fps, still smooth to review, and the control loop gets its frames
# back. N=3 buys another 1.4 ms if you want it and can live with 18.7 fps video.
#
# Capping x264's thread count or forcing preset=ultrafast via
# OPENCV_FFMPEG_WRITER_OPTIONS was measured to do nothing -- OpenCV does not appear
# to honour it in this build. Decimation is what works.
VIDEO_EVERY_N = 2

# Metadata fps set to 10.0 so that video recordings play back in slow motion.
VIDEO_FPS = 10.0

PERF_REPORT_PERIOD = 2.0   # seconds between INFO-level perf summaries

HSV_RANGES = {
    'LOWER_RED_1': np.array([0, 70, 43]), 'UPPER_RED_1': np.array([4, 230, 166]),
    'LOWER_RED_2': np.array([175, 70, 43]), 'UPPER_RED_2': np.array([180, 230, 140]),
    'LOWER_GREEN': np.array([42, 85, 38]), 'UPPER_GREEN': np.array([88, 190, 135]),
    'LOWER_BLACK': np.array([0, 0, 0]), 'UPPER_BLACK': np.array([180, 95, 70]),
    'LOWER_ORANGE': np.array([6, 50, 182]), 'UPPER_ORANGE': np.array([15, 255, 255]),
    'LOWER_BLUE': np.array([114, 50, 110]), 'UPPER_BLUE': np.array([123, 255, 255]),
    'LOWER_MAGENTA': np.array([158, 73, 64]), 'UPPER_MAGENTA': np.array([172, 255, 223])
}

LAB_RANGES = {
    'LOWER_RED_1': np.array([30, 159, 137]), 'UPPER_RED_1': np.array([158, 175, 169]),
    'LOWER_RED_2': np.array([20, 150, 150]), 'UPPER_RED_2': np.array([200, 255, 255]), # Duplicate for now
    'LOWER_GREEN': np.array([79, 80, 115]), 'UPPER_GREEN': np.array([134, 129, 146]),
    'LOWER_BLACK': np.array([0, 115, 115]), 'UPPER_BLACK': np.array([130, 134, 134]),
    'LOWER_ORANGE': np.array([97, 136, 138]), 'UPPER_ORANGE': np.array([177, 169, 172]),
    'LOWER_MAGENTA': np.array([72, 147, 48]), 'UPPER_MAGENTA': np.array([159, 174, 130]),
    'LOWER_BLUE': np.array([28, 136, 44]), 'UPPER_BLUE': np.array([100, 163, 104])
}

if USE_LAB:
    COLOR_RANGES = LAB_RANGES
else:
    COLOR_RANGES = HSV_RANGES

LOWER_RED_1 = COLOR_RANGES['LOWER_RED_1']
UPPER_RED_1 = COLOR_RANGES['UPPER_RED_1']
LOWER_RED_2 = COLOR_RANGES['LOWER_RED_2']
UPPER_RED_2 = COLOR_RANGES['UPPER_RED_2']
LOWER_GREEN = COLOR_RANGES['LOWER_GREEN']
UPPER_GREEN = COLOR_RANGES['UPPER_GREEN']
LOWER_BLACK = COLOR_RANGES['LOWER_BLACK']
UPPER_BLACK = COLOR_RANGES['UPPER_BLACK']
LOWER_ORANGE = COLOR_RANGES['LOWER_ORANGE']
UPPER_ORANGE = COLOR_RANGES['UPPER_ORANGE']
LOWER_MAGENTA = COLOR_RANGES['LOWER_MAGENTA']
UPPER_MAGENTA = COLOR_RANGES['UPPER_MAGENTA']
LOWER_BLUE = COLOR_RANGES['LOWER_BLUE']
UPPER_BLUE = COLOR_RANGES['UPPER_BLUE']
target = 0
detection_params = {'min_area': 300, 'return_rule': 'biggest_in_job', 'return_mask': True}
WALL_MIN_AREA = detection_params['min_area']
BLOCK_MIN_AREA = 250
MAGENTA_MIN_AREA = 300
CLOSE_BLOCK_MIN_AREA = 15

left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 130, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 130, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 155, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 155 , 100, 100
line_roi_x, line_roi_y, line_roi_w, line_roi_h = 280, 190, 80, 40
close_x,close_y,close_w,close_h = 140,110,360,10
full_frame_roi = (0, 80, 640, 170)
close_block_roi = (280, 215, 80, 10)

left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}

roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
for job in [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]:
    x, y, w, h = job['roi']
    cv2.rectangle(roi_mask_walls, (x, y), (x + w, y + h), 255, -1)

roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = full_frame_roi
cv2.rectangle(roi_mask_main_blocks, (x, y), (x + w, y + h), 255, -1)

roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = close_block_roi
cv2.rectangle(roi_mask_close_blocks, (x, y), (x + w, y + h), 255, -1)

roi_mask_line = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_line, (line_roi_x, line_roi_y), (line_roi_x + line_roi_w, line_roi_h + line_roi_y), 255, -1)

roi_mask_magenta = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = full_frame_roi
cv2.rectangle(roi_mask_magenta, (x, y), (x + w, y + h), 255, -1)

roi_mask_close_black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_close_black, (close_x, close_y), (close_x + close_w, close_y + close_h), 255, -1)

# The vertical span every ROI lives inside. Hoisted to module level (it used to be
# recomputed inside process_video_frame every frame) because the worker processes
# need it to size their shared-memory output buffers.
GLOBAL_Y_OFFSET = min(
    left_roi_y, right_roi_y, inner_left_roi_y, inner_right_roi_y,
    line_roi_y, close_y, full_frame_roi[1], close_block_roi[1]
)
GLOBAL_Y_END = max(
    left_roi_y + left_roi_h, right_roi_y + right_roi_h,
    inner_left_roi_y + inner_left_roi_h, inner_right_roi_y + inner_right_roi_h,
    line_roi_y + line_roi_h, close_y + close_h,
    full_frame_roi[1] + full_frame_roi[3], close_block_roi[1] + close_block_roi[3]
)
SLICE_HEIGHT = GLOBAL_Y_END - GLOBAL_Y_OFFSET


# Loop temporaries from the ROI-mask construction above. Deleted so that
# `from tuning import *` exports constants only.
del job, x, y, w, h
