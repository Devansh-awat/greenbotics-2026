"""Frame processing: arena mask, colour thresholding, detection and annotation.

`process_video_frame()` is the entry point -- it turns one BGR frame into the
detections dict the control loop steers on. When the vision pool is running, the
arena mask and the colour masks are computed concurrently in two worker processes
and joined here (see pool.py); the results are identical either way.

Nothing in this module touches hardware, so it is safe to import and exercise on
saved frames. Shared by both the obstacle and open challenges.
"""

import ast
import base64
import math
import cv2
import numpy as np

from src.logs.setup import vlog
from src.obstacle_challenge import tuning as _default_tuning

# The live VisionPool, or None to process inline. Set by each challenge's main at
# startup; pool.py imports this module, so it must not be imported from here.
vision_pool = None

# --- Arena mask constants ---
USE_ARENA_MASK = True
ARENA_SEED_PT = (320, 230)
ARENA_TOP_MARGIN = 0
MAX_WALL_RUN = 160
MIN_WALL_THICK = 8
WALL_GAP_SEAL = 3
ARENA_CLOSE_KERNEL = np.ones((9, 9), np.uint8)
ARENA_SKY_SMOOTH = 31

USE_GROUND_CONTACT = True
GROUND_PROBE_DY = 12
GROUND_CONTACT_MIN = 0.5

# Placeholders for configurable parameters
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = 320
USE_LAB = False
USE_BILATERAL = True
BILATERAL_D = 5
BILATERAL_SIGMA_COLOR = 50
BILATERAL_SIGMA_SPACE = 50
HSV_BEFORE_BLUR = True
USE_BLOCK_MORPH_CLOSE = True
BLOCK_CLOSE_KERNEL = np.ones((5, 5), np.uint8)
USE_VISION_POOL = True
VISION_POOL_TIMEOUT = 0.050
VISION_WORKER_THREADS = 2

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 95, 70])
LOWER_ORANGE = np.array([6, 50, 182])
UPPER_ORANGE = np.array([15, 255, 255])
LOWER_BLUE = np.array([114, 50, 110])
UPPER_BLUE = np.array([123, 255, 255])
LOWER_RED_1 = np.array([0, 70, 43])
UPPER_RED_1 = np.array([4, 230, 166])
LOWER_RED_2 = np.array([175, 70, 43])
UPPER_RED_2 = np.array([180, 230, 140])
LOWER_GREEN = np.array([42, 85, 38])
UPPER_GREEN = np.array([88, 190, 135])
LOWER_MAGENTA = np.array([158, 73, 64])
UPPER_MAGENTA = np.array([172, 255, 223])

WALL_MIN_AREA = 300
BLOCK_MIN_AREA = 250
MAGENTA_MIN_AREA = 300
CLOSE_BLOCK_MIN_AREA = 15

left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 130, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 130, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 155, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 155, 100, 100
line_roi_x, line_roi_y, line_roi_w, line_roi_h = 280, 190, 80, 40
close_x, close_y, close_w, close_h = 140, 110, 360, 10
full_frame_roi = (0, 80, 640, 170)
close_block_roi = (280, 215, 80, 10)

left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}
WALL_JOBS = [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]

roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_line = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_close_black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
roi_mask_magenta = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")

GLOBAL_Y_OFFSET = 80
GLOBAL_Y_END = 280
SLICE_HEIGHT = 200

ARENA_Y_TOP = GLOBAL_Y_OFFSET
ARENA_Y_BOTTOM = GLOBAL_Y_END
ARENA_BAND_H = ARENA_Y_BOTTOM - ARENA_Y_TOP
ARENA_SEED_LOCAL = (float(ARENA_SEED_PT[0]), float(ARENA_SEED_PT[1] - ARENA_Y_TOP))
_ARENA_ROWS = np.arange(ARENA_BAND_H, dtype=np.int32)[:, None]
_ARENA_WALL_KERNEL = np.ones((MIN_WALL_THICK, 1), np.uint8)
_ARENA_WALL_CLOSE_KERNEL = np.ones((WALL_GAP_SEAL, 1), np.uint8)
ARENA_PASSTHROUGH = np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype="uint8")

LINE_SLICE_Y0 = line_roi_y - GLOBAL_Y_OFFSET
LINE_SLICE_Y1 = LINE_SLICE_Y0 + line_roi_h
LINE_X0 = line_roi_x
LINE_X1 = line_roi_x + line_roi_w


def configure(cfg):
    """Dynamically configure the vision pipeline with parameters from the specified config/tuning module."""
    global FRAME_WIDTH, FRAME_HEIGHT, FRAME_MIDPOINT_X
    global USE_LAB, USE_BILATERAL, BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE, HSV_BEFORE_BLUR
    global USE_BLOCK_MORPH_CLOSE, BLOCK_CLOSE_KERNEL
    global USE_VISION_POOL, VISION_POOL_TIMEOUT, VISION_WORKER_THREADS
    global LOWER_BLACK, UPPER_BLACK, LOWER_ORANGE, UPPER_ORANGE, LOWER_BLUE, UPPER_BLUE
    global LOWER_RED_1, UPPER_RED_1, LOWER_RED_2, UPPER_RED_2, LOWER_GREEN, UPPER_GREEN, LOWER_MAGENTA, UPPER_MAGENTA
    global WALL_MIN_AREA, BLOCK_MIN_AREA, MAGENTA_MIN_AREA, CLOSE_BLOCK_MIN_AREA
    global left_roi_x, left_roi_y, left_roi_w, left_roi_h
    global right_roi_x, right_roi_y, right_roi_w, right_roi_h
    global inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h
    global inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h
    global line_roi_x, line_roi_y, line_roi_w, line_roi_h
    global close_x, close_y, close_w, close_h
    global full_frame_roi, close_block_roi
    global left_side_job, right_side_job, inner_left_side_job, inner_right_side_job, WALL_JOBS
    global roi_mask_walls, roi_mask_line, roi_mask_close_black, roi_mask_main_blocks, roi_mask_close_blocks, roi_mask_magenta
    global GLOBAL_Y_OFFSET, GLOBAL_Y_END, SLICE_HEIGHT
    global ARENA_Y_TOP, ARENA_Y_BOTTOM, ARENA_BAND_H, _ARENA_ROWS, _ARENA_WALL_KERNEL, _ARENA_WALL_CLOSE_KERNEL, ARENA_PASSTHROUGH, ARENA_SEED_LOCAL
    global LINE_SLICE_Y0, LINE_SLICE_Y1, LINE_X0, LINE_X1

    FRAME_WIDTH = getattr(cfg, 'FRAME_WIDTH', 640)
    FRAME_HEIGHT = getattr(cfg, 'FRAME_HEIGHT', 360)
    FRAME_MIDPOINT_X = getattr(cfg, 'FRAME_MIDPOINT_X', FRAME_WIDTH // 2)

    USE_LAB = getattr(cfg, 'USE_LAB', False)
    USE_BILATERAL = getattr(cfg, 'USE_BILATERAL', True)
    BILATERAL_D = getattr(cfg, 'BILATERAL_D', 5)
    BILATERAL_SIGMA_COLOR = getattr(cfg, 'BILATERAL_SIGMA_COLOR', 50)
    BILATERAL_SIGMA_SPACE = getattr(cfg, 'BILATERAL_SIGMA_SPACE', 50)
    HSV_BEFORE_BLUR = getattr(cfg, 'HSV_BEFORE_BLUR', True)
    USE_BLOCK_MORPH_CLOSE = getattr(cfg, 'USE_BLOCK_MORPH_CLOSE', True)
    BLOCK_CLOSE_KERNEL = getattr(cfg, 'BLOCK_CLOSE_KERNEL', np.ones((5, 5), np.uint8))
    USE_VISION_POOL = getattr(cfg, 'USE_VISION_POOL', True)
    VISION_POOL_TIMEOUT = getattr(cfg, 'VISION_POOL_TIMEOUT', 0.050)
    VISION_WORKER_THREADS = getattr(cfg, 'VISION_WORKER_THREADS', 2)

    LOWER_BLACK = getattr(cfg, 'LOWER_BLACK', np.array([0, 0, 0]))
    UPPER_BLACK = getattr(cfg, 'UPPER_BLACK', np.array([180, 95, 70]))
    LOWER_ORANGE = getattr(cfg, 'LOWER_ORANGE', np.array([6, 50, 182]))
    UPPER_ORANGE = getattr(cfg, 'UPPER_ORANGE', np.array([15, 255, 255]))
    LOWER_BLUE = getattr(cfg, 'LOWER_BLUE', np.array([114, 50, 110]))
    UPPER_BLUE = getattr(cfg, 'UPPER_BLUE', np.array([123, 255, 255]))

    LOWER_RED_1 = getattr(cfg, 'LOWER_RED_1', np.array([0, 70, 43]))
    UPPER_RED_1 = getattr(cfg, 'UPPER_RED_1', np.array([4, 230, 166]))
    LOWER_RED_2 = getattr(cfg, 'LOWER_RED_2', np.array([175, 70, 43]))
    UPPER_RED_2 = getattr(cfg, 'UPPER_RED_2', np.array([180, 230, 140]))
    LOWER_GREEN = getattr(cfg, 'LOWER_GREEN', np.array([42, 85, 38]))
    UPPER_GREEN = getattr(cfg, 'UPPER_GREEN', np.array([88, 190, 135]))
    LOWER_MAGENTA = getattr(cfg, 'LOWER_MAGENTA', np.array([158, 73, 64]))
    UPPER_MAGENTA = getattr(cfg, 'UPPER_MAGENTA', np.array([172, 255, 223]))

    WALL_MIN_AREA = getattr(cfg, 'WALL_MIN_AREA', 300)
    BLOCK_MIN_AREA = getattr(cfg, 'BLOCK_MIN_AREA', 250)
    MAGENTA_MIN_AREA = getattr(cfg, 'MAGENTA_MIN_AREA', 300)
    CLOSE_BLOCK_MIN_AREA = getattr(cfg, 'CLOSE_BLOCK_MIN_AREA', 15)

    left_roi_x = getattr(cfg, 'left_roi_x', 0)
    left_roi_y = getattr(cfg, 'left_roi_y', 130)
    left_roi_w = getattr(cfg, 'left_roi_w', 135)
    left_roi_h = getattr(cfg, 'left_roi_h', 150)

    right_roi_x = getattr(cfg, 'right_roi_x', 505)
    right_roi_y = getattr(cfg, 'right_roi_y', 130)
    right_roi_w = getattr(cfg, 'right_roi_w', 135)
    right_roi_h = getattr(cfg, 'right_roi_h', 150)

    inner_left_roi_x = getattr(cfg, 'inner_left_roi_x', 140)
    inner_left_roi_y = getattr(cfg, 'inner_left_roi_y', 155)
    inner_left_roi_w = getattr(cfg, 'inner_left_roi_w', 100)
    inner_left_roi_h = getattr(cfg, 'inner_left_roi_h', 100)

    inner_right_roi_x = getattr(cfg, 'inner_right_roi_x', 400)
    inner_right_roi_y = getattr(cfg, 'inner_right_roi_y', 155)
    inner_right_roi_w = getattr(cfg, 'inner_right_roi_w', 100)
    inner_right_roi_h = getattr(cfg, 'inner_right_roi_h', 100)

    line_roi_x = getattr(cfg, 'line_roi_x', 280)
    line_roi_y = getattr(cfg, 'line_roi_y', 190)
    line_roi_w = getattr(cfg, 'line_roi_w', 80)
    line_roi_h = getattr(cfg, 'line_roi_h', 40)

    close_x = getattr(cfg, 'close_x', 140)
    close_y = getattr(cfg, 'close_y', 110)
    close_w = getattr(cfg, 'close_w', 360)
    close_h = getattr(cfg, 'close_h', 10)

    full_frame_roi = getattr(cfg, 'full_frame_roi', (0, 80, 640, 170))
    close_block_roi = getattr(cfg, 'close_block_roi', (280, 215, 80, 10))

    left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
    right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
    inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
    inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}
    WALL_JOBS = [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]

    roi_mask_walls = getattr(cfg, 'roi_mask_walls', None)
    if roi_mask_walls is None:
        roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
        for job in WALL_JOBS:
            jx, jy, jw, jh = job['roi']
            cv2.rectangle(roi_mask_walls, (jx, jy), (jx + jw, jy + jh), 255, -1)

    roi_mask_line = getattr(cfg, 'roi_mask_line', None)
    if roi_mask_line is None:
        roi_mask_line = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
        cv2.rectangle(roi_mask_line, (line_roi_x, line_roi_y), (line_roi_x + line_roi_w, line_roi_y + line_roi_h), 255, -1)

    roi_mask_close_black = getattr(cfg, 'roi_mask_close_black', None)
    if roi_mask_close_black is None:
        roi_mask_close_black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
        cv2.rectangle(roi_mask_close_black, (close_x, close_y), (close_x + close_w, close_y + close_h), 255, -1)

    roi_mask_main_blocks = getattr(cfg, 'roi_mask_main_blocks', None)
    if roi_mask_main_blocks is None:
        roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
        if full_frame_roi and full_frame_roi[2] > 0 and full_frame_roi[3] > 0:
            fx, fy, fw, fh = full_frame_roi
            cv2.rectangle(roi_mask_main_blocks, (fx, fy), (fx + fw, fy + fh), 255, -1)

    roi_mask_close_blocks = getattr(cfg, 'roi_mask_close_blocks', None)
    if roi_mask_close_blocks is None:
        roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
        if close_block_roi and close_block_roi[2] > 0 and close_block_roi[3] > 0:
            cx, cy, cw, ch = close_block_roi
            cv2.rectangle(roi_mask_close_blocks, (cx, cy), (cx + cw, cy + ch), 255, -1)

    roi_mask_magenta = getattr(cfg, 'roi_mask_magenta', None)
    if roi_mask_magenta is None:
        roi_mask_magenta = roi_mask_main_blocks.copy()

    GLOBAL_Y_OFFSET = getattr(cfg, 'GLOBAL_Y_OFFSET', None)
    if GLOBAL_Y_OFFSET is None:
        rois_y0 = [left_roi_y, right_roi_y, inner_left_roi_y, inner_right_roi_y, line_roi_y, close_y]
        if full_frame_roi and full_frame_roi[3] > 0: rois_y0.append(full_frame_roi[1])
        if close_block_roi and close_block_roi[3] > 0: rois_y0.append(close_block_roi[1])
        GLOBAL_Y_OFFSET = min(rois_y0)

    GLOBAL_Y_END = getattr(cfg, 'GLOBAL_Y_END', None)
    if GLOBAL_Y_END is None:
        rois_y1 = [left_roi_y + left_roi_h, right_roi_y + right_roi_h,
                   inner_left_roi_y + inner_left_roi_h, inner_right_roi_y + inner_right_roi_h,
                   line_roi_y + line_roi_h, close_y + close_h]
        if full_frame_roi and full_frame_roi[3] > 0: rois_y1.append(full_frame_roi[1] + full_frame_roi[3])
        if close_block_roi and close_block_roi[3] > 0: rois_y1.append(close_block_roi[1] + close_block_roi[3])
        GLOBAL_Y_END = max(rois_y1)

    SLICE_HEIGHT = GLOBAL_Y_END - GLOBAL_Y_OFFSET

    ARENA_Y_TOP = GLOBAL_Y_OFFSET
    ARENA_Y_BOTTOM = GLOBAL_Y_END
    ARENA_BAND_H = ARENA_Y_BOTTOM - ARENA_Y_TOP
    ARENA_SEED_LOCAL = (float(ARENA_SEED_PT[0]), float(ARENA_SEED_PT[1] - ARENA_Y_TOP))
    _ARENA_ROWS = np.arange(ARENA_BAND_H, dtype=np.int32)[:, None]
    _ARENA_WALL_KERNEL = np.ones((MIN_WALL_THICK, 1), np.uint8)
    _ARENA_WALL_CLOSE_KERNEL = np.ones((WALL_GAP_SEAL, 1), np.uint8)
    ARENA_PASSTHROUGH = np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype="uint8")

    LINE_SLICE_Y0 = line_roi_y - GLOBAL_Y_OFFSET
    LINE_SLICE_Y1 = LINE_SLICE_Y0 + line_roi_h
    LINE_X0 = line_roi_x
    LINE_X1 = line_roi_x + line_roi_w

# Initialise defaults using obstacle challenge tuning
configure(_default_tuning)


def _sliding_median(arr, window):
    """1-D sliding median over the skyline.

    This MUST be a median, not a minimum. A minimum filter can only ever expand the
    accepted region, and it propagates the single most-leaked column across its whole
    window -- one column where the floor blob escapes above the wall drags `window`
    neighbours to the top of the band with it, which collapses the whole mask into a
    rectangle. A median rejects isolated bad columns instead of spreading them.
    """
    if window <= 1:
        return arr
    pad = window // 2
    padded = np.pad(arr, pad, mode='edge')
    win = np.lib.stride_tricks.sliding_window_view(padded, window)
    return np.median(win, axis=-1).astype(np.int32)


def build_arena_mask_from_prepared(cs_slice):
    """
    Same as build_arena_mask, but takes the already-cropped, already colour-converted
    and already-filtered slice from prepare_colour_slice() instead of a raw frame.

    ARENA_Y_TOP/ARENA_Y_BOTTOM are now exactly GLOBAL_Y_OFFSET/GLOBAL_Y_END (see their
    definitions above), so cs_slice already IS the arena band -- no re-cropping here.
    This also computes mask_black once and returns it, since compute_colour_masks_
    from_prepared() needs the identical black threshold and must not re-derive it.

    Returns (arena_mask, floor_mask, sky, mask_black), or (None, None, None,
    mask_black) if the seed point isn't on any floor blob (camera covered, nose into
    a wall) -- caller falls back to pass-through so a bad frame degrades to main_v2
    behaviour instead of blanking every detection. mask_black is always valid since
    it's computed before the seed test.

    arena_mask : full-frame uint8. Inside the band it is the solid skyline region.
                 ABOVE the band it is 0 -- ARENA_Y_TOP is a hard clamp, nothing up
                 there is ever arena. BELOW the band it is 255 (pass-through) so the
                 wall ROIs, which run to y=280, are never clipped.
    floor_mask : full-frame uint8, the drivable mat with interior holes PRESERVED.
                 Used only by the ground-contact test -- the solid arena fill would
                 make that test pass trivially.
    sky        : per-column top boundary, band-local. Annotation only.
    """
    # 1. Floor candidate. Black walls are the only thing excluded -- pillars, floor
    #    lines and magenta parking walls all pass and stay part of the blob.
    mask_black = cv2.inRange(cs_slice, LOWER_BLACK, UPPER_BLACK)
    # UPPER_BLACK's S/V ceiling was widened to still catch the near wall under glare
    # (see the frame-293 investigation), and that ceiling now also catches shadowed
    # green block pixels (low S/V, but not black). A green pixel misread as black can
    # form its own >=MIN_WALL_THICK run in the skyline scan below, planting a false
    # wall boundary on the block's own edge -- the block gates itself out. One extra
    # inRange() call is cheap; recomputing the block's actual boundary from that false
    # "wall" is not something the smoothing below can be relied on to always erase.
    mask_black = cv2.bitwise_and(
        mask_black, cv2.bitwise_not(cv2.inRange(cs_slice, LOWER_GREEN, UPPER_GREEN))
    )
    floor_cand = cv2.bitwise_not(mask_black)
    # Closing seals seams/glare/cable shadows in the mat. Careful raising this: a
    # kernel wide enough to bridge the arena wall merges the far section into our
    # blob and silently defeats the whole mechanism.
    floor_cand = cv2.morphologyEx(floor_cand, cv2.MORPH_CLOSE, ARENA_CLOSE_KERNEL)

    # 2. Keep only the blob reachable from the robot's nose, then take the largest.
    contours, _ = cv2.findContours(floor_cand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    seeded = [c for c in contours if cv2.pointPolygonTest(c, ARENA_SEED_LOCAL, False) >= 0]
    if not seeded:
        return None, None, None, mask_black
    cnt = max(seeded, key=cv2.contourArea)

    filled = np.zeros_like(floor_cand)
    cv2.drawContours(filled, [cnt], -1, 255, -1)
    floor = cv2.bitwise_and(floor_cand, filled)   # fill-then-AND: interior holes come back

    # 3. Skyline, by PER-COLUMN WALL SCAN.
    #
    #    Deriving the top boundary from the floor blob's upper edge does not work.
    #    The room beyond the arena is bright, so it passes NOT(black) too, and the
    #    wall is a thin barrier -- wherever it thins out or leaves frame, the blob
    #    escapes into the room and floor_top collapses to 0 for that column. Verified
    #    on real frames: the mask degenerated into a full-band rectangle.
    #
    #    Instead, scan each column upward from the bottom for the first SUSTAINED
    #    black run -- that is the near wall -- and take the top of that run. This is
    #    purely local per column, so no amount of sideways leakage can affect it, and
    #    anything above the near wall (far track section, room, shoes) is excluded by
    #    construction.
    #
    #    The vertical MORPH_OPEN keeps only black runs >= MIN_WALL_THICK px tall, so
    #    speckle, floor-line shadows and pillar edges can't be mistaken for a wall.
    #    Seal small BRIGHT breaks inside the wall first. The upward walk below needs
    #    a strictly contiguous black run, so a single bright pixel -- glare, a mat
    #    line reflecting off the wall, sensor noise, a JPEG artifact -- stops it dead.
    #    Different columns break at different heights, which made the top boundary
    #    visibly jagged while the bottom edge stayed smooth, and left wall_top a
    #    median of 12px BELOW the topmost black pixel in the same column.
    wall_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, _ARENA_WALL_CLOSE_KERNEL)
    thick = cv2.morphologyEx(wall_black, cv2.MORPH_OPEN, _ARENA_WALL_KERNEL)
    thick_b = thick > 0
    has = thick_b.any(axis=0)

    # lowest row of a wall run in each column, then walk up to the top of that run.
    #
    # This block is ~1.0 ms, a third of the whole arena mask, so it is the obvious
    # thing to try to optimise. Do not bother: a cumsum/searchsorted reformulation
    # (1.06 ms) and an OpenCV/contiguous-flip version (1.45 ms) were both measured
    # SLOWER than this on 100 real frames. np.minimum.accumulate is already good.
    lowest = (ARENA_BAND_H - 1) - np.argmax(thick_b[::-1], axis=0)
    chain = thick_b | (_ARENA_ROWS > lowest)     # force everything below `lowest` True
    run = np.minimum.accumulate(chain[::-1], axis=0)[::-1]
    wall_top = ARENA_BAND_H - run.sum(axis=0)

    # A run longer than MAX_WALL_RUN means the wall has probably merged into a dark
    # background above it, so we can't locate its top edge in this column.
    #
    # Do NOT clamp to `lowest - MAX_WALL_RUN`: that lands the boundary in the MIDDLE
    # of the wall, which is the worst of both worlds -- it neither includes the wall
    # nor excludes what's beyond it -- and it makes the skyline track the wall's
    # ragged BOTTOM edge, which is where the jagged mid-wall boundary came from.
    # Mark the column untrusted instead and let neighbours supply the value.
    trusted = has & ((lowest - wall_top) <= MAX_WALL_RUN)

    # `has` False means no black run >= MIN_WALL_THICK turned up anywhere in the
    # column -- e.g. glare washing out the near wall -- so there is no candidate
    # boundary to distrust in the first place. The old code filled these with the
    # median of trusted columns, borrowed from wherever a wall WAS found elsewhere in
    # the frame; that value has no relation to this column and can (and did, on a
    # real run) land below a floor object here, gating it out of every colour mask.
    # Leave the column fully open instead -- sky=0 excludes nothing -- so an
    # undetectable wall degrades to "don't gate this column" rather than "silently
    # gate it wrong". `has` True but untrusted (wall merged into background) is a
    # different failure -- there IS a wall, just not a locatable top edge -- and
    # still borrows the trusted-column median, unchanged from before.
    fill = int(np.median(wall_top[trusted])) if trusted.any() else 0
    sky = np.where(has, np.where(trusted, wall_top, fill), 0).astype(np.int32)
    sky = sky + ARENA_TOP_MARGIN
    sky = _sliding_median(sky, ARENA_SKY_SMOOTH)
    sky = np.clip(sky, 0, ARENA_BAND_H)

    band_mask = ((_ARENA_ROWS >= sky).astype(np.uint8)) * 255

    arena_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
    arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :] = band_mask
    arena_mask[ARENA_Y_BOTTOM:, :] = 255   # below the band: pass-through, never clip

    # Bound the floor blob by the arena region too -- if it did leak into the room,
    # those pixels must not be able to validate a block in the ground-contact test.
    floor_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
    floor_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :] = cv2.bitwise_and(floor, band_mask)

    return arena_mask, floor_mask, sky, mask_black


def build_arena_mask(frame):
    """Standalone entry point for callers that don't already have a prepared slice
    (the capture_*_pipeline.py diagnostic tools). Production code (pool.py,
    process_video_frame's inline fallback) should call prepare_colour_slice() once
    and pass the result to build_arena_mask_from_prepared() directly instead, so the
    HSV/bilateral conversion never runs twice for one frame."""
    cs_slice = prepare_colour_slice(frame)
    arena_mask, floor_mask, sky, _mask_black = build_arena_mask_from_prepared(cs_slice)
    return arena_mask, floor_mask, sky

# ---------------------------------------------------------------------------
# Colour thresholding (the half of the pipeline that does NOT need the arena mask)
# ---------------------------------------------------------------------------

COLOUR_MASK_NAMES = ('red', 'green', 'magenta', 'orange', 'blue')  # black comes from
                                                                     # build_arena_mask_from_prepared
MASK_NAMES = COLOUR_MASK_NAMES + ('black',)

# Orange and blue are ONLY ever read inside the line ROI (an 80x40 patch), so they
# are thresholded on that patch alone rather than over the whole 640x200 slice --
# 40x fewer pixels. The rest of their mask planes stay zero, which is exactly what
# the full-slice version would have produced there anyway as far as any consumer is
# concerned: global_blue_mask is built only from the line crop, and orange is only
# read as mask_orange_line.
LINE_SLICE_Y0 = line_roi_y - GLOBAL_Y_OFFSET
LINE_SLICE_Y1 = LINE_SLICE_Y0 + line_roi_h
LINE_X0 = line_roi_x
LINE_X1 = line_roi_x + line_roi_w


def filter_slice(frame):
    """Crop the working slice and run the edge-preserving/blur step on it.

    Split out of compute_colour_masks() so callers that just want to see (or
    debug) the filtered image -- e.g. src/tools/capture_*_pipeline.py -- don't
    have to re-derive it by hand and risk drifting from the real pipeline.
    """
    frame_slice = frame[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
    if USE_BILATERAL:
        # Edge-preserving: keeps pillar boundaries crisp where GaussianBlur bleeds
        # pillar colour into the mat and vice versa.
        frame_slice = cv2.bilateralFilter(
            frame_slice, BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE
        )
    else:
        frame_slice = cv2.GaussianBlur(frame_slice, (1, 7), 0)
    return frame_slice


def prepare_colour_slice(frame):
    """Crop the shared working slice, colour-convert it and filter it -- ONCE.

    The one piece of work both build_arena_mask_from_prepared() and
    compute_colour_masks_from_prepared()/compute_colour_masks_only() need. Call this
    once per frame and pass its result to both; do not let either branch re-derive
    HSV/bilateral independently, or they silently drift back into duplicate work
    (which is the bug this function exists to prevent -- see pool.py for how the
    fork-join shares this result across both worker processes).

    ARENA_Y_TOP/ARENA_Y_BOTTOM are exactly GLOBAL_Y_OFFSET/GLOBAL_Y_END (see
    pipeline.py's arena-mask constants), so this slice doubles as the arena band with
    no further cropping needed.
    """
    if HSV_BEFORE_BLUR:
        raw_slice = frame[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
        cs_raw = cv2.cvtColor(raw_slice, cv2.COLOR_BGR2Lab if USE_LAB else cv2.COLOR_BGR2HSV)
        if USE_BILATERAL:
            cs_slice = cv2.bilateralFilter(
                cs_raw, BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE
            )
        else:
            cs_slice = cv2.GaussianBlur(cs_raw, (1, 7), 0)
    else:
        frame_slice = filter_slice(frame)
        cs_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2Lab if USE_LAB else cv2.COLOR_BGR2HSV)
    return cs_slice


def compute_colour_masks_only(cs_slice, out=None):
    """Threshold the prepared slice into the FIVE non-black colour masks. Black is
    owned by build_arena_mask_from_prepared() (it needs it for the wall scan anyway)
    -- see compute_colour_masks_from_prepared() for the caller that stitches both
    together into the full six-mask dict.

    Every operation here is pointwise, so thresholding the whole slice and cropping
    afterwards is identical to the old code's crop-then-threshold -- which is what
    lets this run in a worker process independently of the arena mask.

    `out`, if given, is a (5, SLICE_HEIGHT, FRAME_WIDTH) uint8 view into shared
    memory to write into directly, avoiding an allocation + copy per frame.
    """
    red = cv2.inRange(cs_slice, LOWER_RED_1, UPPER_RED_1)
    if not USE_LAB:
        # In HSV, red wraps around 180->0, so it needs two ranges combined.
        # In LAB red is continuous and the first range is the whole story.
        cv2.bitwise_or(red, cv2.inRange(cs_slice, LOWER_RED_2, UPPER_RED_2), dst=red)

    green = cv2.inRange(cs_slice, LOWER_GREEN, UPPER_GREEN)

    if USE_BLOCK_MORPH_CLOSE:
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, BLOCK_CLOSE_KERNEL)
        green = cv2.morphologyEx(green, cv2.MORPH_CLOSE, BLOCK_CLOSE_KERNEL)

    if out is None:
        orange = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), np.uint8)
        blue = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), np.uint8)
    else:
        orange, blue = out[3], out[4]
        orange[:] = 0
        blue[:] = 0

    line_cs = cs_slice[LINE_SLICE_Y0:LINE_SLICE_Y1, LINE_X0:LINE_X1]
    orange[LINE_SLICE_Y0:LINE_SLICE_Y1, LINE_X0:LINE_X1] = cv2.inRange(
        line_cs, LOWER_ORANGE, UPPER_ORANGE)
    blue[LINE_SLICE_Y0:LINE_SLICE_Y1, LINE_X0:LINE_X1] = cv2.inRange(
        line_cs, LOWER_BLUE, UPPER_BLUE)

    masks = {
        'red': red,
        'green': green,
        'magenta': cv2.inRange(cs_slice, LOWER_MAGENTA, UPPER_MAGENTA),
        'orange': orange,
        'blue': blue,
    }
    if out is not None:
        # orange/blue were written in place above; copy the rest.
        for i, name in enumerate(COLOUR_MASK_NAMES):
            if name not in ('orange', 'blue'):
                out[i] = masks[name]
    return masks


def compute_colour_masks_from_prepared(cs_slice, mask_black, out=None):
    """The full six-mask dict: the five colour masks plus the black mask supplied by
    the caller (normally build_arena_mask_from_prepared()'s output) -- never
    re-thresholded here."""
    masks = compute_colour_masks_only(cs_slice, out=out)
    masks['black'] = mask_black
    return masks


def compute_colour_masks(frame, out=None):
    """Standalone entry point for callers that don't already have a prepared slice
    (the capture_*_pipeline.py diagnostic tools). Production code (pool.py,
    process_video_frame's inline fallback) should call prepare_colour_slice() once
    and pass the result to compute_colour_masks_from_prepared() directly instead."""
    cs_slice = prepare_colour_slice(frame)
    mask_black = cv2.inRange(cs_slice, LOWER_BLACK, UPPER_BLACK)
    return compute_colour_masks_from_prepared(cs_slice, mask_black, out=out)


def process_video_frame(frame):
    """Detect walls, pillars, floor lines and the arena boundary in one frame.

    prepare_colour_slice() (crop + colour-convert + bilateral) runs once, in this
    process, then two branches run concurrently when the pool is up:
      - build_arena_mask_from_prepared    -- worker 'vision-arena' (also owns the
        shared black threshold, since it needs it for the wall scan anyway)
      - compute_colour_masks_only         -- worker 'vision-colour' (red/green/
        magenta/orange/blue; black comes from the arena worker, not re-thresholded)
    The two branches are independent of each other until the bitwise_and below, so
    this is still a fork-join, not a pipeline: the frame we act on is the frame we
    just captured, never the previous one. Contour extraction and all decisions stay
    in this process.
    """
    processed_data = {
        'detected_blocks': [],
        'detected_walls': [],
        'detected_orange': [],
        'detected_blue': [],
        'detected_magenta': [],
        'detected_close_black': []
    }

    my_slice = max(0, full_frame_roi[1] - GLOBAL_Y_OFFSET)
    ly_slice = line_roi_y - GLOBAL_Y_OFFSET
    cy_slice = close_block_roi[1] - GLOBAL_Y_OFFSET

    # --- 0. Colour masks + arena mask (concurrent when the pool is up) ---
    pooled = None
    if USE_VISION_POOL and vision_pool is not None and vision_pool.ok:
        pooled = vision_pool.process(frame)

    if pooled is not None:
        masks, arena_mask, floor_mask, arena_sky, seeded = pooled
        if not seeded:
            arena_mask, floor_mask, arena_sky = None, None, None
    else:
        # Inline fallback (pool disabled/unavailable): prepare the shared slice once
        # and hand it to both branches, same as the pooled path does -- do NOT call
        # compute_colour_masks(frame)/build_arena_mask(frame) here, they'd each
        # independently re-derive HSV/bilateral and silently reintroduce the
        # duplicate work this refactor removes.
        cs_slice = prepare_colour_slice(frame)
        arena_mask = floor_mask = arena_sky = None
        mask_black = None
        if USE_ARENA_MASK:
            arena_mask, floor_mask, arena_sky, mask_black = build_arena_mask_from_prepared(cs_slice)
        if mask_black is None:
            mask_black = cv2.inRange(cs_slice, LOWER_BLACK, UPPER_BLACK)
        masks = compute_colour_masks_from_prepared(cs_slice, mask_black)

    if not USE_ARENA_MASK:
        arena_mask, floor_mask, arena_sky = None, None, None

    # Gates the colour masks BEFORE findContours, so nothing outside the arena can
    # ever become a contour -- and therefore can never win the
    # max(contours, key=contourArea) pick inside process_block_contours().
    if arena_mask is None:
        arena_mask = ARENA_PASSTHROUGH
        floor_mask = None
    processed_data['arena_mask'] = arena_mask
    processed_data['arena_sky'] = arena_sky
    processed_data['arena_seeded'] = floor_mask is not None

    # --- 1. Crop the slice-sized masks down to each ROI ---
    mx, my, mw, mh = full_frame_roi
    lx, ly, lw, lh = line_roi_x, line_roi_y, line_roi_w, line_roi_h
    cx, cy, cw, ch = close_block_roi

    def crop(mask, y0, h, x0, w):
        return mask[y0:y0 + h, x0:x0 + w]

    mask_red_main = crop(masks['red'], my_slice, mh, mx, mw)
    mask_green_main = crop(masks['green'], my_slice, mh, mx, mw)
    mask_magenta_main = crop(masks['magenta'], my_slice, mh, mx, mw)

    mask_orange_line = crop(masks['orange'], ly_slice, lh, lx, lw)
    mask_blue_line = crop(masks['blue'], ly_slice, lh, lx, lw)

    mask_red_close = crop(masks['red'], cy_slice, ch, cx, cw)
    mask_green_close = crop(masks['green'], cy_slice, ch, cx, cw)
    mask_magenta_close = crop(masks['magenta'], cy_slice, ch, cx, cw)

    # Gate each to the arena. All of these ROIs sit inside the arena band.
    if mw > 0 and mh > 0:
        arena_main = arena_mask[my:my + mh, mx:mx + mw]
        mask_red_main = cv2.bitwise_and(mask_red_main, arena_main)
        mask_green_main = cv2.bitwise_and(mask_green_main, arena_main)
        mask_magenta_main = cv2.bitwise_and(mask_magenta_main, arena_main)

    if lw > 0 and lh > 0:
        arena_line = arena_mask[ly:ly + lh, lx:lx + lw]
        mask_orange_line = cv2.bitwise_and(mask_orange_line, arena_line)
        mask_blue_line = cv2.bitwise_and(mask_blue_line, arena_line)

    if cw > 0 and ch > 0:
        arena_close = arena_mask[cy:cy + ch, cx:cx + cw]
        mask_red_close = cv2.bitwise_and(mask_red_close, arena_close)
        mask_green_close = cv2.bitwise_and(mask_green_close, arena_close)
        mask_magenta_close = cv2.bitwise_and(mask_magenta_close, arena_close)

    # --- 2. Reconstruct slice-sized global masks for wall/black detection ---
    global_red_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_green_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_blue_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_magenta_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")

    if mw > 0 and mh > 0:
        global_red_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_red_mask[my_slice:my_slice+mh, mx:mx+mw], mask_red_main)
        global_green_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_green_mask[my_slice:my_slice+mh, mx:mx+mw], mask_green_main)
        global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw], mask_magenta_main)

    if cw > 0 and ch > 0:
        global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_red_close)
        global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_green_close)
        global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_magenta_close)

    if lw > 0 and lh > 0:
        global_blue_mask[ly_slice:ly_slice+lh, lx:lx+lw] = cv2.bitwise_or(global_blue_mask[ly_slice:ly_slice+lh, lx:lx+lw], mask_blue_line)

    # --- 3. Wall and black detection ---
    mask_black = masks['black']

    mask_red_or_green = cv2.bitwise_or(global_red_mask, global_green_mask)
    mask_red_or_green_or_blue = cv2.bitwise_or(mask_red_or_green, global_blue_mask)

    pure_black_mask = cv2.bitwise_and(mask_black, cv2.bitwise_not(mask_red_or_green_or_blue))
    black_or_magenta_mask = cv2.bitwise_or(pure_black_mask, global_magenta_mask)

    roi_mask_walls_slice = roi_mask_walls[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
    roi_mask_close_black_slice = roi_mask_close_black[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]

    final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls_slice)
    final_mask_close_black = cv2.bitwise_and(black_or_magenta_mask, roi_mask_close_black_slice)

    # --- 4. Contour finding (fast-fail on empty masks) ---

    if cv2.countNonZero(mask_magenta_main) > 0:
        contours, _ = cv2.findContours(mask_magenta_main, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MAGENTA_MIN_AREA:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    ccx = int(M["m10"] / M["m00"]) + mx
                    ccy = int(M["m01"] / M["m00"]) + (GLOBAL_Y_OFFSET + my_slice)
                    contour_global = contour + [mx, GLOBAL_Y_OFFSET + my_slice]

                    leftmost_x = contour_global[:, 0, 0].min()
                    rightmost_x = contour_global[:, 0, 0].max()

                    dist_to_center_left = abs(leftmost_x - FRAME_MIDPOINT_X)
                    dist_to_center_right = abs(rightmost_x - FRAME_MIDPOINT_X)

                    if dist_to_center_left <= dist_to_center_right:
                        target_x = leftmost_x
                    else:
                        target_x = rightmost_x

                    processed_data['detected_magenta'].append({
                        'type': 'magenta_block',
                        'area': area,
                        'centroid': (ccx, ccy),
                        'contour': contour_global,
                        'target_x': target_x,
                        'target_y': ccy
                    })

    def process_block_contours(mask, offset_x, offset_y, b_type, b_color, min_area):
        blocks = []
        if cv2.countNonZero(mask) > 0:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > min_area:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        bcx = int(M["m10"] / M["m00"]) + offset_x
                        bcy = int(M["m01"] / M["m00"]) + offset_y
                        cnt_global = cnt + [offset_x, offset_y]

                        if USE_GROUND_CONTACT and floor_mask is not None and b_type == 'block':
                            rx, ry, rw, rh = cv2.boundingRect(cnt_global)
                            probe_y = ry + rh + GROUND_PROBE_DY
                            if probe_y < ARENA_Y_BOTTOM:
                                strip = floor_mask[probe_y, rx:rx + rw]
                                if strip.size and (np.count_nonzero(strip) / strip.size) < GROUND_CONTACT_MIN:
                                    continue

                        blocks.append({'type': b_type, 'color': b_color, 'area': area,
                                       'centroid': (bcx, bcy), 'contour': cnt_global})
        return blocks

    mask_black_close = crop(pure_black_mask, cy_slice, ch, cx, cw)
    if cw > 0 and ch > 0:
        arena_close = arena_mask[cy:cy + ch, cx:cx + cw]
        mask_black_close = cv2.bitwise_and(mask_black_close, arena_close)

    all_detected_blocks = []
    all_detected_blocks.extend(process_block_contours(mask_red_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'red', BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'green', BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_red_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'red', CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'green', CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_magenta_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'magenta', CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_black_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'black', CLOSE_BLOCK_MIN_AREA))

    main_blocks = [b for b in all_detected_blocks if b['type'] == 'block']
    other_blocks = [b for b in all_detected_blocks if b['type'] != 'block']
    main_blocks.sort(key=lambda b: b['centroid'][1], reverse=True)
    processed_data['detected_blocks'] = main_blocks + other_blocks

    if cv2.countNonZero(mask_orange_line) > 0:
        contours, _ = cv2.findContours(mask_orange_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > 20:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    ocx = int(M["m10"] / M["m00"]) + lx
                    ocy = int(M["m01"] / M["m00"]) + ly
                    biggest_contour_global = biggest_contour + [lx, ly]
                    processed_data['detected_orange'].append({'type': 'orange_block', 'color': 'orange', 'area': area, 'centroid': (ocx, ocy), 'contour': biggest_contour_global})

    if cv2.countNonZero(mask_blue_line) > 0:
        contours, _ = cv2.findContours(mask_blue_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > 20:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    bcx = int(M["m10"] / M["m00"]) + lx
                    bcy = int(M["m01"] / M["m00"]) + ly
                    biggest_contour_global = biggest_contour + [lx, ly]
                    processed_data['detected_blue'].append({'type': 'blue_block', 'color': 'blue', 'area': area, 'centroid': (bcx, bcy), 'contour': biggest_contour_global})

    if cv2.countNonZero(final_mask_close_black) > 0:
        contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > WALL_MIN_AREA:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    kcx = int(M["m10"] / M["m00"])
                    kcy = int(M["m01"] / M["m00"]) + GLOBAL_Y_OFFSET
                    contour_global = contour + [0, GLOBAL_Y_OFFSET]
                    processed_data['detected_close_black'].append({
                        'type': 'close_black',
                        'color': 'black',
                        'area': area,
                        'centroid': (kcx, kcy),
                        'contour': contour_global
                    })

    wall_contours_by_roi = {job['type']: [] for job in [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]}
    if cv2.countNonZero(final_mask_walls) > 0:
        contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > WALL_MIN_AREA:
                M = cv2.moments(c)
                if M["m00"] == 0: continue
                wcx = int(M["m10"] / M["m00"])

                job_type = 'unknown'
                if left_side_job['roi'][0] <= wcx < left_side_job['roi'][0] + left_side_job['roi'][2]: job_type = left_side_job['type']
                elif right_side_job['roi'][0] <= wcx < right_side_job['roi'][0] + right_side_job['roi'][2]: job_type = right_side_job['type']
                elif inner_left_side_job['roi'][0] <= wcx < inner_left_side_job['roi'][0] + inner_left_side_job['roi'][2]: job_type = inner_left_side_job['type']
                elif inner_right_side_job['roi'][0] <= wcx < inner_right_side_job['roi'][0] + inner_right_side_job['roi'][2]: job_type = inner_right_side_job['type']

                if job_type != 'unknown':
                    wall_contours_by_roi[job_type].append(c)

    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                wcx = int(M["m10"] / M["m00"])
                wcy = int(M["m01"] / M["m00"]) + GLOBAL_Y_OFFSET
                biggest_contour_global = biggest_contour + [0, GLOBAL_Y_OFFSET]
                processed_data['detected_walls'].append({'type': job_type, 'color': 'black', 'area': area, 'centroid': (wcx, wcy), 'contour': biggest_contour_global})

    # Wall detection in line ROI (detecting black wall to prevent crash)
    wall_line_crop = pure_black_mask[ly_slice:ly_slice+lh, lx:lx+lw]
    wall_pixels = cv2.countNonZero(wall_line_crop)
    total_roi_pixels = lw * lh
    line_roi_wall_pct = (wall_pixels / total_roi_pixels) * 100.0 if total_roi_pixels > 0 else 0.0

    processed_data['line_roi_wall_pct'] = line_roi_wall_pct
    processed_data['detected_line_roi_wall'] = []

    if wall_pixels > 0:
        contours, _ = cv2.findContours(wall_line_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_global = contour + [lx, ly]
            processed_data['detected_line_roi_wall'].append({
                'contour': contour_global
            })

    return processed_data


# ---------------------------------------------------------------------------
# Detection serialisation (for the offline annotator)
# ---------------------------------------------------------------------------
# Under `main.py -v` the run stores untouched frames and the annotated video is
# rebuilt later. Re-running process_video_frame() over the recording ALMOST works --
# it is stateless per frame -- but not quite, because H.264 is lossy and inRange() is
# a hard threshold: a pixel sitting on an HSV boundary flips on a change of 1, and the
# contour edge moves with it. Measured over 200 frames, a lossless (PNG) round trip
# reproduces detections 100% of the time while H.264 manages 23.5%, and JPEG at
# quality 100 is no better than H.264. Only *exactly* lossless helps, and lossless
# recording is not affordable here (uncompressed is 19.4 MB/s against an SD card that
# sustains 23.1 MB/s; the lossless codecs cost 11-17 ms/frame, worse than the x264
# encode whose removal is what got the loop back to 56 fps).
#
# So the detections are logged rather than re-derived. The rebuilt video is then exact
# by construction, whatever the video codec does to the pixels, and the recording can
# be compressed harder rather than less.
#
# Contours dominate the size (median 245 points/frame), so they go out as base64 int16
# rather than JSON integer lists: 0.04 ms/frame instead of 0.14, and coordinates are
# 0..640 so int16 is lossless here. Median 3.2 KB/frame, 0.09 MB/s at 28 fps.

DETECTION_LISTS = (
    'detected_blocks', 'detected_walls', 'detected_orange', 'detected_blue',
    'detected_magenta', 'detected_close_black', 'detected_line_roi_wall',
)
# Per-object scalars worth keeping. Absent keys are skipped rather than written null:
# detected_line_roi_wall entries carry only a contour, and magenta alone has target_x.
_OBJ_SCALARS = ('color', 'area', 'centroid', 'type', 'target_x')


def _enc_pts(arr):
    return base64.b64encode(np.ascontiguousarray(arr, dtype=np.int16)).decode('ascii')


def _dec_pts(s):
    return np.frombuffer(base64.b64decode(s), dtype=np.int16).reshape(-1, 1, 2).astype(np.int32)


def detections_to_record(detections):
    """Everything annotate_video_frame() reads, as a JSON-serialisable dict.

    Called on the control loop, so it stays cheap: no findContours beyond the one the
    arena overlay needs, no copies of the full arena mask (which is 230 KB/frame and
    is only ever used to derive that outline).
    """
    rec = {}
    for key in DETECTION_LISTS:
        objs = []
        for o in detections.get(key, []):
            e = {'c': _enc_pts(o['contour'])}
            for s in _OBJ_SCALARS:
                if s in o:
                    v = o[s]
                    e[s] = [int(v[0]), int(v[1])] if s == 'centroid' else (
                        float(v) if s == 'area' else (int(v) if s == 'target_x' else v))
            objs.append(e)
        rec[key] = objs

    arena_mask = detections.get('arena_mask')
    if arena_mask is not None:
        # Store the outline, not the mask. This is the same call the annotator would
        # have made, just made once here instead -- 0.09 ms.
        band = arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :]
        cnts, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rec['arena_contours'] = [_enc_pts(c) for c in cnts]
    sky = detections.get('arena_sky')
    if sky is not None:
        rec['arena_sky'] = _enc_pts(sky)
    rec['arena_seeded'] = bool(detections.get('arena_seeded', False))
    rec['line_roi_wall_pct'] = float(detections.get('line_roi_wall_pct', 0))
    return rec


def record_to_detections(rec):
    """Inverse of detections_to_record(). Returns a dict annotate_video_frame() accepts.

    'arena_mask' is deliberately absent -- the outline it was only ever used for is
    carried directly as 'arena_contours', which annotate_video_frame() prefers.
    """
    out = {}
    for key in DETECTION_LISTS:
        objs = []
        for e in rec.get(key, []):
            o = {'contour': _dec_pts(e['c'])}
            for s in _OBJ_SCALARS:
                if s in e:
                    o[s] = tuple(e[s]) if s == 'centroid' else e[s]
            objs.append(o)
        out[key] = objs
    if 'arena_contours' in rec:
        out['arena_contours'] = [_dec_pts(c) for c in rec['arena_contours']]
    if rec.get('arena_sky') is not None:
        out['arena_sky'] = np.frombuffer(base64.b64decode(rec['arena_sky']), dtype=np.int16)
    out['arena_seeded'] = rec.get('arena_seeded', False)
    out['line_roi_wall_pct'] = rec.get('line_roi_wall_pct', 0)
    return out


def annotate_video_frame(frame, detections, driving_direction, debug_info="", visual_target_x=None, visual_target_line=None):
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)
    target_line_color = (255, 0, 255)

    all_rois = [
        (left_roi_x, left_roi_y, left_roi_w, left_roi_h),
        (right_roi_x, right_roi_y, right_roi_w, right_roi_h),
        (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h),
        (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h),
        (line_roi_x, line_roi_y, line_roi_w, line_roi_h),
        (close_x, close_y, close_w, close_h),
    ]
    if full_frame_roi and full_frame_roi[2] > 0 and full_frame_roi[3] > 0:
        all_rois.append(full_frame_roi)
    if close_block_roi and close_block_roi[2] > 0 and close_block_roi[3] > 0:
        all_rois.append(close_block_roi)
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    # --- Arena mask overlay (v5) ---
    # Live, the mask is on hand and the outline is derived here. Replayed from a
    # detection record there is no mask, only the outline it produced -- the mask is
    # 230 KB/frame and this is the sole thing it was used for.
    arena_mask = detections.get('arena_mask')
    arena_cnts_pre = detections.get('arena_contours')
    if (arena_mask is not None or arena_cnts_pre is not None) and USE_ARENA_MASK:
        # band limits + chassis rect, as static reference
        cv2.line(annotated_frame, (0, ARENA_Y_TOP), (FRAME_WIDTH, ARENA_Y_TOP), (90, 90, 90), 1)
        cv2.line(annotated_frame, (0, ARENA_Y_BOTTOM), (FRAME_WIDTH, ARENA_Y_BOTTOM), (90, 90, 90), 1)
        cv2.rectangle(annotated_frame, (180, 250), (460, 359), (90, 90, 90), 1)

        # accepted region outline
        if arena_cnts_pre is not None:
            arena_cnts = arena_cnts_pre
        else:
            band = arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :]
            arena_cnts, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in arena_cnts:
            cv2.drawContours(annotated_frame, [c + [0, ARENA_Y_TOP]], -1, (0, 255, 255), 1)

        # skyline: the per-column top boundary. Tune ARENA_TOP_MARGIN against this.
        arena_sky = detections.get('arena_sky')
        if arena_sky is not None:
            pts = np.stack([np.arange(FRAME_WIDTH, dtype=np.int32),
                            np.asarray(arena_sky).astype(np.int32) + ARENA_Y_TOP], axis=1)
            cv2.polylines(annotated_frame, [pts], False, (0, 200, 255), 1)

        # seed point -- green if it landed on a floor blob, red if we fell back to
        # pass-through. First thing to check when the mask misbehaves.
        seeded = detections.get('arena_seeded', False)
        cv2.circle(annotated_frame, ARENA_SEED_PT, 4, (0, 255, 0) if seeded else (0, 0, 255), -1)

    if detections.get('line_roi_wall_pct', 0) > 50:
        for black_obj in detections.get('detected_line_roi_wall', []):
            cv2.drawContours(annotated_frame, [black_obj['contour']], -1, (0, 0, 0), 2)

    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (0, 0, 0), 2)

    for block in detections['detected_blocks']:
        draw_color = (255, 255, 255)
        if block['color'] == 'red':
            draw_color = (0, 0, 255)
        elif block['color'] == 'green':
            draw_color = (0, 255, 0)
        elif block['color'] == 'magenta':
            draw_color = (255, 0, 255)
        elif block['color'] == 'black':
            draw_color = (0, 0, 0)
        cv2.drawContours(annotated_frame, [block['contour']], -1, draw_color, 2)

    for orange_obj in detections['detected_orange']:
        cv2.drawContours(annotated_frame, [orange_obj['contour']], -1, (0, 165, 255), 2)

    for blue_obj in detections['detected_blue']:
        cv2.drawContours(annotated_frame, [blue_obj['contour']], -1, (255, 0, 0), 2)

    for black_obj in detections.get('detected_close_black', []):
        cv2.drawContours(annotated_frame, [black_obj['contour']], -1, (0, 0, 0), 2)

    for magenta_obj in detections['detected_magenta']:
        cv2.drawContours(annotated_frame, [magenta_obj['contour']], -1, (255, 0, 255), 2)
        target_x = magenta_obj['target_x']
        mcy = magenta_obj['centroid'][1]
        cv2.circle(annotated_frame, (target_x, mcy), 7, (255, 255, 255), -1)

    if visual_target_x is not None:
        cv2.line(annotated_frame, (visual_target_x, 0), (visual_target_x, FRAME_HEIGHT), target_line_color, 2)

    if visual_target_line is not None:
        # Draw the actual line pointing towards the block (Cyan)
        pt1, pt2, ideal_angle = visual_target_line[:3]
        cv2.line(annotated_frame, pt1, pt2, (0, 255, 255), 2)

        # Calculate and draw the "ideal" target line (Yellow)
        if len(visual_target_line) == 4:
            pt3 = visual_target_line[3]
            cv2.line(annotated_frame, pt1, pt3, (0, 255, 255), 3)
        else:
            origin_x, origin_y = pt1[0], pt1[1]
            target_len = 200
            target_pt_x = int(origin_x + target_len * math.sin(math.radians(ideal_angle)))
            target_pt_y = int(origin_y - target_len * math.cos(math.radians(ideal_angle)))
            cv2.line(annotated_frame, (origin_x, origin_y), (target_pt_x, target_pt_y), (0, 255, 255), 3)

    if debug_info:
        if isinstance(debug_info, (list, tuple)):
            items = [str(x) for x in debug_info]
        elif isinstance(debug_info, str):
            s = debug_info.strip()
            if s.startswith('[') and s.endswith(']'):
                try:
                    parsed = ast.literal_eval(s)
                    if isinstance(parsed, (list, tuple)):
                        items = [str(x) for x in parsed]
                    else:
                        items = [s]
                except Exception:
                    items = [s]
            elif '\n' in s:
                items = s.split('\n')
            else:
                items = [s]
        else:
            items = [str(debug_info)]

        lines = []
        curr_line = []
        curr_len = 0
        max_chars_per_line = 48
        for item in items:
            item_len = len(item) + (3 if curr_line else 0)
            if curr_line and (curr_len + item_len > max_chars_per_line):
                lines.append(" | ".join(curr_line))
                curr_line = [item]
                curr_len = len(item)
            else:
                curr_line.append(item)
                curr_len += item_len
        if curr_line:
            lines.append(" | ".join(curr_line))

        y_offset = 25
        for line in lines:
            # Draw black outline for high contrast readability
            cv2.putText(annotated_frame, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            # Draw white text
            cv2.putText(annotated_frame, line, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y_offset += 20

    return annotated_frame
