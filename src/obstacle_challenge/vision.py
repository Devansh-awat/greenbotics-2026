"""Frame processing: arena mask, colour thresholding, detection and annotation.

`process_video_frame()` is the entry point -- it turns one BGR frame into the
detections dict the control loop steers on. When the vision pool is running, the
arena mask and the colour masks are computed concurrently in two worker processes
and joined here (see vision_pool.py); the results are identical either way.

Nothing in this module touches hardware, so it is safe to import and exercise on
saved frames.
"""

import math
import cv2
import numpy as np

from src.obstacle_challenge.logsetup import vlog
from src.obstacle_challenge.tuning import *

# The live VisionPool, or None to process inline. Set by main_v5 at startup;
# vision_pool.py imports this module, so it must not be imported from here.
vision_pool = None
# --- Arena mask -------------------------------------------------------------
# v5 only. The fixed ROIs above say *where in the frame* to look; they can't say
# *whether the thing we found is inside the arena*. full_frame_roi spans the whole
# width, so any red/green blob in rows 80-250 becomes a pillar -- including blobs in
# the far track section seen over the inner wall, and anything outside the arena.
#
# The arena mask fixes that. It is rebuilt every frame from the image itself: take
# the largest connected FLOOR-COLOURED blob that CONTAINS the patch of mat directly in
# front of the robot, then, per column, walk UP from that mat and stop at the first
# break. Everything above the break -- the wall, the far track section over it, the
# hall, shoes -- is outside by construction.
#
# Floor-coloured means white mat plus the block, parking-wall and line colours, so
# pillars are part of the blob rather than holes in it, and magenta parking walls
# survive the gate. It does NOT mean NOT(black), which is what this used to be: a grey
# hall floor is not black either, so it joined the blob wherever the wall thinned.
#
# Two things this deliberately does not do, both tried and measured over 3977 frames
# with src/tools/compare_skyline.py:
#   - Stop at the black wall's TOP edge. That edge is only in the mask when the
#     background behind the wall is bright; on a grey hall floor there is nothing to
#     find, and 70% of columns ended up ungated.
#   - Take the topmost pixel of the blob. Membership of the blob is a fact about the
#     whole frame, not about one column; one thin spot in the wall joins the hall to
#     the mat and every column on that side then has blob pixels at row 0.
USE_ARENA_MASK = True        # False -> byte-identical behaviour to main_v2
# 'skyline' : per column, walk up from the mat and gate everything above the first
#             break. Excludes the far track section seen over the INNER wall along
#             with everything beyond the outer wall -- from one column both look the
#             same: floor, then a wall, then more floor.
# 'filldown': per column, top -> down, first floor pixel wins, fill to the bottom.
#             Keeps the far track section over an inner wall. Loses to a leak.
# 'contour' : gate to the seeded floor contour itself. Keeps the far track section
#             (it is reachable floor) at the cost of keeping genuine outside leaks
#             too, whenever the outer wall thins enough to join the hall to the mat.
# annotate_video_frame() draws the mask outline AND the skyline, so whichever is not
# in force stays visible for comparison.
ARENA_MASK_MODE = 'walltop'
ARENA_Y_TOP = 50             # hard clamp: nothing above this row is ever arena
ARENA_Y_BOTTOM = 250         # below this the chassis is in frame (chassis: x180 y250 w280 h110)
ARENA_SEED_PT = (320, 230)   # patch of mat directly in front of the robot, frame coords
ARENA_TOP_MARGIN = 0         # TUNING KNOB: push the skyline DOWN N px to trim the wall band
MAX_WALL_RUN = 160           # run longer than this = column untrusted (see build_arena_mask).
                             # Measured on real frames: wall runs reach p99=90, max=155 px
                             # when the robot is close to a wall. 70 clipped 6% of columns.
MIN_WALL_THICK = 8           # a black run must be >= this tall to count as a wall
WALL_GAP_SEAL = 3            # seal bright breaks up to this tall inside a wall, so the
                             # upward scan isn't stopped early by glare/noise/artifacts.
                             # Do NOT raise much: sealing also lets the run climb past
                             # the wall into dark background. Measured over 40 frames --
                             # seal 3: 0.44 px/col jagged, 1/40 frames collapse to the
                             # full band; seal 9: 0.41 px/col but 7/40 collapse.
ARENA_CLOSE_KERNEL = np.ones((9, 9), np.uint8)
ARENA_SKY_SMOOTH = 31        # sliding-MEDIAN window over sky[x]; <=1 disables

# Wall following: keep only black within this many px ABOVE the floor boundary; 0 is
# off. Contiguity with the mat is not enough on its own -- if the surround thresholds
# as black too (a grey hall floor: measured S 87 V 53 against UPPER_BLACK's S<=110
# V<=85) it sits directly on the wall with no gap and the run walks straight through
# it. A height cap is the second condition that catches that.
#
# Sizing it, measured over 89657 column-samples of 900 frames in the CURRENT venue:
# the attached run is p50 87 px, p90 125, p99 160 where the wall is close. So the cap
# has to clear ~160 or it eats real wall -- at 50 the wall-ROI area drops to 59-74% of
# today's, and the PD gains are tuned against today's. At 120 it is 97-100%, i.e. a
# no-op here. Only 0.5% of runs with real headroom escape the band in this venue, so
# this is insurance for a grey one, not a fix for a problem visible today.
#
# The cap has to SCALE WITH DISTANCE, or it is useless where it matters. A wall's
# apparent height is set by how close its base is -- measured over 52992 clean runs:
#     base at row  40 (far)   p50  42   p99  50
#     base at row 120         p50  72   p99 103
#     base at row 190 (near)  p50  92   p99 163
# A flat cap must clear 163 to keep the near wall, which then allows 163 px in a far
# column whose real wall is 50 -- exactly the columns where a merged background runs
# away. So the cap is a line in the boundary row: cap = SLOPE * sky + WALL_BAND_PX.
# SLOPE 0.9 / PX 25 clips 0.0% of real wall here while capping far columns at 61 px
# instead of 160. SLOPE 0 degrades to a flat cap.
#
# To size it on site:  python3 -m src.tools.compare_skyline --wall-height
WALL_BAND_PX = 0
WALL_BAND_SLOPE = 0.9

USE_GROUND_CONTACT = True    # reject blocks that aren't standing on the mat
GROUND_PROBE_DY = 12         # probe strip this far below a block's bounding box
GROUND_CONTACT_MIN = 0.5     # >= this fraction of the strip must be floor

ARENA_BAND_H = ARENA_Y_BOTTOM - ARENA_Y_TOP
ARENA_SEED_LOCAL = (float(ARENA_SEED_PT[0]), float(ARENA_SEED_PT[1] - ARENA_Y_TOP))
_ARENA_ROWS = np.arange(ARENA_BAND_H, dtype=np.int32)[:, None]
_ARENA_WALL_KERNEL = np.ones((MIN_WALL_THICK, 1), np.uint8)   # vertical, for the wall scan
_ARENA_WALL_CLOSE_KERNEL = np.ones((WALL_GAP_SEAL, 1), np.uint8)
ARENA_PASSTHROUGH = np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype="uint8")

# Everything that is legitimately part of the drivable arena, for the floor blob in
# build_arena_mask(). White is the mat; the block/line colours are things standing ON
# it and must be included, or every pillar punches a hole in the blob and stops the
# upward walk on its own base. RED_2 is HSV's hue wrap-around.
_FLOOR_RANGES = ((LOWER_WHITE, UPPER_WHITE), (LOWER_RED_1, UPPER_RED_1),
                 (LOWER_RED_2, UPPER_RED_2), (LOWER_GREEN, UPPER_GREEN),
                 (LOWER_MAGENTA, UPPER_MAGENTA), (LOWER_ORANGE, UPPER_ORANGE),
                 (LOWER_BLUE, UPPER_BLUE))


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


def _fill_and_smooth_sky(floor):
    """Per-column topmost floor row, with the same unmeasured-column fill and sliding
    median the walk-based boundary gets. Used by ARENA_MASK_MODE == 'filldown'."""
    has = floor.any(axis=0)
    top = np.argmax(floor, axis=0).astype(np.int32)
    if not has.any():
        return np.zeros(FRAME_WIDTH, dtype=np.int32)
    cols = np.arange(FRAME_WIDTH)
    out = np.interp(cols, cols[has], top[has]).astype(np.int32)
    return np.clip(_sliding_median(out + ARENA_TOP_MARGIN, ARENA_SKY_SMOOTH), 0, ARENA_BAND_H)


def build_arena_mask(frame):
    """
    Returns (arena_mask, floor_mask, sky), or (None, None, None) if the seed point
    isn't on any floor blob (camera covered, nose into a wall) -- caller falls back
    to pass-through so a bad frame degrades to main_v2 behaviour instead of blanking
    every detection.

    arena_mask : full-frame uint8. Inside the band it is the solid skyline region.
                 ABOVE the band it is 0 -- ARENA_Y_TOP is a hard clamp, nothing up
                 there is ever arena. BELOW the band it is 255 (pass-through) so the
                 wall ROIs, which run to y=280, are never clipped.
    floor_mask : full-frame uint8, the drivable mat with interior holes PRESERVED.
                 Used only by the ground-contact test -- the solid arena fill would
                 make that test pass trivially.
    sky        : per-column top boundary, band-local. Annotation only.
    """
    band = frame[ARENA_Y_TOP:ARENA_Y_BOTTOM, :]
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)

    # 1. Floor candidate, from POSITIVE colour evidence -- the white mat plus
    #    everything that legitimately stands on it (see _FLOOR_RANGES).
    #
    #    Not NOT(black), which is what this used to be. A grey hall floor is not black,
    #    so it passes NOT(black) and joins the blob wherever the wall thins; it does
    #    not pass a white threshold, because white-vs-grey is a V discrimination and
    #    not the hue call that a grey surround makes unreliable.
    mask_black = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)
    if ARENA_MASK_MODE == 'walltop':
        # ORIGINAL. Black walls are the only thing excluded -- pillars, floor lines
        # and magenta parking walls all pass and stay part of the blob.
        floor_cand = cv2.bitwise_not(mask_black)
    else:
        floor_cand = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
        for lo, hi in _FLOOR_RANGES[1:]:
            cv2.bitwise_or(floor_cand, cv2.inRange(hsv, lo, hi), dst=floor_cand)
    # Closing seals seams/glare/cable shadows in the mat. Careful raising this: a
    # kernel wide enough to bridge the arena wall merges the far section into our blob.
    floor_cand = cv2.morphologyEx(floor_cand, cv2.MORPH_CLOSE, ARENA_CLOSE_KERNEL)

    # 2. Keep only the blob reachable from the robot's nose, then take the largest.
    contours, _ = cv2.findContours(floor_cand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    seeded = [c for c in contours if cv2.pointPolygonTest(c, ARENA_SEED_LOCAL, False) >= 0]
    if not seeded:
        return None, None, None
    cnt = max(seeded, key=cv2.contourArea)

    filled = np.zeros_like(floor_cand)
    cv2.drawContours(filled, [cnt], -1, 255, -1)
    floor = cv2.bitwise_and(floor_cand, filled)   # fill-then-AND: interior holes come back

    # 3. The top boundary. Which of these runs is ARENA_MASK_MODE's whole job.
    if ARENA_MASK_MODE == 'walltop':
        # ORIGINAL: per-column wall scan, cutting at the TOP of the near wall's run.
        #
        # Deriving the top boundary from the floor blob's upper edge does not work.
        # The room beyond the arena is bright, so it passes NOT(black) too, and the
        # wall is a thin barrier -- wherever it thins out or leaves frame, the blob
        # escapes into the room and floor_top collapses to 0 for that column.
        #
        # The vertical MORPH_OPEN keeps only black runs >= MIN_WALL_THICK px tall, so
        # speckle, floor-line shadows and pillar edges can't be mistaken for a wall.
        # Seal small BRIGHT breaks inside the wall first -- the upward walk needs a
        # strictly contiguous black run, so one glare pixel stops it dead.
        wall_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, _ARENA_WALL_CLOSE_KERNEL)
        thick = cv2.morphologyEx(wall_black, cv2.MORPH_OPEN, _ARENA_WALL_KERNEL)
        thick_b = thick > 0
        has = thick_b.any(axis=0)

        lowest = (ARENA_BAND_H - 1) - np.argmax(thick_b[::-1], axis=0)
        chain = thick_b | (_ARENA_ROWS > lowest)     # force everything below `lowest` True
        run = np.minimum.accumulate(chain[::-1], axis=0)[::-1]
        wall_top = ARENA_BAND_H - run.sum(axis=0)

        # A run longer than MAX_WALL_RUN means the wall has probably merged into a dark
        # background above it, so we can't locate its top edge in this column. Do NOT
        # clamp to `lowest - MAX_WALL_RUN`: that lands the boundary in the MIDDLE of the
        # wall, which neither includes it nor excludes what is beyond it. Mark the
        # column untrusted instead and let neighbours supply the value.
        trusted = has & ((lowest - wall_top) <= MAX_WALL_RUN)
        fill = int(np.median(wall_top[trusted])) if trusted.any() else 0
        sky = np.where(trusted, wall_top, fill).astype(np.int32)
        sky = sky + ARENA_TOP_MARGIN
        sky = _sliding_median(sky, ARENA_SKY_SMOOTH)
        sky = np.clip(sky, 0, ARENA_BAND_H)
    else:
        floor_b = floor > 0
        if ARENA_MASK_MODE == 'filldown':
            # Top -> down, first floor pixel wins. Keeps the far track section seen
            # over an INNER wall, at the cost of following a leak when the outer wall
            # thins enough to join the hall to the mat.
            measured = floor_b.any(axis=0)
            top = np.argmax(floor_b, axis=0).astype(np.int32)
            open_col = np.zeros(FRAME_WIDTH, dtype=bool)
        else:
            # 'skyline' / 'contour': walk UP from the mat, stop at the first break.
            #
            # NOT the topmost pixel of the blob: being in the blob is a fact about the
            # whole frame, not about this column. The wall only has to fail in ONE
            # column for the far side to join the mat, and then every column that side
            # has blob pixels near row 0 (2026-08-24_17-01-06 f285: the wall tapers out
            # at x=270 and a 7px strip of hall floor joins the contour for the 269
            # columns to its left). A walk cannot inherit reachability from elsewhere.
            # The interior holes restored in step 2 are what gives it something to stop
            # at -- the filled contour has no wall in it.
            lowest = (ARENA_BAND_H - 1) - np.argmax(floor_b[::-1], axis=0)
            # The walk must START on our own floor. Where the wall fills the column the
            # only blob left is whatever leaked in beyond it, and walking up from THAT
            # reports the room as arena. The mat under the nose is always at or below
            # the seed row.
            measured = floor_b.any(axis=0) & (lowest >= ARENA_SEED_LOCAL[1])
            chain = floor_b | (_ARENA_ROWS > lowest)
            top = ARENA_BAND_H - np.minimum.accumulate(chain[::-1], axis=0)[::-1].sum(axis=0)
            # top == 0 means floor runs unbroken to the top of the band: the wall is
            # above ARENA_Y_TOP here, so the whole column IS arena. A measurement, not
            # a gap -- pin it open and keep it out of the fill, or a neighbour that did
            # see a wall drags a boundary across visible floor.
            open_col = measured & (top == 0)
            measured = measured & ~open_col

        if not measured.any():
            if not open_col.any():
                return None, None, None
            sky = np.zeros(FRAME_WIDTH, dtype=np.int32)
        else:
            # Interpolate unmeasured columns from their neighbours rather than from the
            # median of ALL trusted columns: that borrowed a number with no relation to
            # the column it landed in. np.interp edge-clamps at the frame border.
            cols = np.arange(FRAME_WIDTH)
            sky = np.interp(cols, cols[measured], top[measured]).astype(np.int32)
            sky[open_col] = 0
        sky = _sliding_median(sky + ARENA_TOP_MARGIN, ARENA_SKY_SMOOTH)
        sky = np.clip(sky, 0, ARENA_BAND_H)

    if ARENA_MASK_MODE == 'contour':
        # The solid contour, NOT the holes-restored blob. The holes are the walls, and
        # punching them out of the gate would also gate out the near wall -- which is
        # where close_block_roi looks for black obstacles. Filling costs nothing here
        # (`filled` is already built) and matches what the skyline mode admits.
        band_mask = filled
    elif ARENA_MASK_MODE == 'filldown':
        # Per column, top -> down, first floor pixel wins, fill from there to the
        # bottom. Unlike 'skyline' this keeps the far track section seen over an INNER
        # wall (it is floor, and it is the topmost floor in that column), and unlike
        # 'contour' it follows the column rather than a polygon, so a concavity that
        # opens to the frame edge does not swallow what is beside it.
        #
        # The trade it makes: a column's topmost floor pixel says nothing about HOW it
        # is reachable. Where the outer wall thins enough to join the hall to the mat,
        # the hall becomes the topmost floor for every column on that side. That is the
        # 2026-08-24_17-11-37 f450 case. 'skyline' is the mode that survives it.
        band_mask = ((_ARENA_ROWS >= _fill_and_smooth_sky(floor)).astype(np.uint8)) * 255
    else:
        band_mask = ((_ARENA_ROWS >= sky).astype(np.uint8)) * 255

    arena_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
    arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :] = band_mask
    arena_mask[ARENA_Y_BOTTOM:, :] = 255   # below the band: pass-through, never clip

    # Bound the floor blob by the arena region too -- if it did leak into the room,
    # those pixels must not be able to validate a block in the ground-contact test.
    floor_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
    floor_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :] = cv2.bitwise_and(floor, band_mask)

    return arena_mask, floor_mask, sky

# ---------------------------------------------------------------------------
# Colour thresholding (the half of the pipeline that does NOT need the arena mask)
# ---------------------------------------------------------------------------

MASK_NAMES = ('red', 'green', 'magenta', 'orange', 'blue', 'black')

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


def compute_colour_masks(frame, out=None):
    """Threshold the working slice into the six colour masks.

    Every operation here is pointwise, so thresholding the whole slice and cropping
    afterwards is identical to the old code's crop-then-threshold -- which is what
    lets this run in a worker process independently of the arena mask.

    `out`, if given, is a (6, SLICE_HEIGHT, FRAME_WIDTH) uint8 view into shared
    memory to write into directly, avoiding an allocation + copy per frame.
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

    if USE_LAB:
        hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2Lab)
    else:
        hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2HSV)

    red = cv2.inRange(hsv_slice, LOWER_RED_1, UPPER_RED_1)
    if not USE_LAB:
        # In HSV, red wraps around 180->0, so it needs two ranges combined.
        # In LAB red is continuous and the first range is the whole story.
        cv2.bitwise_or(red, cv2.inRange(hsv_slice, LOWER_RED_2, UPPER_RED_2), dst=red)

    if out is None:
        orange = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), np.uint8)
        blue = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), np.uint8)
    else:
        orange, blue = out[3], out[4]
        orange[:] = 0
        blue[:] = 0

    line_hsv = hsv_slice[LINE_SLICE_Y0:LINE_SLICE_Y1, LINE_X0:LINE_X1]
    orange[LINE_SLICE_Y0:LINE_SLICE_Y1, LINE_X0:LINE_X1] = cv2.inRange(
        line_hsv, LOWER_ORANGE, UPPER_ORANGE)
    blue[LINE_SLICE_Y0:LINE_SLICE_Y1, LINE_X0:LINE_X1] = cv2.inRange(
        line_hsv, LOWER_BLUE, UPPER_BLUE)

    masks = {
        'red': red,
        'green': cv2.inRange(hsv_slice, LOWER_GREEN, UPPER_GREEN),
        'magenta': cv2.inRange(hsv_slice, LOWER_MAGENTA, UPPER_MAGENTA),
        'orange': orange,
        'blue': blue,
        'black': cv2.inRange(hsv_slice, LOWER_BLACK, UPPER_BLACK),
    }
    if out is not None:
        # orange/blue were written in place above; copy the rest.
        for i, name in enumerate(MASK_NAMES):
            if name not in ('orange', 'blue'):
                out[i] = masks[name]
    return masks

def process_video_frame(frame):
    """Detect walls, pillars, floor lines and the arena boundary in one frame.

    Two halves run concurrently when the pool is up:
      - build_arena_mask (~3.3 ms)   -- worker 'vision-arena'
      - compute_colour_masks (~2.5 ms) -- worker 'vision-colour'
    They are independent until the bitwise_and below, so this is a fork-join, not a
    pipeline: the frame we act on is the frame we just captured, never the previous
    one. Contour extraction and all decisions stay in this process.
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
        masks = compute_colour_masks(frame)
        arena_mask = floor_mask = arena_sky = None
        if USE_ARENA_MASK:
            arena_mask, floor_mask, arena_sky = build_arena_mask(frame)

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
    arena_main = arena_mask[my:my + mh, mx:mx + mw]
    mask_red_main = cv2.bitwise_and(mask_red_main, arena_main)
    mask_green_main = cv2.bitwise_and(mask_green_main, arena_main)
    mask_magenta_main = cv2.bitwise_and(mask_magenta_main, arena_main)

    arena_line = arena_mask[ly:ly + lh, lx:lx + lw]
    mask_orange_line = cv2.bitwise_and(mask_orange_line, arena_line)
    mask_blue_line = cv2.bitwise_and(mask_blue_line, arena_line)

    arena_close = arena_mask[cy:cy + ch, cx:cx + cw]
    mask_red_close = cv2.bitwise_and(mask_red_close, arena_close)
    mask_green_close = cv2.bitwise_and(mask_green_close, arena_close)
    mask_magenta_close = cv2.bitwise_and(mask_magenta_close, arena_close)

    # --- 2. Reconstruct slice-sized global masks for wall/black detection ---
    global_red_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_green_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_blue_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_magenta_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")

    global_red_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_red_mask[my_slice:my_slice+mh, mx:mx+mw], mask_red_main)
    global_green_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_green_mask[my_slice:my_slice+mh, mx:mx+mw], mask_green_main)
    global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw], mask_magenta_main)

    global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_red_close)
    global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_green_close)
    global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_magenta_close)

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
    if WALL_BAND_PX and arena_sky is not None:
        # Rows more than WALL_BAND_PX above the floor boundary cannot be wall standing
        # on this mat, whatever they threshold as. Anchored on `arena_sky` rather than
        # on a contiguity walk: the sliding median can nudge sky a few px into the mat,
        # which stops a walk dead but is noise against a 100+ px band.
        cap = (WALL_BAND_SLOPE * arena_sky + WALL_BAND_PX).astype(np.int32)
        in_band = (_ARENA_ROWS >= arena_sky - cap).astype(np.uint8) * 255
        final_mask_walls = cv2.bitwise_and(final_mask_walls, in_band)
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

    all_detected_blocks = []
    all_detected_blocks.extend(process_block_contours(mask_red_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'red', BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'green', BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_red_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'red', CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'green', CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_magenta_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'magenta', CLOSE_BLOCK_MIN_AREA))

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
        full_frame_roi,
        close_block_roi
    ]
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    # --- Arena mask overlay (v5) ---
    arena_mask = detections.get('arena_mask')
    if arena_mask is not None and USE_ARENA_MASK:
        # band limits + chassis rect, as static reference
        cv2.line(annotated_frame, (0, ARENA_Y_TOP), (FRAME_WIDTH, ARENA_Y_TOP), (90, 90, 90), 1)
        cv2.line(annotated_frame, (0, ARENA_Y_BOTTOM), (FRAME_WIDTH, ARENA_Y_BOTTOM), (90, 90, 90), 1)
        cv2.rectangle(annotated_frame, (180, 250), (460, 359), (90, 90, 90), 1)

        # accepted region outline
        band = arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :]
        arena_cnts, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in arena_cnts:
            cv2.drawContours(annotated_frame, [c + [0, ARENA_Y_TOP]], -1, (0, 255, 255), 1)

        # skyline: the per-column top boundary. Tune ARENA_TOP_MARGIN against this.
        # Drawn in BOTH modes. In 'skyline' mode it is the mask's own edge; in
        # 'contour' mode it is the boundary the other mode would have used.
        arena_sky = detections.get('arena_sky')
        if arena_sky is not None:
            pts = np.stack([np.arange(FRAME_WIDTH, dtype=np.int32),
                            np.asarray(arena_sky).astype(np.int32) + ARENA_Y_TOP], axis=1)
            # Violet, 2px: deliberately unlike the yellow mask outline above, because
            # both are drawn every frame and in 'contour' mode they are different
            # shapes -- that difference is the whole point of having the two modes.
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

    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    return annotated_frame
