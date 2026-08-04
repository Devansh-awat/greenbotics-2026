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
# the largest connected non-black blob that CONTAINS the patch of mat directly in
# front of the robot, then extend it upward through the black wall standing on it,
# stopping at the wall's top edge. The far section is a separate blob (the wall
# breaks connectivity) so it is structurally unreachable, not merely thresholded out.
#
# Unlike a brightness-threshold pipeline, pillars are automatically part of the floor
# blob here -- they aren't black, so they pass NOT(black) and stay continuous with the
# mat. No colour repainting stage is needed. Magenta counts as floor too, so the
# parking walls survive the gate.
USE_ARENA_MASK = True        # False -> byte-identical behaviour to main_v2
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

USE_GROUND_CONTACT = True    # reject blocks that aren't standing on the mat
GROUND_PROBE_DY = 12         # probe strip this far below a block's bounding box
GROUND_CONTACT_MIN = 0.5     # >= this fraction of the strip must be floor

ARENA_BAND_H = ARENA_Y_BOTTOM - ARENA_Y_TOP
ARENA_SEED_LOCAL = (float(ARENA_SEED_PT[0]), float(ARENA_SEED_PT[1] - ARENA_Y_TOP))
_ARENA_ROWS = np.arange(ARENA_BAND_H, dtype=np.int32)[:, None]
_ARENA_WALL_KERNEL = np.ones((MIN_WALL_THICK, 1), np.uint8)   # vertical, for the wall scan
_ARENA_WALL_CLOSE_KERNEL = np.ones((WALL_GAP_SEAL, 1), np.uint8)
ARENA_PASSTHROUGH = np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype="uint8")


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

    # 1. Floor candidate. Black walls are the only thing excluded -- pillars, floor
    #    lines and magenta parking walls all pass and stay part of the blob.
    mask_black = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)
    floor_cand = cv2.bitwise_not(mask_black)
    # Closing seals seams/glare/cable shadows in the mat. Careful raising this: a
    # kernel wide enough to bridge the arena wall merges the far section into our
    # blob and silently defeats the whole mechanism.
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
    fill = int(np.median(wall_top[trusted])) if trusted.any() else 0
    sky = np.where(trusted, wall_top, fill).astype(np.int32)
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
        if cv2.countNonZero(mask) > 0:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                biggest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(biggest_contour)
                if area > min_area:
                    M = cv2.moments(biggest_contour)
                    if M["m00"] != 0:
                        bcx = int(M["m10"] / M["m00"]) + offset_x
                        bcy = int(M["m01"] / M["m00"]) + offset_y
                        biggest_contour_global = biggest_contour + [offset_x, offset_y]

                        # Ground-contact test. The arena mask is a SOLID fill that
                        # includes the wall band, so a red logo on the wall or a
                        # pillar in the far section poking into that band still
                        # passes it. This asks the orthogonal question: is the thing
                        # standing on the mat? Real pillars have floor beneath them;
                        # anything resting on a wall has black beneath it.
                        # Skipped for 'close_block' -- that 10px strip is an
                        # emergency reverse-and-swerve reflex where a false negative
                        # is worse than a false positive.
                        if USE_GROUND_CONTACT and floor_mask is not None and b_type == 'block':
                            rx, ry, rw, rh = cv2.boundingRect(biggest_contour_global)
                            probe_y = ry + rh + GROUND_PROBE_DY
                            if probe_y < ARENA_Y_BOTTOM:
                                strip = floor_mask[probe_y, rx:rx + rw]
                                if strip.size and (np.count_nonzero(strip) / strip.size) < GROUND_CONTACT_MIN:
                                    return None

                        return {'type': b_type, 'color': b_color, 'area': area,
                                'centroid': (bcx, bcy), 'contour': biggest_contour_global}
        return None

    all_detected_blocks = []

    res = process_block_contours(mask_red_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'red', BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    res = process_block_contours(mask_green_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'green', BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    res = process_block_contours(mask_red_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'red', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    res = process_block_contours(mask_green_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'green', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    res = process_block_contours(mask_magenta_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'magenta', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

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
