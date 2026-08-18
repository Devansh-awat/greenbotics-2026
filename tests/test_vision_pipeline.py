"""Regression tests for the 2026-08-17 vision-pipeline optimisations.

Two changes are pinned here, both of which MUST be output-identical:

  1. process_video_frame()'s step-2 "global mask" reconstruction now writes ROI
     crops straight into preallocated scratch planes instead of allocating four
     zeroed slices and bitwise_or-ing into them. _reference_process_video_frame()
     below is the pre-change function, copied verbatim (only renamed constants ->
     ``P.<name>`` so it tracks the live tuning); both versions run over real
     recorded frames and every detection field is compared exactly.

  2. CameraThread.get_frame()/get_next_frame() hand out the latest frame by
     reference instead of copying it. That is only safe while every consumer
     treats frames as read-only -- annotate_video_frame() must copy before
     drawing, which test_annotate_does_not_mutate_input() pins.

Run on the Pi (CPU only, no camera/GPIO/motors touched -- but do NOT run while a
robot run is live, it will steal CPU from the control loop):

    python3 -m pytest tests/test_vision_pipeline.py -v
  or, without pytest:
    python3 -m tests.test_vision_pipeline
"""

import glob
import os
import sys

import cv2
import numpy as np

from src.vision import pipeline as P

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET_GLOB = os.path.join(REPO_ROOT, "dataset", "*.jpg")
MAX_FRAMES = 60  # spread evenly across the dataset


# ---------------------------------------------------------------------------
# Reference: process_video_frame() exactly as it was before the scratch-buffer
# optimisation (inline path only -- the pool is never running under the tests).
# ---------------------------------------------------------------------------

def _reference_process_video_frame(frame):
    processed_data = {
        'detected_blocks': [],
        'detected_walls': [],
        'detected_orange': [],
        'detected_blue': [],
        'detected_magenta': [],
        'detected_close_black': []
    }

    my_slice = max(0, P.full_frame_roi[1] - P.GLOBAL_Y_OFFSET)
    ly_slice = P.line_roi_y - P.GLOBAL_Y_OFFSET
    cy_slice = P.close_block_roi[1] - P.GLOBAL_Y_OFFSET

    # --- 0. Colour masks + arena mask (inline; pool never runs in tests) ---
    cs_slice = P.prepare_colour_slice(frame)
    arena_mask = floor_mask = arena_sky = None
    mask_black = None
    if P.USE_ARENA_MASK:
        arena_mask, floor_mask, arena_sky, mask_black = P.build_arena_mask_from_prepared(cs_slice)
    if mask_black is None:
        mask_black = cv2.inRange(cs_slice, P.LOWER_BLACK, P.UPPER_BLACK)
    masks = P.compute_colour_masks_from_prepared(cs_slice, mask_black)

    if not P.USE_ARENA_MASK:
        arena_mask, floor_mask, arena_sky = None, None, None

    if arena_mask is None:
        arena_mask = P.ARENA_PASSTHROUGH
        floor_mask = None
    processed_data['arena_mask'] = arena_mask
    processed_data['arena_sky'] = arena_sky
    processed_data['arena_seeded'] = floor_mask is not None

    # --- 1. Crop the slice-sized masks down to each ROI ---
    mx, my, mw, mh = P.full_frame_roi
    lx, ly, lw, lh = P.line_roi_x, P.line_roi_y, P.line_roi_w, P.line_roi_h
    cx, cy, cw, ch = P.close_block_roi

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

    # --- 2. Reconstruct slice-sized global masks (OLD zeros + bitwise_or) ---
    global_red_mask = np.zeros((P.SLICE_HEIGHT, P.FRAME_WIDTH), dtype="uint8")
    global_green_mask = np.zeros((P.SLICE_HEIGHT, P.FRAME_WIDTH), dtype="uint8")
    global_blue_mask = np.zeros((P.SLICE_HEIGHT, P.FRAME_WIDTH), dtype="uint8")
    global_magenta_mask = np.zeros((P.SLICE_HEIGHT, P.FRAME_WIDTH), dtype="uint8")

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

    roi_mask_walls_slice = P.roi_mask_walls[P.GLOBAL_Y_OFFSET:P.GLOBAL_Y_END, :]
    roi_mask_close_black_slice = P.roi_mask_close_black[P.GLOBAL_Y_OFFSET:P.GLOBAL_Y_END, :]

    final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls_slice)
    final_mask_close_black = cv2.bitwise_and(black_or_magenta_mask, roi_mask_close_black_slice)

    # --- 4. Contour finding ---
    if cv2.countNonZero(mask_magenta_main) > 0:
        contours, _ = cv2.findContours(mask_magenta_main, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > P.MAGENTA_MIN_AREA:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    ccx = int(M["m10"] / M["m00"]) + mx
                    ccy = int(M["m01"] / M["m00"]) + (P.GLOBAL_Y_OFFSET + my_slice)
                    contour_global = contour + [mx, P.GLOBAL_Y_OFFSET + my_slice]

                    leftmost_x = contour_global[:, 0, 0].min()
                    rightmost_x = contour_global[:, 0, 0].max()

                    dist_to_center_left = abs(leftmost_x - P.FRAME_MIDPOINT_X)
                    dist_to_center_right = abs(rightmost_x - P.FRAME_MIDPOINT_X)

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

                        if P.USE_GROUND_CONTACT and floor_mask is not None and b_type == 'block':
                            rx, ry, rw, rh = cv2.boundingRect(cnt_global)
                            probe_y = ry + rh + P.GROUND_PROBE_DY
                            if probe_y < P.ARENA_Y_BOTTOM:
                                strip = floor_mask[probe_y, rx:rx + rw]
                                if strip.size and (np.count_nonzero(strip) / strip.size) < P.GROUND_CONTACT_MIN:
                                    continue

                        blocks.append({'type': b_type, 'color': b_color, 'area': area,
                                       'centroid': (bcx, bcy), 'contour': cnt_global})
        return blocks

    mask_black_close = crop(pure_black_mask, cy_slice, ch, cx, cw)
    if cw > 0 and ch > 0:
        arena_close = arena_mask[cy:cy + ch, cx:cx + cw]
        mask_black_close = cv2.bitwise_and(mask_black_close, arena_close)

    all_detected_blocks = []
    all_detected_blocks.extend(process_block_contours(mask_red_main, mx, P.GLOBAL_Y_OFFSET + my_slice, 'block', 'red', P.BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_main, mx, P.GLOBAL_Y_OFFSET + my_slice, 'block', 'green', P.BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_red_close, cx, P.GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'red', P.CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_close, cx, P.GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'green', P.CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_magenta_close, cx, P.GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'magenta', P.CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_black_close, cx, P.GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'black', P.CLOSE_BLOCK_MIN_AREA))

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
            if area > P.WALL_MIN_AREA:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    kcx = int(M["m10"] / M["m00"])
                    kcy = int(M["m01"] / M["m00"]) + P.GLOBAL_Y_OFFSET
                    contour_global = contour + [0, P.GLOBAL_Y_OFFSET]
                    processed_data['detected_close_black'].append({
                        'type': 'close_black',
                        'color': 'black',
                        'area': area,
                        'centroid': (kcx, kcy),
                        'contour': contour_global
                    })

    wall_contours_by_roi = {job['type']: [] for job in [P.left_side_job, P.right_side_job, P.inner_left_side_job, P.inner_right_side_job]}
    if cv2.countNonZero(final_mask_walls) > 0:
        contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > P.WALL_MIN_AREA:
                M = cv2.moments(c)
                if M["m00"] == 0: continue
                wcx = int(M["m10"] / M["m00"])

                job_type = 'unknown'
                if P.left_side_job['roi'][0] <= wcx < P.left_side_job['roi'][0] + P.left_side_job['roi'][2]: job_type = P.left_side_job['type']
                elif P.right_side_job['roi'][0] <= wcx < P.right_side_job['roi'][0] + P.right_side_job['roi'][2]: job_type = P.right_side_job['type']
                elif P.inner_left_side_job['roi'][0] <= wcx < P.inner_left_side_job['roi'][0] + P.inner_left_side_job['roi'][2]: job_type = P.inner_left_side_job['type']
                elif P.inner_right_side_job['roi'][0] <= wcx < P.inner_right_side_job['roi'][0] + P.inner_right_side_job['roi'][2]: job_type = P.inner_right_side_job['type']

                if job_type != 'unknown':
                    wall_contours_by_roi[job_type].append(c)

    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                wcx = int(M["m10"] / M["m00"])
                wcy = int(M["m01"] / M["m00"]) + P.GLOBAL_Y_OFFSET
                biggest_contour_global = biggest_contour + [0, P.GLOBAL_Y_OFFSET]
                processed_data['detected_walls'].append({'type': job_type, 'color': 'black', 'area': area, 'centroid': (wcx, wcy), 'contour': biggest_contour_global})

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
# Comparison helpers
# ---------------------------------------------------------------------------

def _load_frames():
    paths = sorted(glob.glob(DATASET_GLOB))
    assert paths, f"No dataset frames found at {DATASET_GLOB}"
    step = max(1, len(paths) // MAX_FRAMES)
    frames = []
    for p in paths[::step][:MAX_FRAMES]:
        img = cv2.imread(p)
        if img is not None and img.shape == (P.FRAME_HEIGHT, P.FRAME_WIDTH, 3):
            frames.append((os.path.basename(p), img))
    assert frames, "Dataset frames exist but none matched the pipeline frame size"
    return frames


def _assert_detections_equal(ref, new, ctx):
    for key in P.DETECTION_LISTS:
        r_list, n_list = ref.get(key, []), new.get(key, [])
        assert len(r_list) == len(n_list), \
            f"{ctx}: {key} count {len(r_list)} != {len(n_list)}"
        for i, (r, n) in enumerate(zip(r_list, n_list)):
            for field in ('type', 'color', 'area', 'centroid', 'target_x', 'target_y'):
                assert r.get(field) == n.get(field), \
                    f"{ctx}: {key}[{i}].{field} {r.get(field)!r} != {n.get(field)!r}"
            assert np.array_equal(r['contour'], n['contour']), \
                f"{ctx}: {key}[{i}].contour differs"

    assert np.array_equal(ref['arena_mask'], new['arena_mask']), f"{ctx}: arena_mask differs"
    r_sky, n_sky = ref.get('arena_sky'), new.get('arena_sky')
    assert (r_sky is None) == (n_sky is None), f"{ctx}: arena_sky presence differs"
    if r_sky is not None:
        assert np.array_equal(r_sky, n_sky), f"{ctx}: arena_sky differs"
    assert ref['arena_seeded'] == new['arena_seeded'], f"{ctx}: arena_seeded differs"
    assert ref['line_roi_wall_pct'] == new['line_roi_wall_pct'], f"{ctx}: line_roi_wall_pct differs"


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_process_video_frame_matches_reference():
    """The optimised pipeline reproduces the pre-optimisation output exactly."""
    P.vision_pool = None  # force the inline path, same as the reference
    for name, frame in _load_frames():
        ref = _reference_process_video_frame(frame)
        new = P.process_video_frame(frame)
        _assert_detections_equal(ref, new, name)


def test_process_video_frame_is_repeatable():
    """The scratch buffers carry no state between frames: processing the same
    frame after a different one gives the same answer as processing it fresh."""
    P.vision_pool = None
    frames = _load_frames()
    first_name, first_frame = frames[0]
    baseline = P.process_video_frame(first_frame)
    for _, other in frames[1:8]:
        P.process_video_frame(other)
    again = P.process_video_frame(first_frame)
    _assert_detections_equal(baseline, again, f"{first_name} (repeat)")


def test_annotate_does_not_mutate_input():
    """CameraThread now hands frames out by reference; annotation must copy."""
    P.vision_pool = None
    _, frame = _load_frames()[0]
    detections = P.process_video_frame(frame)
    before = frame.copy()
    P.annotate_video_frame(frame, detections, "clockwise", debug_info="test",
                           visual_target_x=320,
                           visual_target_line=((320, 359), (300, 200), 0.0))
    assert np.array_equal(frame, before), \
        "annotate_video_frame mutated its input frame -- this breaks the " \
        "zero-copy CameraThread.get_next_frame() contract"


def main():
    tests = [
        test_process_video_frame_matches_reference,
        test_process_video_frame_is_repeatable,
        test_annotate_does_not_mutate_input,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
