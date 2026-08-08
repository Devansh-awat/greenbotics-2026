"""
Dumps every stage of the shared vision pipeline (src/vision/pipeline.py) to PNGs,
using one live camera frame -- the open-challenge equivalent of
capture_obstacle_pipeline.py.

It is the SAME pipeline code and the SAME tuning module
(src/obstacle_challenge/tuning.py) that src/open_challenge/main.py imports --
see CLAUDE.md for why vision lives outside both challenge packages. The only
difference from the obstacle capture is which images get saved: the open loop
never reads detected_blocks/detected_magenta, so the pillar-specific colour
masks (red/green/magenta) are left out here; only the wall/line/close-black
masks the open loop actually steers on are captured.

Run from the repo root:  python3 -m src.tools.capture_open_pipeline
"""

import os
import time

import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning
from src.vision import pipeline as vision

OUTPUT_DIR = "pipeline_images/open"


def run_pipeline_capture():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("Initializing camera...")
    if not camera.initialize():
        print("ERROR: Failed to initialize camera!")
        return

    try:
        time.sleep(1.0)
        print("Capturing frame...")
        frame = camera.capture_frame()
        if frame is None:
            print("ERROR: Captured frame is None!")
            return
        print(f"Captured frame shape: {frame.shape}")

        images = {"01_raw_frame.png": frame}

        # --- ROI overlay: only the ROIs the open loop actually reads ---
        roi_overlay = frame.copy()
        rois = [
            ("Left Wall", tuning.left_side_job['roi'], (255, 255, 0)),
            ("Right Wall", tuning.right_side_job['roi'], (255, 255, 0)),
            ("Inner L Wall", tuning.inner_left_side_job['roi'], (255, 255, 0)),
            ("Inner R Wall", tuning.inner_right_side_job['roi'], (255, 255, 0)),
            ("Line ROI", (tuning.line_roi_x, tuning.line_roi_y, tuning.line_roi_w, tuning.line_roi_h), (0, 165, 255)),
            ("Close Black", (tuning.close_x, tuning.close_y, tuning.close_w, tuning.close_h), (0, 0, 255)),
        ]
        for label, (x, y, w, h), color in rois:
            cv2.rectangle(roi_overlay, (x, y), (x + w, y + h), color, 2)
            cv2.putText(roi_overlay, label, (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        images["02_roi_overlay.png"] = roi_overlay

        # --- Arena mask: v5's floor/skyline segmentation. The open loop doesn't
        # gate on it directly (that only matters for pillar detection), but it
        # still runs every frame as part of the shared pipeline, so capture it.
        arena_mask, floor_mask, sky = vision.build_arena_mask(frame)
        if arena_mask is not None:
            images["03_arena_mask.png"] = arena_mask
            sky_overlay = frame.copy()
            pts = np.stack([np.arange(tuning.FRAME_WIDTH, dtype=np.int32),
                             np.asarray(sky).astype(np.int32) + vision.ARENA_Y_TOP], axis=1)
            cv2.polylines(sky_overlay, [pts], False, (0, 200, 255), 2)
            images["04_arena_skyline.png"] = sky_overlay
        else:
            print("Arena mask: seed point isn't on a floor blob (nose into a wall / camera covered)")

        # --- Colour masks the open loop steers on: black (walls), orange/blue
        # (lap line). Pillar colours (red/green/magenta) are skipped -- the open
        # track has none and main.py never reads detected_blocks/detected_magenta.
        masks = vision.compute_colour_masks(frame)
        images["05_mask_black.png"] = masks['black']
        images["06_mask_orange.png"] = masks['orange']
        images["07_mask_blue.png"] = masks['blue']

        # --- Full detection + annotation: exactly what the control loop sees ---
        detections = vision.process_video_frame(frame)
        annotated = vision.annotate_video_frame(
            frame, detections, None, debug_info="pipeline capture")
        images["08_annotated_frame.png"] = annotated

        print(f"Saving {len(images)} pipeline images to {OUTPUT_DIR}/ ...")
        for filename, img in images.items():
            cv2.imwrite(os.path.join(OUTPUT_DIR, filename), img)
            print(f"  {filename}")
        print("Pipeline capture complete.")
    finally:
        camera.cleanup()


if __name__ == "__main__":
    run_pipeline_capture()
