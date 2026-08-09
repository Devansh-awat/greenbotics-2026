"""
Dumps every stage of the shared vision pipeline (src/vision/pipeline.py) to PNGs,
using one live camera frame and the obstacle challenge's own ROIs/colour ranges
(src/obstacle_challenge/tuning.py) -- pillar and parking-wall detection included.

Calls the real pipeline functions (build_arena_mask, compute_colour_masks,
process_video_frame, annotate_video_frame) rather than re-deriving the masks, so
the images can never drift from what the control loop actually sees.

See capture_open_pipeline.py for the open-challenge equivalent: same pipeline
code, same tuning module, but it skips the pillar-specific masks the open track
has no use for.

Run from the repo root:  python3 -m src.tools.capture_obstacle_pipeline
"""

import os
import time

import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning
from src.vision import pipeline as vision

OUTPUT_DIR = "pipeline_images/obstacle"


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

        # --- ROI overlay: every ROI the obstacle loop reads from tuning.py ---
        roi_overlay = frame.copy()
        rois = [
            ("Left Wall", tuning.left_side_job['roi'], (255, 255, 0)),
            ("Right Wall", tuning.right_side_job['roi'], (255, 255, 0)),
            ("Inner L Wall", tuning.inner_left_side_job['roi'], (255, 255, 0)),
            ("Inner R Wall", tuning.inner_right_side_job['roi'], (255, 255, 0)),
            ("Line ROI", (tuning.line_roi_x, tuning.line_roi_y, tuning.line_roi_w, tuning.line_roi_h), (0, 165, 255)),
            ("Close Black", (tuning.close_x, tuning.close_y, tuning.close_w, tuning.close_h), (0, 0, 255)),
            ("Main Blocks", tuning.full_frame_roi, (0, 255, 0)),
            ("Close Blocks", tuning.close_block_roi, (255, 0, 255)),
        ]
        for label, (x, y, w, h), color in rois:
            cv2.rectangle(roi_overlay, (x, y), (x + w, y + h), color, 2)
            cv2.putText(roi_overlay, label, (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        images["02_roi_overlay.png"] = roi_overlay

        # --- Arena mask: v5's floor/skyline segmentation ---
        arena_mask, floor_mask, sky = vision.build_arena_mask(frame)
        if arena_mask is not None:
            images["03_arena_mask.png"] = arena_mask
            images["04_floor_mask.png"] = floor_mask
            sky_overlay = frame.copy()
            pts = np.stack([np.arange(tuning.FRAME_WIDTH, dtype=np.int32),
                             np.asarray(sky).astype(np.int32) + vision.ARENA_Y_TOP], axis=1)
            cv2.polylines(sky_overlay, [pts], False, (0, 200, 255), 2)
            images["05_arena_skyline.png"] = sky_overlay
        else:
            print("Arena mask: seed point isn't on a floor blob (nose into a wall / camera covered)")

        # --- Filtered slice: the bilateral/Gaussian step, before HSV/Lab and
        # thresholding -- USE_BILATERAL/BILATERAL_* in tuning.py control it.
        images["06_filtered_slice.png"] = vision.filter_slice(frame)

        # --- Colour masks: red/green/magenta/orange/blue/black on the working slice ---
        masks = vision.compute_colour_masks(frame)
        images["07_mask_red.png"] = masks['red']
        images["08_mask_green.png"] = masks['green']
        images["09_mask_magenta.png"] = masks['magenta']
        images["10_mask_orange.png"] = masks['orange']
        images["11_mask_blue.png"] = masks['blue']
        images["12_mask_black.png"] = masks['black']

        # --- Full detection + annotation: exactly what the control loop sees ---
        detections = vision.process_video_frame(frame)
        annotated = vision.annotate_video_frame(
            frame, detections, "clockwise", debug_info="pipeline capture")
        images["13_annotated_frame.png"] = annotated

        print(f"Saving {len(images)} pipeline images to {OUTPUT_DIR}/ ...")
        for filename, img in images.items():
            cv2.imwrite(os.path.join(OUTPUT_DIR, filename), img)
            print(f"  {filename}")
        print("Pipeline capture complete.")
    finally:
        camera.cleanup()


if __name__ == "__main__":
    run_pipeline_capture()
