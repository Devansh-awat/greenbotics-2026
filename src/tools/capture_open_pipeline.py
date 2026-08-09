"""
Dumps stages of the open challenge vision pipeline to PNGs:
1. Raw (raw BGR frame)
2. HSV Raw (unfiltered HSV)
3. HSV Bilateral (bilateral filtered HSV)
4. Bilateral BGR (bilateral filtered BGR)
5. Skyline Full Mask (binary arena/skyline mask)
6. Skyline Overlay (skyline boundary line overlay on frame)
7. Color masks (black, orange, blue mask PNGs)
8. ROI (open challenge ROI overlay)
9. Final calculation (annotated frame with calculations: wall error, PD angle, turn trigger)

Cleans old files in the destination directory first.

Run from repo root: python3 -m src.tools.capture_open_pipeline
"""

import os
import shutil
import time

import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning
from src.vision import pipeline as vision

OUTPUT_DIR = "pipeline_images/open"


def run_pipeline_capture():
    # --- 0. Clean output directory first ---
    if os.path.exists(OUTPUT_DIR):
        print(f"Removing old files in {OUTPUT_DIR}/...")
        for f in os.listdir(OUTPUT_DIR):
            file_path = os.path.join(OUTPUT_DIR, f)
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.remove(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
    else:
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

        images = {}

        # --- 1. Raw BGR Frame ---
        images["01_raw.png"] = frame.copy()

        # --- 2. HSV Raw & 3. HSV Bilateral Filter & 4. Bilateral BGR ---
        slice_bgr = frame[tuning.GLOBAL_Y_OFFSET:tuning.GLOBAL_Y_END, :]

        # Raw HSV (before bilateral filter)
        hsv_raw = cv2.cvtColor(slice_bgr, cv2.COLOR_BGR2HSV)
        images["02_hsv_raw.png"] = hsv_raw

        # Bilateral Filter applied directly on HSV
        if tuning.USE_BILATERAL:
            hsv_filtered = cv2.bilateralFilter(
                hsv_raw, tuning.BILATERAL_D, tuning.BILATERAL_SIGMA_COLOR, tuning.BILATERAL_SIGMA_SPACE
            )
            bilateral_bgr = cv2.bilateralFilter(
                frame, tuning.BILATERAL_D, tuning.BILATERAL_SIGMA_COLOR, tuning.BILATERAL_SIGMA_SPACE
            )
        else:
            hsv_filtered = cv2.GaussianBlur(hsv_raw, (1, 7), 0)
            bilateral_bgr = cv2.GaussianBlur(frame, (1, 7), 0)

        images["03_hsv_bilateral.png"] = hsv_filtered
        images["04_bilateral_bgr.png"] = bilateral_bgr

        # --- 5. Skyline Full Mask & 6. Skyline Overlay ---
        arena_mask, floor_mask, sky = vision.build_arena_mask(frame)
        if arena_mask is not None:
            images["05_skyline_full_mask.png"] = arena_mask
            skyline_overlay = frame.copy()
            pts = np.stack([
                np.arange(tuning.FRAME_WIDTH, dtype=np.int32),
                np.asarray(sky).astype(np.int32) + vision.ARENA_Y_TOP
            ], axis=1)
            cv2.polylines(skyline_overlay, [pts], False, (0, 200, 255), 2)
            images["06_skyline_overlay.png"] = skyline_overlay
        else:
            print("WARNING: Arena mask could not be computed (seed point off floor)")

        # --- 7. Color Masks (Open Challenge: Black, Orange, Blue) ---
        masks = vision.compute_colour_masks(frame)
        images["07a_mask_black.png"] = masks['black']
        images["07b_mask_orange.png"] = masks['orange']
        images["07c_mask_blue.png"] = masks['blue']

        # --- 8. ROI Overlay ---
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
        images["08_roi_overlay.png"] = roi_overlay

        # --- 9. Final Calculation (Annotated Frame) ---
        detections = vision.process_video_frame(frame)
        detected_walls = detections.get('detected_walls', [])
        left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
        right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
        wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
        wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')

        if left_pixel_size < 700 and (right_pixel_size + wall_inner_right_size) > 100:
            right_pixel_size_boosted = right_pixel_size * 2 + 25000
            left_pixel_size_boosted = left_pixel_size
        elif right_pixel_size < 700 and (left_pixel_size + wall_inner_left_size) > 100:
            left_pixel_size_boosted = left_pixel_size * 2 + 25000
            right_pixel_size_boosted = right_pixel_size
        else:
            left_pixel_size_boosted = left_pixel_size
            right_pixel_size_boosted = right_pixel_size

        wall_error = (left_pixel_size_boosted + wall_inner_left_size) - (right_pixel_size_boosted + wall_inner_right_size)
        angle = (wall_error * tuning.WALL_KP) + 1
        angle = np.clip(angle, -40, 40)

        close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
        line_roi_wall_pct = detections.get('line_roi_wall_pct', 0)

        debug_info = [
            f"L:{int(left_pixel_size)}", f"R:{int(right_pixel_size)}",
            f"IL:{int(wall_inner_left_size)}", f"IR:{int(wall_inner_right_size)}",
            f"Err:{int(wall_error)}", f"Wall%:{int(line_roi_wall_pct)}",
            f"CloseBlk:{int(close_black_area)}", f"Angle:{int(angle)}"
        ]

        annotated_frame = vision.annotate_video_frame(
            frame, detections, None, debug_info=str(debug_info)
        )
        images["09_final_calculation_annotated.png"] = annotated_frame

        # Save images
        print(f"Saving {len(images)} pipeline images to {OUTPUT_DIR}/ ...")
        for filename, img in images.items():
            cv2.imwrite(os.path.join(OUTPUT_DIR, filename), img)
            print(f"  {filename}")
        print("Pipeline capture complete.")
    finally:
        camera.cleanup()


if __name__ == "__main__":
    run_pipeline_capture()


