"""
Dumps stages of the updated open challenge vision pipeline to PNGs:
1. 01_raw.png              : Raw BGR frame (640x360)
2. 02_full_hsv.png         : Full HSV frame (640x360)
3. 03_full_bilateral.png    : Full bilateral filtered HSV frame (640x360)
4. 04_black_mask.png       : Black mask (padded to 640x360)
5. 05_black_mask_rois.png  : Black mask bitwised AND onto wall ROIs (640x360)
6. 06_black_mask_arena.png : Black mask bitwised AND with arena mask (640x360)
7. 07_final_annotated.png  : Final annotated frame without text (640x360)

No orange/blue masks are generated. All images are padded to match full video frame size (640x360).

Run from repo root: python3 -m src.tools.capture_open_pipeline [--input <image_path>]
"""

import argparse
import os
import shutil
import time

import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning
from src.vision import pipeline as vision

OUTPUT_DIR = "pipeline_images/open"


def pad_slice(slice_img, top_offset=tuning.GLOBAL_Y_OFFSET, target_height=tuning.FRAME_HEIGHT, target_width=tuning.FRAME_WIDTH):
    """Pads a cropped slice image (height = GLOBAL_Y_END - GLOBAL_Y_OFFSET) above and below to match target_height x target_width."""
    h, w = slice_img.shape[:2]
    if h == target_height and w == target_width:
        return slice_img.copy()
    if slice_img.ndim == 3:
        padded = np.zeros((target_height, target_width, slice_img.shape[2]), dtype=slice_img.dtype)
    else:
        padded = np.zeros((target_height, target_width), dtype=slice_img.dtype)
    padded[top_offset:top_offset + h, 0:w] = slice_img
    return padded


def process_frame_all_stages(frame):
    """
    Given a BGR frame (640x360), processes all vision stages and returns a dictionary of
    all pipeline stage images, each padded/sized to (360, 640) for standard video assembly.
    """
    images = {}

    # --- 1. Raw BGR Frame (640x360) ---
    images["01_raw.png"] = frame.copy()

    # --- 2. Full HSV Frame (640x360) ---
    full_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    images["02_full_hsv.png"] = full_hsv

    # --- 3. Full Bilateral Filtered Frame (640x360) ---
    if getattr(tuning, 'HSV_BEFORE_BLUR', True):
        full_cs = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab if getattr(tuning, 'USE_LAB', False) else cv2.COLOR_BGR2HSV)
        if getattr(tuning, 'USE_BILATERAL', True):
            full_bilateral = cv2.bilateralFilter(
                full_cs, tuning.BILATERAL_D, tuning.BILATERAL_SIGMA_COLOR, tuning.BILATERAL_SIGMA_SPACE
            )
        else:
            full_bilateral = cv2.GaussianBlur(full_cs, (1, 7), 0)
    else:
        if getattr(tuning, 'USE_BILATERAL', True):
            filtered_bgr = cv2.bilateralFilter(
                frame, tuning.BILATERAL_D, tuning.BILATERAL_SIGMA_COLOR, tuning.BILATERAL_SIGMA_SPACE
            )
        else:
            filtered_bgr = cv2.GaussianBlur(frame, (1, 7), 0)
        full_bilateral = cv2.cvtColor(filtered_bgr, cv2.COLOR_BGR2Lab if getattr(tuning, 'USE_LAB', False) else cv2.COLOR_BGR2HSV)

    images["03_full_bilateral.png"] = full_bilateral

    # --- Prepare slice for pipeline masks & arena mask ---
    cs_slice = vision.prepare_colour_slice(frame)
    arena_mask, floor_mask, sky, mask_black_slice = vision.build_arena_mask_from_prepared(cs_slice)

    if mask_black_slice is None:
        mask_black_slice = cv2.inRange(cs_slice, tuning.LOWER_BLACK, tuning.UPPER_BLACK)

    # --- 4. Black Mask (Padded to 640x360) ---
    padded_black_mask = pad_slice(mask_black_slice)
    images["04_black_mask.png"] = padded_black_mask

    # --- 5. Black Mask Bitwised AND onto ROIs (640x360) ---
    black_mask_rois = cv2.bitwise_and(padded_black_mask, tuning.roi_mask_walls)
    images["05_black_mask_rois.png"] = black_mask_rois

    # --- 6. Black Mask Bitwised AND with Arena Mask (640x360) ---
    if arena_mask is None:
        arena_mask = vision.ARENA_PASSTHROUGH
    black_mask_arena = cv2.bitwise_and(padded_black_mask, arena_mask)
    images["06_black_mask_arena.png"] = black_mask_arena

    # --- 7. Final Annotated Frame Without Text (640x360) ---
    detections = vision.process_video_frame(frame)
    annotated_frame = vision.annotate_video_frame(frame, detections, None, debug_info="")
    images["07_final_annotated.png"] = annotated_frame

    return images


def run_pipeline_capture(input_image_path=None):
    # --- Clean output directory first ---
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

    frame = None
    if input_image_path:
        print(f"Loading input frame from {input_image_path}...")
        frame = cv2.imread(input_image_path)
        if frame is None:
            print(f"ERROR: Could not read image from {input_image_path}")
            return
        if frame.shape[:2] != (tuning.FRAME_HEIGHT, tuning.FRAME_WIDTH):
            frame = cv2.resize(frame, (tuning.FRAME_WIDTH, tuning.FRAME_HEIGHT))
    else:
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
        finally:
            camera.cleanup()

    print(f"Captured/Loaded frame shape: {frame.shape}")

    images = process_frame_all_stages(frame)

    # Save images
    print(f"Saving {len(images)} pipeline images to {OUTPUT_DIR}/ ...")
    for filename, img in images.items():
        cv2.imwrite(os.path.join(OUTPUT_DIR, filename), img)
        print(f"  {filename} ({img.shape})")
    print("Pipeline capture complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture Open Challenge vision pipeline stages.")
    parser.add_argument("--input", type=str, default=None, help="Optional path to input image file instead of live camera.")
    args = parser.parse_args()

    run_pipeline_capture(input_image_path=args.input)
