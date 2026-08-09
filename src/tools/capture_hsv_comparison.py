"""
Captures a live frame from the Pi camera and saves:
1. Raw BGR frame
2. HSV frame with NO filtering ("nothing")
3. HSV frame WITH bilateral filtering
4. HSV working slice with NO filtering
5. HSV working slice WITH bilateral filtering
"""

import os
import time
import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning

OUTPUT_DIR = "captured_hsv_images"

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print("Initializing camera...")
    if not camera.initialize():
        print("ERROR: Could not initialize camera!")
        return

    try:
        # Allow camera auto-exposure and white balance to settle
        time.sleep(1.0)
        print("Capturing live frame...")
        frame = camera.capture_frame()
        if frame is None:
            print("ERROR: Captured frame is None!")
            return

        print(f"Frame captured successfully: shape={frame.shape}")

        # 1. Raw BGR Frame
        cv2.imwrite(os.path.join(OUTPUT_DIR, "01_raw_bgr_frame.png"), frame)

        # 2. HSV with NOTHING (Unfiltered)
        hsv_unfiltered = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "02_hsv_unfiltered.png"), hsv_unfiltered)

        # 3. HSV WITH BILATERAL FILTER
        d = getattr(tuning, 'BILATERAL_D', 5)
        sigma_color = getattr(tuning, 'BILATERAL_SIGMA_COLOR', 50)
        sigma_space = getattr(tuning, 'BILATERAL_SIGMA_SPACE', 50)

        bilateral_frame = cv2.bilateralFilter(frame, d, sigma_color, sigma_space)
        hsv_bilateral = cv2.cvtColor(bilateral_frame, cv2.COLOR_BGR2HSV)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "03_hsv_bilateral.png"), hsv_bilateral)

        # 4. Pipeline Slice Comparison
        y0 = tuning.GLOBAL_Y_OFFSET
        y1 = tuning.GLOBAL_Y_END
        slice_unfiltered = frame[y0:y1, :]
        slice_hsv_unfiltered = cv2.cvtColor(slice_unfiltered, cv2.COLOR_BGR2HSV)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "04_slice_hsv_unfiltered.png"), slice_hsv_unfiltered)

        slice_bilateral = cv2.bilateralFilter(slice_unfiltered, d, sigma_color, sigma_space)
        slice_hsv_bilateral = cv2.cvtColor(slice_bilateral, cv2.COLOR_BGR2HSV)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "05_slice_hsv_bilateral.png"), slice_hsv_bilateral)

        print("Saved images to", OUTPUT_DIR)
        print("Done!")

    finally:
        camera.cleanup()

if __name__ == "__main__":
    main()
