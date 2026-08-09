"""
Captures a live frame from the Pi camera and generates:
- 1 Unfiltered HSV slice
- 4 Bilateral Filtered HSV slices with different parameter sets (d, sigma_color, sigma_space)
"""

import os
import time
import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning

OUTPUT_DIR = "captured_hsv_sweep"

PARAM_SETS = [
    {"name": "02_slice_hsv_bilateral_d3_sig25", "d": 3, "sc": 25, "ss": 25, "desc": "Light (d=3, sigma=25)"},
    {"name": "03_slice_hsv_bilateral_d5_sig50", "d": 5, "sc": 50, "ss": 50, "desc": "Default Tuning (d=5, sigma=50)"},
    {"name": "04_slice_hsv_bilateral_d7_sig75", "d": 7, "sc": 75, "ss": 75, "desc": "Medium-Strong (d=7, sigma=75)"},
    {"name": "05_slice_hsv_bilateral_d9_sig100", "d": 9, "sc": 100, "ss": 100, "desc": "Heavy (d=9, sigma=100)"},
]

def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print("Initializing camera...")
    if not camera.initialize():
        print("ERROR: Could not initialize camera!")
        return

    try:
        time.sleep(1.0)
        print("Capturing live frame...")
        frame = camera.capture_frame()
        if frame is None:
            print("ERROR: Captured frame is None!")
            return

        print(f"Captured frame shape: {frame.shape}")

        # Save raw frame for baseline
        cv2.imwrite(os.path.join(OUTPUT_DIR, "00_raw_bgr_frame.png"), frame)

        # Working slice
        y0 = tuning.GLOBAL_Y_OFFSET
        y1 = tuning.GLOBAL_Y_END
        slice_bgr = frame[y0:y1, :]

        # 1. Unfiltered HSV Slice
        slice_hsv_unfiltered = cv2.cvtColor(slice_bgr, cv2.COLOR_BGR2HSV)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "01_slice_hsv_unfiltered.png"), slice_hsv_unfiltered)
        print("Saved 01_slice_hsv_unfiltered.png")

        # 2. Sweep Bilateral Filter Parameters
        for p in PARAM_SETS:
            d, sc, ss = p["d"], p["sc"], p["ss"]
            filtered_bgr = cv2.bilateralFilter(slice_bgr, d, sc, ss)
            filtered_hsv = cv2.cvtColor(filtered_bgr, cv2.COLOR_BGR2HSV)
            filename = f"{p['name']}.png"
            cv2.imwrite(os.path.join(OUTPUT_DIR, filename), filtered_hsv)
            print(f"Saved {filename} ({p['desc']})")

        print("\nAll HSV slice images saved to:", OUTPUT_DIR)

    finally:
        camera.cleanup()

if __name__ == "__main__":
    main()
