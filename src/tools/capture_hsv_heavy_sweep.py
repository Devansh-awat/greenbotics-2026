"""
Captures a live frame from the Pi camera and generates:
1. Unfiltered HSV slice
2. Gaussian Blur HSV slice
3. Ultra-Heavy Bilateral (d=11, sigma_color=150, sigma_space=150)
4. Extreme-Heavy Bilateral (d=15, sigma_color=200, sigma_space=200)
5. High Color / Low Space Bilateral (d=9, sigma_color=150, sigma_space=30)
6. Low Color / High Space Bilateral (d=9, sigma_color=30, sigma_space=150)
"""

import os
import time
import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning

OUTPUT_DIR = "captured_hsv_heavy_sweep"

PARAM_SETS = [
    {
        "filename": "02_slice_hsv_gaussian_1x7.png",
        "type": "gaussian",
        "ksize": (1, 7),
        "desc": "Gaussian Blur (1x7 kernel, legacy pipeline)"
    },
    {
        "filename": "03_slice_hsv_ultra_heavy_d11_sig150.png",
        "type": "bilateral",
        "d": 11, "sc": 150, "ss": 150,
        "desc": "Ultra-Heavy (d=11, sigmaColor=150, sigmaSpace=150)"
    },
    {
        "filename": "04_slice_hsv_extreme_d15_sig200.png",
        "type": "bilateral",
        "d": 15, "sc": 200, "ss": 200,
        "desc": "Extreme-Heavy (d=15, sigmaColor=200, sigmaSpace=200)"
    },
    {
        "filename": "05_slice_hsv_high_color_low_space_d9.png",
        "type": "bilateral",
        "d": 9, "sc": 150, "ss": 30,
        "desc": "High Color / Low Space Blur (d=9, sigmaColor=150, sigmaSpace=30)"
    },
    {
        "filename": "06_slice_hsv_low_color_high_space_d9.png",
        "type": "bilateral",
        "d": 9, "sc": 30, "ss": 150,
        "desc": "Strict Edge / High Spatial Blur (d=9, sigmaColor=30, sigmaSpace=150)"
    },
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
        cv2.imwrite(os.path.join(OUTPUT_DIR, "00_raw_bgr_frame.png"), frame)

        # Working slice
        y0 = tuning.GLOBAL_Y_OFFSET
        y1 = tuning.GLOBAL_Y_END
        slice_bgr = frame[y0:y1, :]

        # 1. Unfiltered HSV Slice
        slice_hsv_unfiltered = cv2.cvtColor(slice_bgr, cv2.COLOR_BGR2HSV)
        cv2.imwrite(os.path.join(OUTPUT_DIR, "01_slice_hsv_unfiltered.png"), slice_hsv_unfiltered)
        print("Saved 01_slice_hsv_unfiltered.png (Unfiltered)")

        # 2. Process all sweep filters
        for p in PARAM_SETS:
            fname = p["filename"]
            if p["type"] == "gaussian":
                filtered_bgr = cv2.GaussianBlur(slice_bgr, p["ksize"], 0)
            elif p["type"] == "bilateral":
                filtered_bgr = cv2.bilateralFilter(slice_bgr, p["d"], p["sc"], p["ss"])
            
            filtered_hsv = cv2.cvtColor(filtered_bgr, cv2.COLOR_BGR2HSV)
            cv2.imwrite(os.path.join(OUTPUT_DIR, fname), filtered_hsv)
            print(f"Saved {fname} -- {p['desc']}")

        print("\nAll heavy sweep images saved to:", OUTPUT_DIR)

    finally:
        camera.cleanup()

if __name__ == "__main__":
    main()
