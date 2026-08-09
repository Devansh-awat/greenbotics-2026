"""
Captures a live frame from the Pi camera at full resolution (640x360) and generates:
- Normal (BGR) and HSV versions for:
  1. Unfiltered (Raw)
  2. 3 Gaussian Blurs (3x3, 1x7 legacy, 9x9 heavy)
  3. 4 Bilateral Filters (Weak d=3, Medium d=5, Strong d=7, Super Strong d=11)
- Writes a Markdown comparison file with side-by-side tables.
"""

import os
import time
import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge import tuning

OUTPUT_DIR = "captured_full_filter_comparison"

FILTERS = [
    {
        "id": "01_unfiltered",
        "title": "Unfiltered (Raw)",
        "type": "none",
        "params": {}
    },
    {
        "id": "02_gaussian_3x3",
        "title": "Gaussian Blur (Small 3x3)",
        "type": "gaussian",
        "ksize": (3, 3), "sigma": 0
    },
    {
        "id": "03_gaussian_1x7",
        "title": "Gaussian Blur (Medium 1x7 - Legacy Pipeline)",
        "type": "gaussian",
        "ksize": (1, 7), "sigma": 0
    },
    {
        "id": "04_gaussian_9x9",
        "title": "Gaussian Blur (Heavy 9x9)",
        "type": "gaussian",
        "ksize": (9, 9), "sigma": 2.0
    },
    {
        "id": "05_bilateral_weak",
        "title": "Weak Bilateral Filter (d=3, sigma=25)",
        "type": "bilateral",
        "d": 3, "sc": 25, "ss": 25
    },
    {
        "id": "06_bilateral_medium",
        "title": "Medium Bilateral Filter (d=5, sigma=50 - Default Tuning)",
        "type": "bilateral",
        "d": 5, "sc": 50, "ss": 50
    },
    {
        "id": "07_bilateral_strong",
        "title": "Strong Bilateral Filter (d=7, sigma=75)",
        "type": "bilateral",
        "d": 7, "sc": 75, "ss": 75
    },
    {
        "id": "08_bilateral_super_strong",
        "title": "Super Strong Bilateral Filter (d=11, sigma=150)",
        "type": "bilateral",
        "d": 11, "sc": 150, "ss": 150
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

        print(f"Captured frame dimensions: {frame.shape[1]}x{frame.shape[0]} (Channels: {frame.shape[2]})")

        for item in FILTERS:
            fid = item["id"]
            ftype = item["type"]

            if ftype == "none":
                filtered_bgr = frame.copy()
            elif ftype == "gaussian":
                filtered_bgr = cv2.GaussianBlur(frame, item["ksize"], item["sigma"])
            elif ftype == "bilateral":
                filtered_bgr = cv2.bilateralFilter(frame, item["d"], item["sc"], item["ss"])

            filtered_hsv = cv2.cvtColor(filtered_bgr, cv2.COLOR_BGR2HSV)

            # Save full frame images
            norm_path = os.path.join(OUTPUT_DIR, f"{fid}_normal.png")
            hsv_path = os.path.join(OUTPUT_DIR, f"{fid}_hsv.png")

            cv2.imwrite(norm_path, filtered_bgr)
            cv2.imwrite(hsv_path, filtered_hsv)

            print(f"Processed {item['title']} -> {fid}_normal.png & {fid}_hsv.png")

        print("\nAll full resolution filter images generated successfully!")

    finally:
        camera.cleanup()

if __name__ == "__main__":
    main()
