"""
Captures a live frame from the Pi camera (640x360) and sweeps sigmaColor and sigmaSpace values at fixed d=5:
Generates Normal (BGR) and HSV images for each parameter pair and saves them to docs/filter_images/.
Also updates docs/filter_comparison.md with embedded markdown images.
"""

import os
import time
import cv2
import numpy as np

from src.sensors import camera

OUTPUT_DIR = "docs/filter_images"

SIGMA_SWEEP = [
    {"id": "01_unfiltered", "title": "Unfiltered (Raw Frame)", "type": "none"},
    {"id": "02_d5_sc15_ss15", "title": "d=5, sigmaColor=15, sigmaSpace=15 (Low/Low)", "type": "bilateral", "d": 5, "sc": 15, "ss": 15},
    {"id": "03_d5_sc50_ss15", "title": "d=5, sigmaColor=50, sigmaSpace=15 (Med Color / Low Space)", "type": "bilateral", "d": 5, "sc": 50, "ss": 15},
    {"id": "04_d5_sc100_ss15", "title": "d=5, sigmaColor=100, sigmaSpace=15 (High Color / Low Space)", "type": "bilateral", "d": 5, "sc": 100, "ss": 15},
    {"id": "05_d5_sc15_ss50", "title": "d=5, sigmaColor=15, sigmaSpace=50 (Low Color / Med Space)", "type": "bilateral", "d": 5, "sc": 15, "ss": 50},
    {"id": "06_d5_sc50_ss50", "title": "d=5, sigmaColor=50, sigmaSpace=50 (Default Tuning)", "type": "bilateral", "d": 5, "sc": 50, "ss": 50},
    {"id": "07_d5_sc100_ss50", "title": "d=5, sigmaColor=100, sigmaSpace=50 (High Color / Med Space)", "type": "bilateral", "d": 5, "sc": 100, "ss": 50},
    {"id": "08_d5_sc15_ss100", "title": "d=5, sigmaColor=15, sigmaSpace=100 (Low Color / High Space)", "type": "bilateral", "d": 5, "sc": 15, "ss": 100},
    {"id": "09_d5_sc50_ss100", "title": "d=5, sigmaColor=50, sigmaSpace=100 (Med Color / High Space)", "type": "bilateral", "d": 5, "sc": 50, "ss": 100},
    {"id": "10_d5_sc150_ss150", "title": "d=5, sigmaColor=150, sigmaSpace=150 (Ultra High Color & Space)", "type": "bilateral", "d": 5, "sc": 150, "ss": 150},
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

        print(f"Captured frame shape: {frame.shape[1]}x{frame.shape[0]}")

        for item in SIGMA_SWEEP:
            fid = item["id"]
            ftype = item["type"]

            if ftype == "none":
                filtered_bgr = frame.copy()
            else:
                filtered_bgr = cv2.bilateralFilter(frame, item["d"], item["sc"], item["ss"])

            filtered_hsv = cv2.cvtColor(filtered_bgr, cv2.COLOR_BGR2HSV)

            norm_path = os.path.join(OUTPUT_DIR, f"{fid}_normal.png")
            hsv_path = os.path.join(OUTPUT_DIR, f"{fid}_hsv.png")

            cv2.imwrite(norm_path, filtered_bgr)
            cv2.imwrite(hsv_path, filtered_hsv)

            print(f"Saved {item['title']} -> {fid}_normal.png & {fid}_hsv.png")

        print("\nAll d=5 sigma sweep images generated successfully in", OUTPUT_DIR)

    finally:
        camera.cleanup()

if __name__ == "__main__":
    main()
