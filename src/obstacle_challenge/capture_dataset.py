"""
Manual dataset capture tool for YOLO training.
Press the physical button (GPIO 23) to capture a frame, Ctrl+C to quit.
Images are saved as raw unannotated JPEGs to greenbotics-1/dataset/.
"""
import os
import sys
import cv2
import time
from datetime import datetime
from gpiozero import Button
from src.sensors import camera

DATASET_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'dataset'
)

def main():
    os.makedirs(DATASET_DIR, exist_ok=True)

    print("Initializing camera...")
    if not camera.initialize():
        print("Camera failed to initialize. Exiting.")
        sys.exit(1)

    button = Button(23)
    session_prefix = datetime.now().strftime("%Y%m%d_%H%M%S")
    count = 0

    print(f"Saving to: {DATASET_DIR}")
    print("Press the button to capture a photo, Ctrl+C to quit.\n")

    try:
        while True:
            button.wait_for_press()

            frame = camera.capture_frame()
            if frame is None:
                print("  ERROR: Failed to capture frame.")
                continue

            filename = f"{session_prefix}_{count:06d}.jpg"
            filepath = os.path.join(DATASET_DIR, filename)
            cv2.imwrite(filepath, frame)
            count += 1
            print(f"  [{count}] Saved: {filename}")

            # Wait for button release to avoid multiple captures per press
            button.wait_for_release()

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        camera.cleanup()
        print(f"Done. {count} images saved to {DATASET_DIR}")

if __name__ == "__main__":
    main()
