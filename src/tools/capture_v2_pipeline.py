import os
import cv2
import numpy as np
import time
from src.sensors import camera
from src.obstacle_challenge import main_v2

def run_pipeline_capture():
    artifact_dir = "/home/devansh/.gemini/antigravity-ide/brain/d8002521-c525-4827-83f9-7858d19db3f9/pipeline_images"
    local_dir = "/home/devansh/greenbotics-1/pipeline_images"
    
    os.makedirs(artifact_dir, exist_ok=True)
    os.makedirs(local_dir, exist_ok=True)

    print("Initializing camera...")
    if not camera.initialize():
        print("ERROR: Failed to initialize camera!")
        return

    try:
        time.sleep(1.0)
        print("Capturing frame...")
        frame = camera.capture_frame()
        print(f"Captured frame shape: {frame.shape}")

        if frame is None:
            print("ERROR: Captured frame is None!")
            return

        # Frame as captured from Picamera2 (main_v2.py uses this directly)
        bgr_frame = frame.copy()

        # Extract all parameters and ROIs from main_v2
        left_roi_x, left_roi_y, left_roi_w, left_roi_h = main_v2.left_roi_x, main_v2.left_roi_y, main_v2.left_roi_w, main_v2.left_roi_h
        right_roi_x, right_roi_y, right_roi_w, right_roi_h = main_v2.right_roi_x, main_v2.right_roi_y, main_v2.right_roi_w, main_v2.right_roi_h
        inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = main_v2.inner_left_roi_x, main_v2.inner_left_roi_y, main_v2.inner_left_roi_w, main_v2.inner_left_roi_h
        inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = main_v2.inner_right_roi_x, main_v2.inner_right_roi_y, main_v2.inner_right_roi_w, main_v2.inner_right_roi_h
        line_roi_x, line_roi_y, line_roi_w, line_roi_h = main_v2.line_roi_x, main_v2.line_roi_y, main_v2.line_roi_w, main_v2.line_roi_h
        close_x, close_y, close_w, close_h = main_v2.close_x, main_v2.close_y, main_v2.close_w, main_v2.close_h
        full_frame_roi = main_v2.full_frame_roi
        close_block_roi = main_v2.close_block_roi

        # 1. Save raw BGR frame
        images_to_save = {}
        images_to_save["01_raw_frame.png"] = bgr_frame

        # 2. ROI Overlay Frame
        roi_overlay = bgr_frame.copy()
        rois = [
            ("Left Wall ROI", (left_roi_x, left_roi_y, left_roi_w, left_roi_h), (255, 255, 0)),
            ("Right Wall ROI", (right_roi_x, right_roi_y, right_roi_w, right_roi_h), (255, 255, 0)),
            ("Inner L Wall ROI", (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), (255, 255, 0)),
            ("Inner R Wall ROI", (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), (255, 255, 0)),
            ("Line ROI", (line_roi_x, line_roi_y, line_roi_w, line_roi_h), (0, 165, 255)),
            ("Close Black ROI", (close_x, close_y, close_w, close_h), (0, 0, 255)),
            ("Main Blocks ROI", full_frame_roi, (0, 255, 0)),
            ("Close Blocks ROI", close_block_roi, (255, 0, 255))
        ]
        for label, (x, y, w, h), color in rois:
            cv2.rectangle(roi_overlay, (x, y), (x + w, y + h), color, 2)
            cv2.putText(roi_overlay, label, (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        images_to_save["02_roi_overlay.png"] = roi_overlay

        # 3. Global Slicing
        GLOBAL_Y_OFFSET = min(
            left_roi_y, right_roi_y, inner_left_roi_y, inner_right_roi_y,
            line_roi_y, close_y, full_frame_roi[1], close_block_roi[1]
        )
        GLOBAL_Y_END = max(
            left_roi_y + left_roi_h, right_roi_y + right_roi_h,
            inner_left_roi_y + inner_left_roi_h, inner_right_roi_y + inner_right_roi_h,
            line_roi_y + line_roi_h, close_y + close_h,
            full_frame_roi[1] + full_frame_roi[3], close_block_roi[1] + close_block_roi[3]
        )
        SLICE_HEIGHT = GLOBAL_Y_END - GLOBAL_Y_OFFSET

        frame_slice = bgr_frame[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
        blurred_slice = cv2.GaussianBlur(frame_slice, (1, 7), 0)
        images_to_save["03_blurred_frame_slice.png"] = blurred_slice

        if main_v2.USE_LAB:
            hsv_slice = cv2.cvtColor(blurred_slice, cv2.COLOR_BGR2Lab)
        else:
            hsv_slice = cv2.cvtColor(blurred_slice, cv2.COLOR_BGR2HSV)

        # Main Blocks ROI crops & masks
        mx, my, mw, mh = full_frame_roi
        my_slice = max(0, my - GLOBAL_Y_OFFSET)
        main_crop = hsv_slice[my_slice:my_slice+mh, mx:mx+mw]

        mask_red1_main = cv2.inRange(main_crop, main_v2.LOWER_RED_1, main_v2.UPPER_RED_1)
        if main_v2.USE_LAB:
            mask_red_main = mask_red1_main
        else:
            mask_red2_main = cv2.inRange(main_crop, main_v2.LOWER_RED_2, main_v2.UPPER_RED_2)
            mask_red_main = cv2.bitwise_or(mask_red1_main, mask_red2_main)
        mask_green_main = cv2.inRange(main_crop, main_v2.LOWER_GREEN, main_v2.UPPER_GREEN)
        mask_magenta_main = cv2.inRange(main_crop, main_v2.LOWER_MAGENTA, main_v2.UPPER_MAGENTA)

        images_to_save["04_mask_red_main.png"] = mask_red_main
        images_to_save["05_mask_green_main.png"] = mask_green_main
        images_to_save["06_mask_magenta_main.png"] = mask_magenta_main

        # Line ROI crops & masks
        lx, ly, lw, lh = line_roi_x, line_roi_y, line_roi_w, line_roi_h
        ly_slice = ly - GLOBAL_Y_OFFSET
        line_crop = hsv_slice[ly_slice:ly_slice+lh, lx:lx+lw]
        mask_orange_line = cv2.inRange(line_crop, main_v2.LOWER_ORANGE, main_v2.UPPER_ORANGE)
        mask_blue_line = cv2.inRange(line_crop, main_v2.LOWER_BLUE, main_v2.UPPER_BLUE)

        images_to_save["07_mask_orange_line.png"] = mask_orange_line
        images_to_save["08_mask_blue_line.png"] = mask_blue_line

        # Close Blocks ROI crops & masks
        cx, cy, cw, ch = close_block_roi
        cy_slice = cy - GLOBAL_Y_OFFSET
        close_crop = hsv_slice[cy_slice:cy_slice+ch, cx:cx+cw]
        mask_red1_close = cv2.inRange(close_crop, main_v2.LOWER_RED_1, main_v2.UPPER_RED_1)
        if main_v2.USE_LAB:
            mask_red_close = mask_red1_close
        else:
            mask_red2_close = cv2.inRange(close_crop, main_v2.LOWER_RED_2, main_v2.UPPER_RED_2)
            mask_red_close = cv2.bitwise_or(mask_red1_close, mask_red2_close)
        mask_green_close = cv2.inRange(close_crop, main_v2.LOWER_GREEN, main_v2.UPPER_GREEN)
        mask_magenta_close = cv2.inRange(close_crop, main_v2.LOWER_MAGENTA, main_v2.UPPER_MAGENTA)

        images_to_save["09_mask_red_close.png"] = mask_red_close
        images_to_save["10_mask_green_close.png"] = mask_green_close
        images_to_save["11_mask_magenta_close.png"] = mask_magenta_close

        # Global Reconstructed Masks (Slice Size 640xSLICE_HEIGHT)
        global_red_mask = np.zeros((SLICE_HEIGHT, main_v2.FRAME_WIDTH), dtype="uint8")
        global_green_mask = np.zeros((SLICE_HEIGHT, main_v2.FRAME_WIDTH), dtype="uint8")
        global_blue_mask = np.zeros((SLICE_HEIGHT, main_v2.FRAME_WIDTH), dtype="uint8")
        global_magenta_mask = np.zeros((SLICE_HEIGHT, main_v2.FRAME_WIDTH), dtype="uint8")

        global_red_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_red_mask[my_slice:my_slice+mh, mx:mx+mw], mask_red_main)
        global_green_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_green_mask[my_slice:my_slice+mh, mx:mx+mw], mask_green_main)
        global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw], mask_magenta_main)

        global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_red_close)
        global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_green_close)
        global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_magenta_close)

        global_blue_mask[ly_slice:ly_slice+lh, lx:lx+lw] = cv2.bitwise_or(global_blue_mask[ly_slice:ly_slice+lh, lx:lx+lw], mask_blue_line)

        images_to_save["12_global_red_mask.png"] = global_red_mask
        images_to_save["13_global_green_mask.png"] = global_green_mask
        images_to_save["14_global_blue_mask.png"] = global_blue_mask
        images_to_save["15_global_magenta_mask.png"] = global_magenta_mask

        # Wall & Black Detection
        mask_black = cv2.inRange(hsv_slice, main_v2.LOWER_BLACK, main_v2.UPPER_BLACK)
        mask_red_or_green = cv2.bitwise_or(global_red_mask, global_green_mask)
        mask_red_or_green_or_blue = cv2.bitwise_or(mask_red_or_green, global_blue_mask)

        pure_black_mask = cv2.bitwise_and(mask_black, cv2.bitwise_not(mask_red_or_green_or_blue))
        black_or_magenta_mask = cv2.bitwise_or(pure_black_mask, global_magenta_mask)

        images_to_save["16_mask_black_raw.png"] = mask_black
        images_to_save["17_pure_black_mask.png"] = pure_black_mask
        images_to_save["18_black_or_magenta_mask.png"] = black_or_magenta_mask

        # Sliced ROI Masks
        roi_mask_walls_slice = main_v2.roi_mask_walls[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
        roi_mask_close_black_slice = main_v2.roi_mask_close_black[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]

        final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls_slice)
        final_mask_close_black = cv2.bitwise_and(black_or_magenta_mask, roi_mask_close_black_slice)

        images_to_save["19_roi_mask_walls_slice.png"] = roi_mask_walls_slice
        images_to_save["20_final_mask_walls.png"] = final_mask_walls
        images_to_save["21_roi_mask_close_black_slice.png"] = roi_mask_close_black_slice
        images_to_save["22_final_mask_close_black.png"] = final_mask_close_black

        # Run process_video_frame and annotate_video_frame
        detections = main_v2.process_video_frame(bgr_frame)
        annotated_frame = main_v2.annotate_video_frame(bgr_frame, detections, "counterclockwise", debug_info="Pipeline Capture")
        images_to_save["23_annotated_frame.png"] = annotated_frame

        print("Saving pipeline images...")
        for filename, img in images_to_save.items():
            p1 = os.path.join(local_dir, filename)
            p2 = os.path.join(artifact_dir, filename)
            cv2.imwrite(p1, img)
            cv2.imwrite(p2, img)
            print(f"Saved: {filename}")

        print("Pipeline capture complete!")
    finally:
        camera.cleanup()

if __name__ == "__main__":
    run_pipeline_capture()
