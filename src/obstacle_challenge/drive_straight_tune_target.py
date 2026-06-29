import time
import cv2
import numpy as np
import threading
import sys
import os

from src.motors import motor, servo
from src.sensors import camera
from src.obstacle_challenge import main_v3

class CameraThread(threading.Thread):
    def __init__(self, camera_instance):
        super().__init__()
        self.camera = camera_instance
        self.latest_frame = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True

    def run(self):
        while not self.stop_event.is_set():
            frame = self.camera.capture_frame()
            with self.lock:
                self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

    def stop(self):
        self.stop_event.set()

def detect_biggest_block(frame):
    overlay_frame = frame.copy()
    
    # Use the same ROI as main_v3.full_frame_roi
    mx, my, mw, mh = main_v3.full_frame_roi
    crop = frame[my:my+mh, mx:mx+mw]
    
    blurred = cv2.GaussianBlur(crop, (1, 7), 0)
    hsv_crop = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    mask_red1 = cv2.inRange(hsv_crop, main_v3.LOWER_RED_1, main_v3.UPPER_RED_1)
    mask_red2 = cv2.inRange(hsv_crop, main_v3.LOWER_RED_2, main_v3.UPPER_RED_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv_crop, main_v3.LOWER_GREEN, main_v3.UPPER_GREEN)
    
    def get_biggest(mask, color_name):
        if cv2.countNonZero(mask) > 0:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                biggest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(biggest_contour)
                if area > main_v3.BLOCK_MIN_AREA:
                    M = cv2.moments(biggest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) + mx
                        cy = int(M["m01"] / M["m00"]) + my
                        contour_global = biggest_contour + [mx, my]
                        return {'color': color_name, 'area': area, 'centroid': (cx, cy), 'contour': contour_global}
        return None

    red_block = get_biggest(mask_red, 'red')
    green_block = get_biggest(mask_green, 'green')
    
    blocks = []
    if red_block: blocks.append(red_block)
    if green_block: blocks.append(green_block)
    
    # Draw ROI bounding box in blue
    cv2.rectangle(overlay_frame, (mx, my), (mx + mw, my + mh), (255, 0, 0), 1)
    
    if not blocks:
        return None, overlay_frame
        
    largest_block = max(blocks, key=lambda b: b['area'])
    
    draw_color = (0, 0, 255) if largest_block['color'] == 'red' else (0, 255, 0)
    cv2.drawContours(overlay_frame, [largest_block['contour']], -1, draw_color, 2)
    
    return largest_block, overlay_frame

def main():
    print("--- Starting tracking and driving test (No Gyro, Straight at 0 deg) ---")
    
    # Initialization
    if not motor.initialize():
        print("Motor init failed")
        return
    if not servo.initialize():
        print("Servo init failed")
        return
    if not camera.initialize():
        print("Camera init failed")
        return

    # Set servo to 0 to go straight
    servo.set_angle(0.0)

    camera_thread = CameraThread(camera)
    camera_thread.start()

    print("Waiting 2 seconds for camera to stabilize...")
    time.sleep(2)

    # Tracking Variables
    start_distance = motor.encoder.distance
    target_distance_mm = 500  # 50 cm = 500 mm
    
    out = None
    points = []
    
    # Motor parameters
    speed = 60
    start_time = time.time()
    motor.forward(speed)
    print(f"Driving forward at speed {speed} to reach {target_distance_mm/10} cm (or stop after 3 seconds)...")
    
    final_frame = None
    
    try:
        while True:
            # Time Tracking
            elapsed = time.time() - start_time
            print(f"Elapsed time: {elapsed:.2f}s", end="\r", flush=True)
            if elapsed >= 3.0:
                print(f"\nReached 3 seconds limit ({elapsed:.2f}s). Stopping.")
                break

            # Distance Tracking
            current_distance = abs(motor.encoder.distance - start_distance)
            if current_distance >= target_distance_mm:
                print(f"Reached target distance of {target_distance_mm/10} cm.")
                break
                
            # Video & Vision Processing
            frame = camera_thread.get_frame()
            if frame is not None:
                # Capture frame for later extrapolation
                final_frame = frame.copy()
                
                largest_block, overlay_frame = detect_biggest_block(frame)
                
                if out is None:
                    h, w = overlay_frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'avc1')
                    out = cv2.VideoWriter('output_tracking_video.mp4', fourcc, 20.0, (w, h))

                if largest_block is not None:
                    # Centroid of largest red or green block
                    cx, cy = largest_block['centroid']
                    points.append((cx, cy))
                    
                # Visualize Points
                for p in points:
                    cv2.circle(overlay_frame, p, 3, (0, 255, 255), -1)
                for i in range(len(points) - 1):
                    cv2.line(overlay_frame, points[i], points[i+1], (0, 255, 255), 2)
                    
                if out:
                    out.write(overlay_frame)
                    
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        motor.brake()
        time.sleep(0.2)
        servo.set_angle(0)
        
        # Extrapolate Line
        extrapolate_and_save(points, final_frame)
        
        # Cleanup Resources
        cleanup_all(camera_thread, out)

def extrapolate_and_save(points, frame):
    """Draw points and extrapolate the path via fitLine."""
    if len(points) >= 2 and frame is not None:
        print("Extrapolating path from tracked points...")
        _, overlay_frame = detect_biggest_block(frame)
        
        points_array = np.array(points, dtype=np.int32)
        
        # Draw past points again on final image
        for p in points:
            cv2.circle(overlay_frame, p, 3, (0, 255, 255), -1)
        for i in range(len(points) - 1):
            cv2.line(overlay_frame, points[i], points[i+1], (0, 255, 255), 2)
            
        # Fit Line Equation
        [vx, vy, x0, y0] = cv2.fitLine(points_array, cv2.DIST_L2, 0, 0.01, 0.01)
        vx = float(vx[0])
        vy = float(vy[0])
        x0 = float(x0[0])
        y0 = float(y0[0])
        
        h, w = overlay_frame.shape[:2]
        if vy != 0:
            x_top = int(x0 - (y0 / vy) * vx)
            x_bottom = int(x0 + ((h - y0) / vy) * vx)
            print(f"Top coordinate: ({x_top}, 0)")
            print(f"Bottom coordinate: ({x_bottom}, {h})")
        
        # Extend line indefinitely across image bounds
        pt1 = (int(x0 - 500*vx), int(y0 - 500*vy))
        pt2 = (int(x0 + 500*vx), int(y0 + 500*vy))
        
        # Draw transparent line
        line_overlay = overlay_frame.copy()
        cv2.line(line_overlay, pt1, pt2, (255, 0, 255), 5)
        cv2.addWeighted(line_overlay, 0.3, overlay_frame, 0.7, 0, overlay_frame)
        
        cv2.imwrite("final_extrapolation.png", overlay_frame)
        print("Extrapolated line successfully saved to 'final_extrapolation.png'")
    else:
        print("Not enough points to extrapolate a line on final frame.")

def cleanup_all(camera_thread, video_writer=None):
    """Safely shuts down everything."""
    camera_thread.stop()
    camera_thread.join(1)
    
    motor.cleanup()
    servo.cleanup()
    camera.cleanup()
    
    if video_writer is not None:
        video_writer.release()
        
    cv2.destroyAllWindows()
    print("Cleanup complete.")

if __name__ == "__main__":
    main()
