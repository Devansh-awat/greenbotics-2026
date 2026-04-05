import time
import cv2
import numpy as np
import threading
import sys
import os

from src.motors import motor, servo
from src.sensors import bno055, camera
from src.obstacle_challenge import config

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

class ImuThread(threading.Thread):
    def __init__(self, bno):
        super().__init__()
        self.bno = bno
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.heading = None

    def run(self):
        try:
            self.bno.initialize()
            print("ImuThread: IMU initialized.")
            while not self.stop_event.is_set():
                heading = self.bno.get_heading()
                with self.lock:
                    self.heading = heading
        except Exception as e:
            print(f"ImuThread: ERROR: {e}")
        finally:
            self.bno.cleanup()

    def get_heading(self):
        with self.lock:
            return self.heading

    def stop(self):
        self.stop_event.set()

def steer_with_gyro(current_heading, target_heading, kp=0.85):
    error = target_heading - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    steer_angle = kp * error
    return np.clip(steer_angle, -45, 45)

def main():
    print("--- Starting tracking and driving test ---")
    
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

    imu_thread = ImuThread(bno055)
    imu_thread.start()
    camera_thread = CameraThread(camera)
    camera_thread.start()

    print("Waiting 2 seconds for sensors to stabilize...")
    time.sleep(2)

    target_heading = imu_thread.get_heading()
    if target_heading is None:
        print("Error: Failed to get initial heading from IMU")
        cleanup_all(imu_thread, camera_thread)
        return
        
    print(f"Target heading locked at: {target_heading:.2f} degrees")

    # Tracking Variables
    start_distance = motor.encoder.distance
    target_distance_mm = 500  # 50 cm = 500 mm
    
    out = None
    points = []
    
    # Motor parameters
    speed = 60
    motor.forward(speed)
    print(f"Driving forward at speed {speed} to reach {target_distance_mm/10} cm...")
    
    final_frame = None
    
    try:
        while True:
            # Distance Tracking
            current_distance = abs(motor.encoder.distance - start_distance)
            if current_distance >= target_distance_mm:
                print(f"Reached target distance of {target_distance_mm/10} cm.")
                break
                
            # Gyro Heading & Steering
            heading = imu_thread.get_heading()
            if heading is not None:
                steer_angle = steer_with_gyro(heading, target_heading)
                servo.set_angle(steer_angle)
                
            # Video & Vision Processing
            frame = camera_thread.get_frame()
            if frame is not None:
                # Capture frame for later extrapolation
                final_frame = frame.copy()
                
                # find_biggest_block expects full frame and returns cropped overlay
                largest_block, overlay_frame, _ = camera.find_biggest_block(frame)
                
                if out is None:
                    h, w = overlay_frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
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
        cleanup_all(imu_thread, camera_thread, out)

def extrapolate_and_save(points, frame):
    """Draw points and extrapolate the path via fitLine."""
    if len(points) >= 2 and frame is not None:
        print("Extrapolating path from tracked points...")
        _, overlay_frame, _ = camera.find_biggest_block(frame)
        
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

def cleanup_all(imu_thread, camera_thread, video_writer=None):
    """Safely shuts down everything."""
    imu_thread.stop()
    camera_thread.stop()
    
    imu_thread.join(1)
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
