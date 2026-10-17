from picamera2 import Picamera2
import cv2


picam2 = None

def initialize():
    """Initializes the Picamera2."""
    global picam2
    try:
        picam2 = Picamera2()
        cam_config = picam2.create_preview_configuration(
            # 640x360 is produced directly by the ISP. The `raw` size pins the
            # sensor to mode 1 (2304x1296, 2x2 binned) -- the only mode that is
            # both uncropped (full FoV) and capable of 56fps. Previously this was
            # done by requesting a 2304x1296 `main` and reading a `lores`; that
            # selected the same mode but burned ~2.5% of a core and ~500MB/s of
            # memory bandwidth producing a full-res stream nobody read.
            main={"format": "RGB888", "size": (640, 360)},
            raw={"size": (2304, 1296)},
            controls={
                        "ExposureTime": 3000,
                        #"AnalogueGain": 10000.0,
                        "FrameRate": 56,
                    }
        )
        picam2.configure(cam_config)
        picam2.start()
        #picam2.set_controls(controls={'ExposureTime': 20000, 'AnalogueGain': 1})
        print("INFO: Camera Initialized.")
        return True
    except Exception as e:
        print(f"FATAL: Camera failed to initialize: {e}")
        return False

def capture_frame():
    """Captures and returns a single frame from the camera."""
    if picam2:
        frame = picam2.capture_array("main")
        #frame = cv2.rotate(frame, cv2.ROTATE_180)
        #if frame.shape[2] == 4:
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame
    return None

def cleanup():
    """Stops the camera."""
    print("--- Cleaning up Camera ---")
    if picam2:
        picam2.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    from src.obstacle_challenge import tuning

    print("--- Testing Camera ROIs (from tuning, no detection) ---")
    if not initialize():
        print("Camera test failed during initialization.")
    else:
        cv2.namedWindow("Camera Test")

        light_blue = (255, 255, 0)
        all_rois = [
            (tuning.left_roi_x, tuning.left_roi_y, tuning.left_roi_w, tuning.left_roi_h),
            (tuning.right_roi_x, tuning.right_roi_y, tuning.right_roi_w, tuning.right_roi_h),
            (tuning.inner_left_roi_x, tuning.inner_left_roi_y, tuning.inner_left_roi_w, tuning.inner_left_roi_h),
            (tuning.inner_right_roi_x, tuning.inner_right_roi_y, tuning.inner_right_roi_w, tuning.inner_right_roi_h),
            tuning.full_frame_roi,
            tuning.close_block_roi,
            (tuning.line_roi_x, tuning.line_roi_y, tuning.line_roi_w, tuning.line_roi_h),
            (tuning.close_x, tuning.close_y, tuning.close_w, tuning.close_h),
        ]

        try:
            while True:
                frame = capture_frame()
                for x, y, w, h in all_rois:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), light_blue, 2)
                cv2.line(frame,(242,277),(222,355),(255,255,255),2)
                cv2.line(frame,(242,277),(369,277),(255,255,255),2)
                cv2.imshow("Camera Test", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
