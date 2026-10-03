"""Parking steering net: camera frame -> servo angle (degrees), for the camera line-tracking
stage of parking()/parking2().

Trained on parking.mp4 (angle read back off the "Servo angle:" overlay). Those frames
carry drawn telemetry, so the same fixed regions are blanked here before inference as
were blanked in training -- keep MASK in sync with the training notebook.

    net = ParkNet()
    angle = net(frame, clockwise=driving_direction == "clockwise")
"""
import os

import cv2
import numpy as np
import onnxruntime as ort

MODEL_PATH = os.path.join(os.path.dirname(__file__), "park_net.onnx")
W, H = 256, 144  # must match training resize (cv2.INTER_AREA)

# (y0, y1, x0, x1) in 640x360 camera pixels: overlay text, both 40 px parking ROIs
# (+ their outlines), and the bottom band with the magenta boxes / robot body.
MASK = [(0, 160, 0, 345), (135, 360, 595, 640), (135, 360, 0, 45), (325, 360, 0, 640)]


def apply_mask(frame_bgr):
    f = frame_bgr.copy()
    for y0, y1, x0, x1 in MASK:
        f[y0:y1, x0:x1] = 0
    return f


class ParkNet:
    def __init__(self, path=MODEL_PATH, threads=4):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(path, opts, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name

    def __call__(self, frame_bgr, clockwise=True):
        x = cv2.resize(apply_mask(frame_bgr), (W, H), interpolation=cv2.INTER_AREA)[None]
        d = np.array([1.0 if clockwise else -1.0], np.float32)
        return float(self.sess.run(None, {self.inp: x, "direction": d})[0][0])


if __name__ == "__main__":
    import sys
    import time

    net = ParkNet()
    cap = cv2.VideoCapture(sys.argv[1])  # e.g. obstacle/<run>/parking.mp4
    ok, f = cap.read()
    assert ok, "could not read video"
    t = time.perf_counter()
    for _ in range(50):
        a = net(f)
    print(f"angle={a:+.1f}  {(time.perf_counter() - t) / 50 * 1e3:.1f} ms/frame")
