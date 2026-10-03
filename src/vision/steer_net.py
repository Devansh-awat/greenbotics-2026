"""End-to-end steering net: camera frame -> servo angle (degrees).

Trained on raw.mp4 runs from obstacle/ (labels = the `Angle:` the controller
commanded on that frame). Normalisation lives inside the ONNX graph, so feed it
the untouched CameraThread frame (640x360 BGR uint8).

    net = SteerNet()
    angle = net(frame, clockwise=driving_direction == "clockwise")
"""
import os

import cv2
import numpy as np
import onnxruntime as ort

MODEL_PATH = os.path.join(os.path.dirname(__file__), "steer_net.onnx")
W, H = 256, 144  # must match training resize (cv2.INTER_AREA)


class SteerNet:
    def __init__(self, path=MODEL_PATH, threads=2):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(path, opts, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name

        self.has_dir = len(self.sess.get_inputs()) > 1  # v2 also takes driving direction

    def __call__(self, frame_bgr, clockwise=True):
        x = cv2.resize(frame_bgr, (W, H), interpolation=cv2.INTER_AREA)[None]
        feed = {self.inp: x}
        if self.has_dir:
            feed["direction"] = np.array([1.0 if clockwise else -1.0], np.float32)
        return float(self.sess.run(None, feed)[0][0])


if __name__ == "__main__":
    import sys
    import time

    net = SteerNet()
    cap = cv2.VideoCapture(sys.argv[1])  # e.g. obstacle/<run>/raw.mp4
    ok, f = cap.read()
    assert ok, "could not read video"
    t = time.perf_counter()
    for _ in range(50):
        a = net(f)
    print(f"angle={a:+.1f}  {(time.perf_counter() - t) / 50 * 1e3:.1f} ms/frame")
