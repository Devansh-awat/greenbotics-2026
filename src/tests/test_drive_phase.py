"""Runs the REAL drive_phase() of both challenges on dataset frames, with the camera,
motor, servo, IMU and video recorder faked. Catches mistakes in the loop plumbing
(recording, annotation, logging, stop_when) that the rules tests do not reach.

    python3 -m pytest tests/test_drive_phase.py -v
  or:
    python3 -m tests.test_drive_phase
"""

import glob
import os
import sys
import types

import cv2
import numpy as np

from src.obstacle_challenge import main as obstacle_main
from src.obstacle_challenge import rules as obstacle_rules
from src.obstacle_challenge import tuning as T
from src.open_challenge import config as OC
from src.open_challenge import main as open_main
from src.open_challenge import rules as open_rules
from src.vision import pipeline as P

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _frames(limit=120):
    out = []
    for p in sorted(glob.glob(os.path.join(REPO_ROOT, 'dataset', '*.jpg')))[:limit * 2]:
        f = cv2.imread(p)
        if f is not None and f.shape == (360, 640, 3):
            out.append(f)
        if len(out) >= limit:
            break
    assert out
    return out


class FakeCamera:
    def __init__(self, frames):
        self.frames, self.i = frames, 0

    def get_next_frame(self, past):
        if self.i >= len(self.frames):
            raise RuntimeError("ran out of frames: stop_when never fired")
        f = self.frames[self.i]
        self.i += 1
        return f, self.i, 0.0


class FakeEncoder:
    """VideoEncoderProcess stand-in: runs prepare() like the real one would."""
    def __init__(self):
        self.written, self.tags = 0, []

    def write(self, frame, tag=None, prepare=None):
        img = prepare(frame) if prepare is not None else frame
        assert img.shape == frame.shape
        self.written += 1
        self.tags.append(tag)
        return True


class FakeMotor:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        return lambda *a, **k: self.calls.append(name)

    def get_measured_rpm(self):
        return 123.0

    def encoder_position(self):
        return 0


class Null:
    def __getattr__(self, name):
        return lambda *a, **k: None

    def __bool__(self):
        return True     # throttles fire every frame -> every log line is exercised


class _patched:
    def __init__(self, module, **attrs):
        self.module, self.attrs, self.saved = module, attrs, {}

    def __enter__(self):
        for k, v in self.attrs.items():
            self.saved[k] = getattr(self.module, k)
            setattr(self.module, k, v)

    def __exit__(self, *exc):
        for k, v in self.saved.items():
            setattr(self.module, k, v)


def test_obstacle_drive_phase():
    P.vision_pool = None
    frames = _frames()
    for record_raw in (False, True):
        enc, overlay, motor = FakeEncoder(), [], FakeMotor()
        overlay_log = types.SimpleNamespace(append=lambda *a, **k: overlay.append(a))
        mem = obstacle_rules.Memory()
        with _patched(obstacle_main, motor=motor, servo=Null(), Throttle=lambda *a: Null(),
                      time=types.SimpleNamespace(monotonic=lambda: 10.0, perf_counter=lambda: 0.0,
                                                 sleep=lambda s: None)):
            result = obstacle_main.drive_phase(
                FakeCamera(frames), enc, overlay_log, record_raw, Null(), 'clockwise', 0.0,
                mem, stop_when=lambda world, mem: 'done' if enc.written >= 100 else None)
        assert result == 'done'
        assert enc.written == 100
        assert 'start_rpm_control' in motor.calls
        if record_raw:
            assert len(overlay) == 100 and any('Rule:' in d for d in overlay[0][2])


def test_open_drive_phase():
    P.vision_pool = None
    P.configure(OC)
    try:
        frames = _frames()
        enc, motor = FakeEncoder(), FakeMotor()
        imu = types.SimpleNamespace(get_vibration=lambda period: {
            "total_deg_s": 1.0, "pitch_roll_deg_s": 1.0, "axes_deg_s": (1.0, 1.0),
            "yaw_jitter_deg_s": 1.0, "peak_deg_s": 2.0, "samples": 10})
        mem = open_rules.Memory()
        with _patched(open_main, motor=motor, servo=Null(), Throttle=lambda *a: Null()):
            ctx = open_main.DriveContext(FakeCamera(frames), imu, enc, Null(), 0.0)
            result = open_main.drive_phase(
                ctx, mem, stop_when=lambda world, mem: 'final' if enc.written >= 100 else None)
        assert result == 'final'
        assert enc.written == 100
        assert ctx.latest_vibration_deg_s == 1.0
        # Memory carries over: a second phase continues from the same state.
        turns = mem.turns
        with _patched(open_main, motor=motor, servo=Null(), Throttle=lambda *a: Null()):
            open_main.drive_phase(ctx, mem, stop_when=lambda world, mem: 'x' if enc.written >= 110 else None)
        assert mem.turns >= turns
    finally:
        P.configure(T)


def main():
    failed = 0
    for t in (test_obstacle_drive_phase, test_open_drive_phase):
        try:
            t()
            print(f"PASS  {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e!r}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
