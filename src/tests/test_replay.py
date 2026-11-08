"""Smoke test for src/tools/replay.py: it runs end to end on images and, when one is
on this machine, a recorded run.

    python3 -m pytest tests/test_replay.py -v
  or:
    python3 -m tests.test_replay
"""

import glob
import os
import sys

from src.tools import replay

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_replay_images(capsys=None):
    session = sorted(glob.glob(os.path.join(REPO_ROOT, 'dataset', '*.jpg')))[0].rsplit('_', 1)[0]
    replay.main([session, '--dir', 'ccw'])


def test_replay_open_images():
    from src.obstacle_challenge import tuning
    from src.vision import pipeline
    session = sorted(glob.glob(os.path.join(REPO_ROOT, 'dataset', '*.jpg')))[0].rsplit('_', 1)[0]
    try:
        replay.main([session, '--open', '--summary'])
    finally:
        pipeline.configure(tuning)     # --open reconfigured the shared pipeline


def test_replay_recording():
    logs = [l for l in sorted(glob.glob(os.path.join(REPO_ROOT, 'obstacle', '*', 'raw.mp4.overlay.jsonl')))
            if os.path.getsize(l) > 300_000]
    if not logs:
        print("  (no recorded runs on this machine -- skipped)")
        return
    replay.main([os.path.dirname(logs[-1]), '--summary'])
    replay.main([logs[-1], '--frames', '0:30', '--changed'])


def main():
    failed = 0
    for t in (test_replay_images, test_replay_open_images, test_replay_recording):
        try:
            t()
            print(f"PASS  {t.__name__}")
        except Exception as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e!r}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
