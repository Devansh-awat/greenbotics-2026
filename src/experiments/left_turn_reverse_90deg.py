#!/usr/bin/env python3
"""
left_turn_reverse_90deg.py

Hardware experiment: same as left_turn_forward_90deg.py (steer to -50 deg, 80 wheel rpm,
stop after a 90 degree heading change, brake, straighten), but driving in reverse.

Run from the repo root:  python3 -m src.experiments.left_turn_reverse_90deg
"""

import sys

from src.experiments.left_turn_forward_90deg import run_experiment

if __name__ == "__main__":
    sys.exit(run_experiment("reverse"))
