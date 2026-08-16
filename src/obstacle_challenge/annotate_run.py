"""Launcher for the offline run annotator.

Rebuilds the annotated obstacle.mp4 from the raw.mp4 recorded by `main.py -v`.

Usage:
    python3 -m src.obstacle_challenge.annotate_run
    python3 -m src.obstacle_challenge.annotate_run obstacle/<timestamp>
    python3 -m src.obstacle_challenge.annotate_run path/to/raw.mp4
"""

from src.tools.annotate_run import main

if __name__ == '__main__':
    main()
