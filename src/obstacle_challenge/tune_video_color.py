"""Launcher for Obstacle Challenge Video HSV Color Tuner.

Usage:
    python3 -m src.obstacle_challenge.tune_video_color
    python3 -m src.obstacle_challenge.tune_video_color path/to/video.mp4
"""

from src.tools.video_color_tuning import main

if __name__ == '__main__':
    main()
