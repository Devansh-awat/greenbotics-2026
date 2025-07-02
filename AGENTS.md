# AGENTS.md

Autonomous driving robot for WRO Future Engineers on Raspberry Pi 5 (RP1 I/O, Python 3).

## Directory Structure

```
├── src/
│   ├── obstacle_challenge/   # Obstacle challenge logic & state machine
│   │   ├── main.py           # Entry point: per-frame control loop & state machine
│   │   ├── control.py        # Gyro heading maths & straight driving primitives
│   │   ├── maneuvers.py      # Scripted sequences (start maneuver, parking 1 & 2)
│   │   ├── tuning.py         # Tunable constants, thresholds, ROIs, gains
│   │   ├── config.py         # Hardware pins, PWM channels, physical dimensions
│   │   └── video.py          # Background VideoEncoderProcess (avc1/mp4)
│   ├── open_challenge/       # Open challenge logic (wall following, laps)
│   │   ├── main.py           # Entry point: high-speed wall-following control loop
│   │   └── config.py         # Open challenge specific constants & thresholds
│   ├── vision/
│   │   ├── pipeline.py       # process_video_frame(): arena mask, HSV masks, contour detection
│   │   └── pool.py           # VisionPool: fork-join worker processes via /dev/shm
│   ├── motors/
│   │   ├── motor.py          # DC drive motor: PWM duty & closed-loop wheel RPM control
│   │   └── servo.py          # Steering servo: hardware PWM angle control (-45° to +40°)
│   ├── sensors/
│   │   ├── bno086.py         # BNO086 IMU (yaw/heading via I2C/SPI)
│   │   ├── camera.py         # IMX708 camera capture (Picamera2 / OpenCV)
│   │   ├── distance.py       # 4x VL53L4CD ToF distance sensors (I2C via XSHUT addressing)
│   │   ├── encoder.py        # RP1 PIO assembly quadrature encoder counter
│   │   └── i2c_bus.py        # Shared I2C bus initialization
│   ├── threads/
│   │   └── hw_threads.py     # CameraThread, ImuThread, SensorThread, PerfMonitor
│   ├── logs/
│   │   └── setup.py          # QueueHandler/QueueListener non-blocking logging (`robot.*`)
│   └── tools/                # Tuning & calibration utilities
│       ├── color_tuning.py   # Live HSV threshold tuner
│       ├── video_color_tuning.py # Video-based HSV tuner
│       ├── calibrate_bno.py  # IMU calibration script
│       ├── capture_dataset.py # Raw frame dataset capture
│       └── power_mode.py     # CPU performance governor configuration
├── tests/
│   └── test_vision_pipeline.py # Regression tests comparing vision outputs on dataset
└── dataset/                  # Sample arena camera frames (.jpg)
```

## Execution Commands

- **Obstacle Challenge**: `python3 -m src.obstacle_challenge.main`
- **Open Challenge**: `python3 -m src.open_challenge.main`
- **Run Tests**: `python3 -m pytest tests/test_vision_pipeline.py` or `python3 -m tests.test_vision_pipeline`
- **Color Tuning**: `python3 src/tools/color_tuning.py` or `python3 src/tools/video_color_tuning.py`

## Core Architecture & Conventions

1. **Sense → Decide → Act Loop**:
   - Hardware I/O runs in background threads (`hw_threads.py`), publishing non-blocking latest states.
   - Camera runs at ~56 FPS (IMX708 640x360). Main loop blocks on `CameraThread.get_next_frame()` condition variable.
   - Actuation (`servo.set_angle()`, `motor.set_speed()`) happens immediately after decision, before annotation/video encoding.
2. **Vision Pipeline** (`src/vision/pipeline.py`):
   - Generates arena mask and HSV masks (black walls, orange/blue lines, red/green/magenta blocks).
   - Optional 2-process fork-join acceleration via `VisionPool` using `/dev/shm` shared memory.
3. **Control & State Machine**:
   - Priority order: Avoid Head-on Wall > Pass Traffic Block (target-line angle law) > Corner Turn > Wall Follow (PD).
   - Heading control: Gyro heading error using shortest angular difference (`control.py`).
4. **Steering Convention**:
   - Steering angle clamped between `config.INPUT_ANGLE_MIN_SERVO` (-45°) and `config.INPUT_ANGLE_MAX_SERVO` (+40°). Positive is right, negative is left.
5. **Logging & Video Recording**:
   - Logging writes to non-blocking memory queue (`src/logs/setup.py`).
   - Video encoding runs in separate process (`src/obstacle_challenge/video.py`) to avoid GIL contention.
