# CLAUDE.md

Guidance for working in this repo. This is the **Greenbotics 2025 WRO Future Engineers** robot — an
autonomous self-driving car built on a Raspberry Pi 5, competing in two challenges (Open + Obstacle).

> ⚠️ **All `README.md` files in this repo (top-level and `src/README.md`) are outdated — do not trust
> them.** They may describe hardware, wiring, dependencies, or behavior that no longer matches the
> code. Treat the source code as the only source of truth. This file documents the **software** as it
> currently is; if something here conflicts with a README, this file (and the code) wins.

## Hardware reality (read before changing anything)

This code runs **on the robot's Raspberry Pi 5** and talks to real hardware. It will not run on a dev
machine — it imports `picamera2`, `lgpio`, `rpi-hardware-pwm`, `board`/`busio` (Blinka), and `gpiozero`.
Don't try to "just run" the mains locally; reason about them statically or run on the Pi.

Physical setup:
- **Drive:** LEGO EV3 medium motor via a TB6612FNG driver (DC motor mode), hardware PWM on GPIO19
  (chip 0, channel 3, 10 kHz). Direction pins AIN1=26, AIN2=13, STBY=6.
- **Steering:** SG90 servo, hardware PWM on GPIO18 (chip 0, channel 2, 50 Hz). Input angle range
  ±45° maps to calibrated pulse widths; see `SERVO_*` / `CALIBRATED_*` constants in the configs.
- **Wheel encoder:** quadrature encoder read by a **PIO program** on `board.D20` (see `encoder.py`).
  540.18 counts/wheel rev, ~455 max wheel rpm, ~40 rpm stall floor.
- **Camera:** Raspberry Pi camera via Picamera2, 640×480 (obstacle) / 640×360 (open), low exposure
  (~3 ms) to avoid motion blur. OpenCV for processing.
- **IMU:** BNO055 (Adafruit CircuitPython driver) for heading/gyro.
- **Distance (ToF):** VL53L1X (front, on TCA9548A I2C mux channel 0) + VL53L8CX (back, on **SPI**).
- **Start button** on GPIO23, status LED.

## Layout

```
src/
  vision/               Shared vision pipeline -- used by BOTH challenges
    pipeline.py         arena mask, colour masks, process_video_frame, annotation
    pool.py             the two vision worker processes + their shared memory (VisionPool)
  threads/              Shared background threads -- used by BOTH challenges
    hw_threads.py       CameraThread / ImuThread / SensorThread / PerfMonitor
  logs/                 Shared logging -- used by BOTH challenges
    setup.py            the `robot.*` logger tree, Throttle, non-blocking log queue
  open_challenge/       Open Challenge (3 laps, wall following)
    main.py             CURRENT. Same vision/threads/logging as the obstacle challenge;
                         only the driving logic differs (no pillars, no parking).
    config.py           legacy standalone config; no longer imported by main.py
  obstacle_challenge/   Obstacle Challenge (red/green pillars + parking)
    main.py             CURRENT. Run setup + the per-frame control loop, nothing else.
    config.py           pins, PWM/servo calibration -- also imported by src/motors, src/sensors/camera
    tuning.py           every constant: colour ranges, ROIs, gains, perf switches
    video.py            the annotated-run recorder, in its own process
    control.py          heading maths + gyro-stabilised drive primitives
    maneuvers.py        scripted sequences: initial maneuver, parking, parking2
    nationals/          older snapshot of the obstacle code (main.py/utils.py) — reference only
  sensors/              drivers: bno055, camera, distance, encoder, vl53l1x, vl53l8cx_python, i2c_bus
  motors/               motor.py (drive), servo.py (steering)
  teleop/               server.py — phone-based manual driving over WebSocket (no sensors)
```

Vision, threads and logging live outside both challenge packages because the code is
shared verbatim: both mains import `src.vision.pipeline`, `src.vision.pool.VisionPool`,
`src.threads.hw_threads`, and `src.logs.setup`. `process_video_frame()` also detects
pillars/parking (magenta) objects that don't exist on the open track — the open loop
just never reads those keys. Tuning constants (`obstacle_challenge/tuning.py`) and
hardware pin config (`obstacle_challenge/config.py`) stay inside `obstacle_challenge/`
since they're control tuning, not vision code — but both are imported by the open
challenge too, and `config.py` is also the canonical pin config for `src/motors` and
`src/sensors/camera.py`.

`test_*.py`, `sensor_test.py`, `capture_dataset.py`, `color_tuning.py`, `color_annotate_tuner.py`
are standalone diagnostics/tuning utilities, not part of the competition runs.

## Running

Always run as a **module from the repo root** (the code uses `src.` package imports):

```bash
python3 -m src.obstacle_challenge.main   # Obstacle Challenge
python3 -m src.open_challenge.main       # Open Challenge
python3 -m src.teleop.server             # manual phone teleop -> http://<pi-ip>:8000
```

(`src/README.md` lists dependencies but is outdated — verify against the actual imports in the code.)

## How the challenge code works

Both mains follow the same architecture: a set of **daemon threads** feed the main control loop.

- `CameraThread` — continuously captures frames; main loop pulls the latest via `get_next_frame()`.
- `ImuThread` — initializes the BNO055 and continuously updates `heading`.
- `SensorThread` (obstacle) — polls the ToF sensors so the loop never blocks on I2C/SPI.
- `VideoWriterThread` / `AnnotateAndWriteThread` / `DatasetCaptureThread` (obstacle) — record runs and
  dump training frames to `dataset/`. Off the hot path.

The main loop, each frame:
1. `process_video_frame(frame)` — HSV (default) color thresholding inside fixed **ROI rectangles** to
   detect walls (black), the lap-counting line (orange/blue), and pillars (red/green/magenta).
2. Decide a steering `angle` and `speed` from detections + ToF + heading.
3. `steer_with_gyro` / `drive_straight_with_gyro` apply the command with a heading-lock P controller.

Key behaviors:
- **Lap counting:** crossing the orange (and blue) floor line increments `turn_counter`, gated by a
  cooldown (`ORANGE_COOLDOWN_FRAMES`) so one line isn't counted twice. Run ends after `TOTAL_TURNS`.
- **Obstacle pillars:** red pillars are passed on the **left**, green on the **right**. The code steers
  to a target angle relative to the detected block, with special handling for "close blocks" that fill
  the frame, plus recovery latches (see the long comments around the inner-corner recovery in
  `main_v3.py`) for over-rotation cases.
- **Parking** (obstacle): `parking*()` functions handle the start/end parking maneuvers using ToF +
  encoder moves.
- **Direction:** `driving_direction` (`'clockwise'` / `'counter-clockwise'`) mirrors most of the logic.

## Config / tuning

- `*/config.py` holds pins, PWM/servo calibration, frame sizes, color HSV ranges, ROI rectangles, and
  control gains. **For v5 every tunable lives in `obstacle_challenge/tuning.py`** (HSV/LAB ranges,
  ROIs, `MOTOR_SPEED`, areas, perf switches) — that is the only file to edit when tuning the
  obstacle run. `main_v3.py` still overrides them inline at the top of its own file.
- Color thresholding can run in **HSV or LAB** (`USE_LAB` flag in `main_v3.py`); both range tables are
  kept. HSV is the default.
- Color ranges were tuned from saved samples (`sensors/color_samples/*.npy`) using the tuner scripts.

## Driver APIs (what to call)

- `motor.initialize()`, `motor.forward(speed)` / `reverse(speed)` (0–100 duty), `brake()`, `standby()`,
  `move(distance_cm, ...)` (encoder PID positioning), `drive_distance(...)`, RPM control
  (`start_rpm_control`/`set_rpm_target`/`get_measured_rpm`/`stop_rpm_control`), `cleanup()`.
- `servo.initialize()`, `servo.set_angle(deg)` (clipped to ±45 safe range), `set_angle_unlimited(deg)`,
  `cleanup()`.
- `bno055.initialize()`, `get_heading()`, `get_initial_heading()`, `cleanup()`.
- `distance.initialise()`, `get_distance(channel)` (channels: `FRONT_CHANNEL`=0, `BACK_CHANNEL`=3),
  `reinit_sensor(channel)`, `get_diag()`, `cleanup()`.
- `camera.initialize()`, `capture_frame()`, `find_objects_in_rois(...)`, `find_biggest_block(...)`,
  `cleanup()`.

## Conventions / gotchas

- Heading math: use `get_angular_difference(a, b)` for wraparound-safe angle deltas — it lives in
  `obstacle_challenge/control.py`; the open challenge imports it from there too.
- Prefer encoder-closed moves over duty + stopwatch: `control.drive_distance_with_gyro()` or
  `motor.move()`/`motor.drive_distance()`. Remaining open-loop sites are marked `BUG:` in
  `maneuvers.py` and `main_v5.py`.
- Servo positive vs negative angle = which way the wheels turn; `SERVO_CENTER_OFFSET` trims center.
- Always `cleanup()` motors/servo/sensors on exit — the mains wrap the loop in try/finally for this.
- This repo runs hardware. Don't add code that assumes a screen, and keep heavy work off the control
  loop (use threads as the existing code does).
