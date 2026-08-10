# Documentation (`docs/`)

This folder holds the supporting diagrams referenced throughout the main [README](../README.md). It contains a single `diagrams/` subfolder, organized by subsystem, holding PNG/JPG/GIF images and their `.drawio` source files.

## Folder Purpose Map

| Path | Contents | Referenced from (main README section) |
|---|---|---|
| `diagrams/mobility/` | PNG/JPEG diagrams and photos for drivetrain, steering, differential, and speed/torque calculations. | Section 1 — Mobility & Mechanical Design |
| `diagrams/powerNsense/` | PNG/JPEG diagrams and photos for power architecture, wiring, circuit, and sensor placement/calibration. Includes `.drawio` source files alongside their exported `.png`. | Section 2 — Power & Sensor Architecture |
| `diagrams/software/` | PNG diagrams for software architecture, state machine, and control loops, with `.drawio` sources. Its `pipeline/` sub-subfolder holds vision-pipeline stage screenshots (raw frame → HSV → masks → annotated output). | Section 3 — Software Architecture & Obstacle Strategy |
| `diagrams/systemsThinking/` | PNG/JPEG/GIF diagrams and photographic evidence for system integration and failure-analysis case studies. | Section 4 — Systems Thinking & Engineering Decisions |

Each subfolder is a flat collection of images (and, where applicable, their `.drawio` source) for that subsystem. 
