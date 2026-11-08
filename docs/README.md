# Documentation (`docs/`)

This folder holds the [Engineering Journal PDF](Engineering_Journal/Engineering_Journal.pdf), the [Documentation Summary PDF](Engineering_Journal/Greenbotics_Presentation.pdf) and the supporting material for the main [README](../README.md): diagrams, photos and experiment data.

## Folder Purpose Map

| Path | Contents | Referenced from (main README section) |
|---|---|---|
| `diagrams/` (top level) | Team logo, QR codes, video thumbnail, 2025 scoreboard and the 2025 vs 2026 comparison image. | Section 1 — Introduction |
| `diagrams/mobility/` | Diagrams, photos and GIFs for drivetrain, steering, differential, and speed/torque calculations. | Section 2 — Mobility and Mechanical Design |
| `diagrams/powerNsense/` | Diagrams and photos for power architecture, wiring, circuit, and sensor placement and calibration. `.drawio` source files sit next to their exported `.png`. | Section 3 — Power and Sensor Architecture |
| `diagrams/software/` | Diagrams, GIFs and test logs for software architecture, state machine, control loops and troubleshooting, with `.drawio` sources. The `pipeline/` subfolder holds the vision-pipeline stage images (raw frame → HSV → masks → annotated output). | Section 4 — Software Architecture and Obstacle Strategy |
| `diagrams/systemsThinking/` | Diagrams, photos and GIFs for system integration and failure-analysis case studies, with `.drawio` sources. | Section 5 — Systems Thinking and Engineering Decisions |
| `diagrams/gitNReproducibility/` | Photos and images for the build instructions. | Section 6 — Reproducibility and GitHub Organization |
| `experiments/` | Raw data from the encoder precision experiment (Excel file). | Section 2.10.2 |
| `Engineering_Journal/` | Printable PDF export of the main README and the documentation summary | Section 6.6 |
