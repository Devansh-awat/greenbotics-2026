# Models (`models/`)

This folder holds the CAD/PCB source files for the robot's 3D-printed chassis parts and the custom PCB.

## Nested folders (contents summarized here per project convention — no separate README inside)

**`chassis/`** — 3D-printable STL files for the chassis:
- `BearingToWheelShaft.stl`
- `BottomChasis.stl`
- `CameraMount.stl`
- `DifferentialMountTop.stl`
- `MotorShaft.stl`
- `ServoMount.stl`
- `TopChasis.stl`
- `TPUBumper.stl`
- `VariableServoHorn.stl`
- `WheelStabilizer.stl`

**`PCB/`** — KiCad project for the custom PCB (single, current design — `HAT+`):
- `HAT+.kicad_pcb` — PCB layout
- `HAT+.kicad_sch` — schematic
- `HAT+.kicad_pro` — project file
- `HAT+.kicad_prl` — project local settings
- `HAT+.kicad_dru` — design rules
- `HAT_Plus_sch.kicad_sch` — schematic (supporting sheet)
- `fp-lib-table`, `fp-info-cache` — footprint library references/cache
- `sym-lib-table` — symbol library reference
