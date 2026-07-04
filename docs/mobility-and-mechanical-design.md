# Mobility and Mechanical Design — WRO 2026 Future Engineers

Team **Greenbotics** · Vehicle codename **v2** · CAD: `WRO2026Chasisv1.1.f3d`

This document is the mechanical companion to [`software-architecture-and-obstacle-strategy.md`](software-architecture-and-obstacle-strategy.md). The software doc covers everything from the camera down to the servo PWM byte; this doc covers everything from the servo PWM byte down to the rubber–mat contact patch. Whenever a number is still pending a bench measurement it is marked **`[PENDING]`** with a pointer to §13 ("Tests & experiments still needed"), where the procedure to capture it lives.

> **Scope note.** The robot is still under active construction at the time of writing. Components and dimensions described as "current" reflect the v2 CAD model as committed; **`[PENDING]`** values get filled in as bench measurements come in. The document is written as though v2 is final so a reader can see the intended end-state, with the open items collected in §13.

---

## Table of contents

1. [Design goals & headline specs](#1-design-goals--headline-specs)
2. [Powertrain — required-spec derivation](#2-powertrain--required-spec-derivation)
3. [Powertrain — chosen motor & alternatives](#3-powertrain--chosen-motor--alternatives)
4. [Drivetrain — gear ratio, differential, axle](#4-drivetrain--gear-ratio-differential-axle)
5. [Wheels & tyres](#5-wheels--tyres)
6. [Steering — Ackermann geometry & servo](#6-steering--ackermann-geometry--servo)
7. [Chassis — two-plate architecture](#7-chassis--two-plate-architecture)
8. [Materials, print settings, and 3MF reproducibility](#8-materials-print-settings-and-3mf-reproducibility)
9. [Mass & centre of gravity](#9-mass--centre-of-gravity)
10. [Design iterations — v1 → v2 → v3 → v4](#10-design-iterations--v1--v2--v3--v4)
11. [Why theory didn't match reality](#11-why-theory-didnt-match-reality)
12. [Problems faced & innovative solutions](#12-problems-faced--innovative-solutions)
13. [Tests & experiments still needed](#13-tests--experiments-still-needed)
14. [Assembly instructions](#14-assembly-instructions)
15. [CAD render gallery](#15-cad-render-gallery)

---

## 1. Design goals & headline specs

The robot is designed around a small set of quantitative goals. The single hard external constraint is the rule book — WRO 2026 Future Engineers rule **11.1** caps the vehicle at **300 × 200 × 300 mm** and rule **11.2** caps it at **1.5 kg** — and everything else is chosen to finish the three-lap run quickly while leaving the software headroom (see software-doc §1).

| Goal | Target | Driver |
|---|---|---|
| Finish time (3 laps) | **≈ 30 s** (Open Challenge)<br>**≈ 45 s** (Obstacle Challenge) | The rules allow up to 3 minutes (rule 9.1/9.2); self-imposed competitive targets |
| Top straight-line speed | **≥ 1.47 m/s** (design) | From the path-length / finish-time budget in §2.2 |
| 0 → top speed | **≤ 0.5 s** | Must accelerate out of every corner before the next decision frame |
| Minimum turn radius | **≤ 0.5 m** `[PENDING E-6]` | Must clear the narrowest corridor the coin-toss can set (rules §5, Fig. 7) |
| Total mass | **≤ 1.0 kg** (well under the 1.5 kg cap) | Keeps motor-torque demand low and reduces rolling/inertial losses |
| Overall footprint | **184 × 96 mm** | Fits comfortably inside the 300 × 200 mm vehicle box |

Headline specs the v2 build targets against those goals:

| Spec | v2 value | Source |
|---|---|---|
| Length × Width × Height | **184 × 96 × 66 mm** | Fusion bounding-box export |
| Mass (full robot, design budget) | **0.750 kg** | §2 spreadsheet (used in the motor calc; updated for design changes) |
| Mass (measured) | **`[PENDING E-7]`** | per-component scale + Fusion mass-props |
| Drive motor | **N25 6 V 1330 RPM with quadrature encoder** | [robu.in product page](https://robu.in/product/n25-6v-1330rpm-metal-gear-motor-with-encoder-d-type-shaft/) |
| Steering servo | **EMAX ES08A II** | §6.2 |
| Drive ratio (motor drive gear → differential gear) | **13 : 38 = 2.923 : 1** | §4 |
| Wheel diameter | **56 mm** (LEGO Spike small) | §5 |
| Top speed (theoretical, no slip) | **≈ 1.33 m/s** (at 6 V) / **≈ 1.77 m/s** (at 8 V) | §2, §5 |
| Battery | **Orange 3S 1550 mAh 35 C LiPo** | §3.3 |
| Drivetrain layout | **Single-motor RWD through a mechanical differential, Ackermann front steer** | §4, §6 — satisfies rules 11.3/11.5 |

All tuneable parameters (encoder ticks-per-revolution, servo centre, battery cutoff, etc.) are mirrored into the firmware's `config.py`, so the doc and the code can never drift.

---

## 2. Powertrain — required-spec derivation

The most important calculation in the whole mechanical design is the one that asks **"what stall torque and what no-load RPM does the drive motor actually need?"** Everything downstream — drive ratio, wheel size, current draw, battery sizing — is derived from this single sheet.

### 2.1 Inputs

The 2025 robot gave us a calibrated baseline: a single LEGO EV3 medium motor at 9 V gave 280 RPM into 62.4 mm wheels through 20 T → 28 T gearing, finishing the open challenge in 45 s. From that baseline we set 2026 targets in the right-hand column.

| Variable | Symbol | 2025 (baseline) | 2026 (design target) |
|---|---|---|---|
| Motor RPM (no-load) | `N_m` | 280 | derived |
| Wheel diameter | `D_w` | 62.4 mm | 70 mm |
| Wheel circumference | `C_w = π·D_w` | 0.1960 m | 0.2199 m |
| Motor-side drive-gear teeth | `Z_p` | 20 | 13 |
| Differential-gear teeth | `Z_w` | 28 | 38 |
| Drive ratio | `G = Z_w / Z_p` | 1.40 | 2.923 |
| Target time | `t` | 45 s | **20 s** |
| Robot mass | `m` | — | **0.750 kg** |
| Rolling-friction coeff | `μ` | — | 0.10 (conservative for our wheel-on-mat) |
| Acceleration window | `t_a` | — | 0.5 s |
| Drivetrain efficiency | `η` | — | 80 % |
| Driven motors | `n` | 1 | 1 |

### 2.2 Required speed (from the baseline path length)

The 2025 baseline robot completed the run in **45 s** at an average speed of **0.653 m/s** (derived from 280 motor RPM, 1.40 drive ratio, and 62.4 mm wheels). This gives us a baseline run distance (approximate path length) of:

```
L_run = v_baseline × t_baseline = 0.6535 m/s × 45 s ≈ 29.41 m
```

To achieve our 2026 competitive design target of finishing in **20 s**, the required average speed is:

```
v_req = L_run / t_target = 29.405 m / 20 s ≈ 1.470 m/s
```

### 2.3 Required RPM (wheel, then motor)

```
N_wheel = (v_req / C_w) × 60   = (1.4703 / 0.2199) × 60   = 401.14 RPM
N_motor = N_wheel × G          = 401.14 × 2.923           = 1172.6 RPM
```

At the nominal 6 V, the N25 motor is rated at **1330 RPM no-load**. Since our target requires **1173 RPM**, the motor can achieve this at the nominal 6 V. However, we are overvolting the motor to **8 V** (providing an estimated no-load speed of `1330 × 1.33 ≈ 1770 RPM` with ~50% headroom) to ensure we comfortably hit the speed target even with battery voltage sag and load under real track conditions.

### 2.4 Required force

```
a_accel = v_req / t_a = 1.4703 / 0.5                      = 2.941 m/s²
F_accel = m × a_accel = 0.750 × 2.9405                    = 2.205 N
F_roll  = μ × m × g = 0.1 × 0.750 × 9.81                   = 0.736 N
F_total = F_accel + F_roll                                = 2.941 N
```

### 2.5 Required wheel torque, then motor torque

We calculate the torque requirements separately for starting (acceleration + rolling resistance) and sustaining (cruise, rolling resistance only):

**Starting Torque (during acceleration window):**
```
τ_wheel_start = F_total × (D_w / 2) = 2.9411 × 0.035        = 0.1029 N·m
τ_motor_start = τ_wheel_start / (G × η) = 0.1029 / (2.923 × 0.80) = 0.0440 N·m
                                                          = 4.402 N·cm
                                                          ≈ 0.449 kg·cm
```

**Sustaining Torque (steady-state cruise):**
```
τ_wheel_cruise = F_roll × (D_w / 2) = 0.7358 × 0.035        = 0.0258 N·m
τ_motor_cruise = τ_wheel_cruise / (G × η) = 0.0258 / (2.923 × 0.80) = 0.0110 N·m
                                                          = 1.102 N·cm
                                                          ≈ 0.112 kg·cm
```

### 2.6 Stall-torque budget vs the chosen motor

We never run a brushed DC motor anywhere near stall for sustained periods — the conventional continuous operating point is **30–40 % of stall torque** to stay below the thermal limit. 

By overvolting the motor to **8 V** (even though it is rated for 6 V):
* The rated continuous torque at operating RPM increases to **≈ 0.14 kg·cm** (scaled up from the 6 V nominal rating of **0.08 kg·cm**).
* The raw gearbox output stall torque at 8 V scales to **≈ 0.84 kg·cm** (scaled up from the 6 V output stall torque of `0.191 × 4.15 × 0.80 ≈ 0.63 kg·cm` based on a ~4.15 internal reduction).

Comparing this to our torque requirements:
1. **Steady-State Cruise:** The sustaining cruise torque of **0.112 kg·cm** is safely below the overvolted continuous rating of **0.14 kg·cm**, preventing thermal runaway.
2. **Peak Acceleration:** The starting torque requirement of **0.449 kg·cm** exceeds the continuous operating limit of **0.14 kg·cm**, but is well within the 8 V output stall torque of **0.84 kg·cm**. 

Because the starting torque exceeds the continuous operating limit, the motor will accelerate slightly slower at the very beginning of its ramp, but it operates safely without overheating since the acceleration phase is a transient 0.5 s window.

> **`[PENDING E-1]`** — exact stall torque, stall current, and the real (lower-than-80 %) drivetrain efficiency are confirmed on the bench, after which §2.5 is re-derived with the measured numbers.

---

## 3. Powertrain — chosen motor & alternatives

We searched four sources — **Maxon, Faulhaber, Pololu, and generic brushed gearmotors** — for a motor that meets §2 (≥ 1.10 N·cm sustaining / 4.40 N·cm starting, ≥ 1173 RPM, integrated encoder, light, affordable, deliverable to India in time).

### 3.1 The chosen motor

**N25 6 V 1330 RPM Metal Gear Motor with Encoder, D-shaft** (robu.in).

| Parameter | Value | Why it matters |
|---|---|---|
| Voltage | 6 V nominal | Matches one of the two buck rails (§3.3) |
| No-load RPM | 1330 | Hits the §2.3 target with 13 % headroom (expanded further by overvolting) |
| Stall torque | rated 0.08 kg·cm continuous (@ 6V) / ~0.14 kg·cm (@ 8V), ~0.63 kg·cm stall @ 6V / ~0.84 kg·cm stall @ 8V | Peak torque required for acceleration is safely below stall |
| Output shaft | D-shaft | Simple coupling to the 13 T drive gear |
| Encoder | integrated magnetic quadrature | Read by the software's PIO counter (software-doc §2.2) |
| Mass | 95 g | Datasheet-listed mass for the N25 gearmotor |
| Cost / availability | low-cost, stocked locally | Cheap enough to keep a spare for competition day |

### 3.2 Alternatives considered (and rejected)

| Source | Why rejected |
|---|---|
| **Maxon** (e.g. DCX-class brushed + planetary gearhead) | Best torque-to-size available, but very expensive and a multi-week ship to India; the fine encoder ribbon is fragile. Engineering overkill for a 0.8 kg car. |
| **Faulhaber** (e.g. 12 mm coreless + planetary) | Cleaner, quieter, longer-life gearbox than the N25, but again expensive, long lead time, and no affordable integrated-encoder variant in our budget. |
| **Pololu micro-metal gearmotor range** | Excellent, very well documented, but the units that fit our space have stall-torque margins that are too tight, and the tiny shaft/body complicates coupling to our drive gear and differential. |
| **Generic JGA25-371 12 V 280 RPM** | Lots of torque and stocked locally, but far too low-RPM (would need an extra step-up stage we have no room for), ~90g+, and electrically noisy on the 12 V rail. |
| **N20 6 V 500 RPM** (~15 g) | Cheap and tiny, but even geared we cannot reach the required wheel RPM without a step-up stage, and the smaller gearbox has less torque margin than the N25. |

The N25 sits in the sweet spot: enough torque margin to survive a heavier v3, an integrated encoder we actually plan to use, and a price that lets us keep a spare.

### 3.3 Battery & power-conditioning

- **Pack:** Orange 3S 1550 mAh 35 C LiPo. Nominal 11.1 V, max 12.6 V.
- **Mass:** `[PENDING E-7]` (the 2025 3S 2200 mAh pack was 175 g; the 1550 mAh pack is lighter).
- **Why 3S not 2S:** a sagged 2S pack (≈ 7 V) can fall below the buck dropout; 3S keeps both rails clean across the whole discharge curve.
- **Voltage rails:** two **XY-3606 step-down buck converters** (same model as the 2025 robot — see [`../README.md`](../README.md), *Voltage Converter*; note: the 2025 README will be moved to an archive folder, update the link when it moves). One buck is set to **6 V** for the drive motor and steering servo, the other to **5 V** for the Raspberry Pi 5 / sensor stack.
- **Motor driver:** **TB6612FNG dual H-bridge** (carried over from 2025). One channel drives the N25; the second is a spare.

```
                   ┌─────────────┐    6 V      ┌──────────┐    PWM     ┌─────┐
   3S LiPo ──────► │ Buck #1     ├────────────►│ TB6612FNG├────────────►│ N25 │
   11.1 V          │ XY-3606     │             │ (CH-A)   │             └──┬──┘
                   └─────────────┘             └──────────┘                │ encoder
                          │   6 V                                          ▼
                          └──────────► EMAX ES08A II steering servo    Raspberry Pi 5 PIO
                   ┌─────────────┐    5 V
                   │ Buck #2     ├────────────► Raspberry Pi 5 + sensor 5 V rail
                   │ XY-3606     │
                   └─────────────┘
```

---

## 4. Drivetrain — gear ratio, differential, axle

### 4.1 Layout

The drivetrain is **single-motor rear-wheel drive through a mechanical differential**:

```
  N25 motor ──► 13 T drive gear ──► 38 T differential gear ──► differential ──┬──► left rear wheel
                                                                              └──► right rear wheel
```

The front wheels are unpowered and steered by the Ackermann linkage (§6). The motor only ever drives the rear pair. **The gears are not 3D-printed** — the 13 T drive gear ships on the motor side and the 38 T gear is part of the differential; the "13 : 38" figure is simply the tooth count of that single drive-gear-to-differential-gear mesh.

### 4.2 Why a differential (and why this satisfies the rules)

Rule **11.3** requires a 4-wheeled vehicle with **one driving axle and one steering actuator**, and explicitly **disqualifies a differential-wheeled base**; rule **11.5** forbids an electronic differential with one motor per side. A single motor feeding a **mechanical differential** is exactly the compliant, intended solution: the drive wheels are physically connected through a gearbox/differential, not driven independently.

Mechanically, the differential also earns its place. A solid rear axle forces the inside rear tyre to *scrub* on every turn, because the outer wheel travels a longer arc. On smooth WRO mat that scrub shows up as understeer in tight corners and yaw kicks on transitions. The differential lets each rear wheel choose its own speed, so the chassis tracks the steered angle cleanly and the IMU heading we feed the controller (software-doc §3.3) agrees with where the car is actually pointing.

### 4.3 Why this ratio

`G = 38 / 13 = 2.923` was chosen by solving §2.3 in reverse: given the motor's 1330 RPM and the 56 mm wheel, what ratio makes the wheel turn at the required ~501 RPM with headroom?

```
G = N_motor / N_wheel_required ≈ 1330 / 501 ≈ 2.65
```

The 13 T drive gear and 38 T differential gear give the ratio 2.923.

### 4.4 The differential

The differential is an off-the-shelf bevel-gear unit ordered from Amazon (**`[PENDING]`** — part link / weight / bearing spec to be added on delivery). Its housing (`diffrential gear v2` in the CAD) is **closed to keep dust off the gears** — important on a venue floor where grit otherwise works into an open gear train and changes the backlash mid-competition. The 38 T gear is pressed onto the differential; the spider gears inside split torque between the two rear wheels.

---

## 5. Wheels & tyres

### 5.1 Chosen wheel

**LEGO Spike small wheel, 56 mm diameter, soft rubber tyre on a hard plastic hub.** We use this wheel exclusively. It fits the LEGO Technic axle we use as the rear shaft, and the soft rubber grips the WRO mat well.

### 5.2 Why this wheel (and rejection of the 70 mm alternative)

We initially considered upgrading to **70 mm wheels** to achieve a higher theoretical speed at 6 V, but we rejected this path:
1. **Controllability:** At 70 mm, the robot's top speed would make the vision/steering control loop too difficult to tune and control reliably.
2. **Play & Backlash:** The larger diameter amplifies any minor play and backlash in the 3D-printed differential housing, axle bearings, and steering joints, making the vehicle's handling unpredictable.
3. **Low Center of Gravity:** The 56 mm wheels keep the chassis lower, which stabilizes the robot in tight turns and provides a better, more consistent forward-tilted camera view of the mat.

### 5.3 Front and rear wheels are the same

Both front and rear use the 56 mm LEGO Spike small wheel. With Ackermann geometry (§6) handling slip, there is no need for the asymmetric narrow-front / wide-rear arrangement; a single common wheel keeps the bill of materials simple.

---

## 6. Steering — Ackermann geometry & servo

### 6.1 Why Ackermann (and why skid steering isn't even an option)

The 2025 robot used **parallel** steering (both front wheels at the same angle). That works for gentle turns, but at the tight inside-corner limits of the WRO track it forces the inside wheel to skid, because geometrically the inside wheel must turn through a *larger* angle than the outside wheel to stay tangent to its (tighter) arc. **Ackermann steering** fixes this by design: the linkage turns the inside wheel more than the outside wheel, the difference set by wheelbase and track. No skid → predictable yaw → the IMU heading controller agrees with the chassis.

**Skid steering is not a candidate at all** — it is *prohibited*. Rule 11.3 disqualifies a differential-wheeled base, and 11.5 forbids one-motor-per-side electronic differentials. A steered Ackermann front end with a single mechanical-differential drive axle is the rules-compliant architecture, and it is also the one our whole obstacle-avoidance pipeline (software-doc §5.4) is built around, since servo angle is the control input.

### 6.2 Servo selection — EMAX ES08A II

| Parameter | Value (per manufacturer listing) | Notes |
|---|---|---|
| Stall torque | ≈ 1.8 kg·cm @ 6 V | We need only ~0.3 kg·cm to steer on mat `[PENDING E-5]` |
| Speed | ≈ 0.1 s / 60° | Software clamps to ±10°/frame (software-doc §6.3), well inside this |
| Mass | ≈ 8–9 g | Tiny |
| Cost | low | Cheap enough to keep a spare |

**Alternatives considered:**

| Servo | Why rejected |
|---|---|
| **TowerPro SG90** | The "SG90" units on local marketplaces turned out to be counterfeits with brittle gears and inconsistent centre offset — we could not trust QC. |
| **TowerPro MG90s (metal gear)** | Tried during 2025 pre-competition bench testing and it **burned out** under sustained PWM — the metal gear train's friction kept the brushed motor near stall, and the case got hot enough to deform. |
| **Higher-end metal-gear digital servos** (e.g. Hitec / Savox class) | Excellent reliability, but several times the price and far more torque than a 0.8 kg car's steering needs. |

The ES08A II is the lowest-risk option that is neither a counterfeit SG90 nor the MG90s that already failed on us.

### 6.3 Ackermann geometry — current status

The Ackermann angle correction is set by `tan(δ_inner) − tan(δ_outer) = T / L`, where `T` is the track and `L` the wheelbase.

| Parameter | v2 value | Source |
|---|---|---|
| Wheelbase `L` | **`[PENDING E-4]`** | measured from the assembled v2 |
| Track `T` | **`[PENDING E-4]`** | measured |
| Max steer angle (inner) | **`[PENDING E-4]`** (hard clamp ±40° in firmware) | software-doc §8 |
| Resulting turn radius | **`[PENDING E-6]`** (target ≤ 0.5 m) | §1 |

The **current steering beam is a LEGO beam**. Once we lock `L` and `T` on the assembled robot we will measure the exact Ackermann arm angle, model a printed replacement beam at that angle in Fusion, and swap it in. The print → measure → reprint loop is the cheapest way to dial this in (a few hours per cycle).

---

## 7. Chassis — two-plate architecture

### 7.1 What's in the v2 chassis

The chassis is a **two-plate sandwich** with the battery and drivetrain between the plates.

- **Bottom plate** — the single body `Body21` in the CAD, which is the **combination of three sub-parts merged into one printed plate**: the base (`3dLego`), the motor mount (`mountmotor`), and the differential mount (`diffrential gear v2` mount). One piece — **the top cap of the differential mount — is printed separately** and screws on over the differential, closing the housing against dust.
- **Top plate** (`TopChasisSnsors`) carries the Raspberry Pi 5 and the sensor PCB (TCA9548A + the buck rails), plus the camera and IMU.
- **Battery cradle** (`BatteryWindow`) **mounts to the bottom plate** and holds the Orange 3S LiPo (`Orange1500`) flat along the chassis centreline — the lowest-CoG position (§9).
- **Spacers/standoffs** (`39s` and `32s`) are **printed separately** and screw between the bottom plate (at the back of the differential-mount part) and the top plate, setting the plate-to-plate gap.

See the iso renders in §15.1–15.2 and the exploded view in §15.6.

### 7.2 Why two plates (and not a monocoque, not three layers)

| Option | Pro | Con | Verdict |
|---|---|---|---|
| **Monocoque (one big print)** | Stiffest, lightest | Any component swap needs a full re-print; a failure halfway through wastes the whole print | Rejected |
| **Two plates (chosen)** | Components swap independently; each plate prints in a few hours; the Pi/PCB live above the noisy motor | Slightly heavier than a monocoque (spacers) | **Chosen** |
| **Three plates** | Best thermal isolation and cable routing | Adds height (raises CoG) and connector count | Over-engineered for v2; revisit in v4 |

### 7.3 Component-placement rationale

- **Motor at the rear, on the chassis centreline** → motor mass and torque reaction both stay centred → no yaw bias.
- **Battery centred between the plates** → lowest CoG height, mass over the wheelbase centre → balanced axle loading.
- **Raspberry Pi 5 on top, fan up** → unobstructed airflow and easy SD-card access between runs.
- **Camera at the front of the top plate, tilted forward** (software-doc §6.2) → maximum mat coverage for the ROI vision.

### 7.4 CAD software — Fusion 360

| Tool | Note |
|---|---|
| **SolidWorks** | No native macOS client (Windows-only). Most of the team is on Mac, so this was a non-starter. |
| **Onshape** | Runs in the browser, so it is Mac-friendly, but we preferred a full desktop parametric tool with offline mass-properties and direct 3MF/STEP export. |
| **FreeCAD** | Free and genuinely cross-platform (Mac/Windows/Linux), but its assembly workflow was less robust for a 14-component assembly than we wanted. |
| **Fusion 360 (chosen)** | Native clients on **both macOS (the team) and Windows (our coach)**, free **Education licence** obtained through a school ID (we are a robotics-club team, not a school team), fully parametric assemblies, built-in mass-properties, and direct 3MF/STEP/STL export. |

---

## 8. Materials, print settings, and 3MF reproducibility

### 8.1 Material — PLA across the board

All structural parts are **PLA**. We print on an open-frame **Bambu Lab A1**, which rules out ABS (warps badly without an enclosure). PLA was chosen over PETG because it is stiffer (the 184 mm top plate must not flex and shake the camera), warps less on long flat plates, is the cheapest, and is locally stocked in several colours. PETG, ABS, nylon-CF and TPU were each considered and set aside for these reasons; we don't print tyres (LEGO rubber wheels), so TPU isn't needed.

### 8.2 Print settings — the actual OrcaSlicer profile

We **forked the stock `0.20mm Standard @BBL A1` profile into a custom `0.20mm Fast A1` profile**. The real values (read straight from the profile JSON) are:

| Setting | Value | Why |
|---|---|---|
| Printer / nozzle | Bambu Lab A1, 0.4 mm | House printer |
| Layer height | **0.20 mm**, with **variable (adaptive) layer height** enabled per-part | Fine layers on curved/critical surfaces, coarse on flat plates → faster prints |
| Wall loops (perimeters) | **2**, **Arachne** wall generator | At this part scale the perimeters carry the load; Arachne handles the thin features cleanly |
| Top / bottom shells | **4 top / 3 bottom** | Solid skins do the structural work that the low infill doesn't |
| Infill | **7 %, rectilinear** | Deliberately low — small wall-dominated parts get most of their strength from the perimeters and shells, so we trade infill for weight and print time |
| Outer / inner wall speed | **300 / 450 mm/s** | Fast-iteration profile on the A1 |
| Sparse / solid infill speed | **500 / 500 mm/s** | — |
| Travel speed | **700 mm/s** | — |
| Supports | **On**, build-plate-only, 30° threshold | Only where overhangs need them |
| Brim / seam | brim none · seam aligned | — |

This is explicitly a **fast-iteration** profile: low infill and high speeds get a part off the printer quickly while we are still revising geometry every day. For final competition parts we can raise the infill on the load-bearing plates without touching the rest of the profile.

### 8.3 Why 3MF for the project files

Every printed part is committed as a **.3mf**, not a raw .stl. The project 3MF carries **multiple plates** (the parts laid out across several build plates exactly as we print them).

| Format | Stores | Why we don't rely on it |
|---|---|---|
| **STL** | Mesh only | A teammate or judge can't reproduce the print without guessing infill, walls, supports |
| **STEP** | Parametric geometry | Great for editing, but carries no print settings |
| **3MF** | Mesh + process profile + **multi-plate layout** + supports + material | A teammate opens it in OrcaSlicer, hits Print, and gets the **byte-identical** plates we printed |

**3MF is what makes the build reproducible** — directly addressing the rubric's "can another team reproduce this robot?" criterion (§Appendix A).

---

## 9. Mass & centre of gravity

### 9.1 Why CoG matters

Three failure modes are CoG-driven:

1. **Roll-over on tight turns.** With CoG too high, the lateral acceleration in an Ackermann corner can lift the inside wheel. The no-lift condition is `h < T·g·r / (2·v²)`; at v = 1.33 m/s, r = 0.5 m and the ~96 mm track this allows roughly `h < 135 mm` — far above our actual height, so we have margin.
2. **Front-axle unload under acceleration.** CoG too far back → hard accelerate-out-of-corner lifts the front and we lose steering authority for the next frame.
3. **Rear-wheel slip.** CoG too far forward → the differential's torque exceeds the rear contact patch → wheelspin → heading dead-reckoning drifts.

### 9.2 Target CoG envelope

| Axis | Target | Reasoning |
|---|---|---|
| Height above ground | **≤ 35 mm** | Roll threshold above, with a safety factor of ~2 |
| Longitudinal | **45–55 % of wheelbase from the front axle** | Slight rear bias to keep the drive wheels loaded |
| Lateral | **±2 mm of centreline** | Symmetric chassis → heading agrees with motion |

### 9.3 v2 CoG measurement plan

Fusion's mass-properties tool will give a predicted CoG once every part has the right material assigned. The model currently reports a CoG only with **default materials** assigned, which makes the raw number meaningless until we do the assignment. The protocol (§13 test E-7) is:

1. Assign PLA (1.24 g/cm³) to every printed part in Fusion.
2. Assign measured masses to the non-printed parts (motor, battery, Pi, sensor PCB, wheels — each `[PENDING]`).
3. Read CoG from Fusion.
4. **Verify on the bench** by balancing the assembled robot on a knife-edge front-to-back and left-to-right, to ±1 mm.
5. If the bench CoG disagrees with Fusion by > 5 mm, audit which component mass was wrong.

---

## 10. Design iterations — v1 → v2 → v3 → v4

### 10.1 v1 — 2025 season robot (shipped)

- LEGO Technic chassis, LEGO EV3 medium motor, SG90 servo, **parallel** steering, 56 mm LEGO Spike wheels (the same wheel v2 uses).
- 280 RPM × 62.4 mm → 0.65 m/s; 45 s open-challenge lap.
- Full writeup in [`../README.md`](../README.md) (the 2025 season doc — to be moved to an archive folder; update the link when it moves).
- **Lesson learned:** the LEGO chassis iterated fast (snap parts on/off) but was heavy (the EV3 motor alone is 40 g, plus the LEGO beams). Mass became the late-season bottleneck.

### 10.2 v2 — 2026 season, current build

- Fully custom 3D-printed two-plate chassis.
- N25 6 V 1330 RPM brushed gearmotor with quadrature encoder (≈ 3.5× the v1 motor's RPM).
- 13 T → 38 T drive into a closed mechanical differential.
- 56 mm LEGO Spike wheels (front and rear).
- EMAX ES08A II servo (after the SG90/MG90s failures in v1).
- **Ackermann** front steering, replacing v1's parallel steering.
- **Why each change:** custom chassis is lighter than LEGO, offsetting the N25's larger 95 g weight compared to the EV3 medium motor (which casing-wise was bulkier and required heavy LEGO structural framing); encoder → distance-based parking instead of timer-based; Ackermann → no scrub → heading agrees with chassis; lighter 1550 mAh pack → less mass at the same run time.

### 10.3 v3 — planned

- **Goal:** fewer parts. Merge the sub-mounts so the chassis is essentially **one bottom plate + one top plate**.
- **Risk:** any single cracked mount then needs a full chassis re-print.
- **Trigger:** when the print-and-assemble cycle starts costing more time than the bench-debug cycle.

### 10.4 v4 — planned

- **Goal:** a **smaller differential**, freeing space for a wider sensor PCB and allowing a shorter wheelbase (tighter turn radius).
- **Trigger:** post-Regionals, ahead of Nationals.

### 10.5 Iteration summary

| Iteration | Status | Chassis | Motor | Servo | Steering | Wheels |
|---|---|---|---|---|---|---|
| v1 | Shipped (2025) | LEGO Technic | LEGO EV3 medium | SG90 | Parallel | Spike 56 mm |
| v2 | **Current build** | 3D-printed two-plate | N25 6 V 1330 RPM enc | EMAX ES08A II | Ackermann | Spike 56 mm |
| v3 | Planned | two-plate, merged mounts | N25 (carry) | EMAX (carry) | Ackermann | 56 mm |
| v4 | Planned | v3 + smaller differential | N25 (carry) | EMAX (carry) | Ackermann | TBD |

---

## 11. Why theory didn't match reality

1. **Path length: naive vs measured.** The tight-centreline estimate (~29 m over three laps) is optimistic — the real weaving racing line is longer, which is why we re-measured it as a ~2.5 m-radius circle (~47 m) and set the Open Challenge finish target at ≈ 30 s rather than the spreadsheet's original 20 s (and we expect the Obstacle Challenge to take ≈ 45 s due to pillar avoidance), at the same 1.47 m/s design speed.
2. **Wheel diameter: 70 mm designed → 56 mm built.** Top speed drops from ≈ 1.67 to ≈ 1.33 m/s (at 6 V) / ≈ 1.77 m/s (at 8 V). The 70 mm upgrade was rejected due to controllability and play/backlash concerns, locking us to the 56 mm wheels.
3. **Drivetrain efficiency: 80 % assumed → `[PENDING E-1]`.** A drive-gear mesh plus a bevel differential is realistically 60–70 %; §2.5 gets re-derived once measured.
4. **Mass: 0.750 kg actual vs 0.818 kg budgeted.** Design improvements have reduced the overall robot mass to 0.750 kg, offsetting the motor's actual 95 g weight (which was originally misestimated as 30 g in the initial draft).
5. **Rolling friction μ = 0.10 assumed** — conservative; the mat is smoother, so we are over-spec'd on torque (a good problem).

---

## 12. Problems faced & innovative solutions

### 12.1 MG90s servo burnout (from v1 troubleshooting)

**Problem.** Pre-competition bench tests in 2025 ran the MG90s metal-gear servo at the sustained PWM duty our obstacle code commands. The case got hot enough to soften, and the brushed motor inside burned out — we smelled it before we saw it.

**Diagnosis.** The MG90s metal gear train has more friction than the SG90's plastic gears, and hobby servos in this torque class are sized for *intermittent* positioning, not the continuous sub-second corrections our control loop demands. The motor sat near stall current at too high a duty cycle.

**Solution.** Move to the **EMAX ES08A II** (lighter, lower-friction gears at the same nominal torque, with an internal driver that limits stall current), and add the software-side **±10°/frame slew clamp** (software-doc §6.3), which bounds the commanded angle change per frame and so lowers the average power dissipated in the gear train.

### 12.2 *(More to be added as the v2 build reveals them.)*

> Placeholder for two further entries from the v2 build/test. Likely candidates: rear-axle deflection under acceleration, top-plate flex shaking the camera, servo-horn slip under load, or buck dropout at peak motor current.

---

## 13. Tests & experiments still needed

Each test fills a `[PENDING]` value above.

| ID | Test | Setup | Fills |
|---|---|---|---|
| **E-1** | Motor bench characterisation | N25 at 6 V, ammeter in series, tacho on the shaft, encoder via Pi PIO; sweep duty 20→100 % | §2.6 stall margin; §11 efficiency |
| **E-2** | Drivetrain efficiency | Robot on a rolling-road jig; constant duty; compare motor current to drag-free RPM | §2.5 efficiency |
| **E-3** | Top speed on mat | `MOTOR_SPEED = 100 %`, 3 m straight, time + IMU log; compare to 1.33 m/s (6 V) / 1.77 m/s (8 V) | §1; motor performance validation |
| **E-4** | Wheelbase / track / steer angle | Calipers + protractor on the assembled v2 at max-left and max-right | §6.3 Ackermann table |
| **E-5** | Steering load torque | Lift the front, command max angle, clamp-meter the servo current | §6.2 |
| **E-6** | Turn-radius measurement | Full lock at low speed; pen at the rear axle traces an arc | §6.3 turn radius |
| **E-7** | Mass & CoG | Scale every part, assign in Fusion, read CoG, verify by knife-edge balance | §1 mass; §9 |
| **E-8** | Encoder integrity at speed | Run under load, watch for missed counts on the PIO debug counter | software-doc §5.7 |
| **E-9** | Battery sag at peak current | Scope the 6 V rail, trigger on a hard-accel PWM edge | §11 sag |

---

## 14. Assembly instructions

This is the build sequence from a fresh plate of parts. Photo callouts (**"as per photo"**) refer to the bench-build photos in §13's eventual capture set.

> **Tools:** Phillips driver, M2 / M2.5 / M3 hex drivers, sharp-nose pliers, needle file (clean print artefacts), the LEGO steering sub-assembly built from the project `.io` file.

1. **Motor.** Snap the N25 motor into the bottom plate and screw it down with **M3**.
2. **Drive-gear bearing.** Fit the bearing for the 13 T drive gear into its seat.
3. **Motor shaft + 13 T gear.** Put the motor shaft into the 13 T gear and slide it through the bearing until it seats against the motor.
4. **Differential.** Drop the differential in, add the remaining bearings **as per the photo**, slide in the bearing-slider part, and screw it down with **M2**.
5. **Differential cap.** Put the separately-printed top part of the differential mount on top and screw it down with **M2.5** (this closes the dust-proof housing).
6. **Steering.** Assemble the LEGO steering sub-assembly **per the project `.io` file**, then fit it into the front of the bottom plate.
7. **Servo mount.** Place the servo-mount part above the steering assembly.
8. **Servo.** Press the servo horn onto the servo-horn part, seat the servo into the servo mount, and screw it in with **M2.5**.
9. **Battery cradle.** Screw the battery window (cradle) onto the bottom plate.
10. **Top plate.** Put the top plate on and screw in the **39 mm (`39s`) spacers** at the back of the differential-mount part, joining the bottom plate to the top plate.
11. **Electronics.** Mount the custom PCB onto the Raspberry Pi, then screw the Pi/PCB stack down on its spacers.
12. **Connect & power-test.** Connect the camera; land the battery into the screw terminal on the PCB. Switch on, and **test it following the software doc** before going further.
13. **Wheels (last).** Once all motors and electronics check out, fit the wheels: the **front wheels go directly into the LEGO steering assembly**, and the **rear wheels mount with the black LEGO screws into the bearing-slider part**.

---

## 15. CAD render gallery

> Rendered directly from `cad/WRO2026Chasisv1.1.f3d` via the Fusion 360 API (origin/joint/sketch glyphs hidden; the redundant `3dLego` source occurrence hidden so the merged `Body21` bottom plate shows cleanly).

### 15.1 Iso, top-right

![iso top right](cad/iso_top_right.png)

The broad top plate carries the battery cradle. The motor/differential housing sits below at the rear (right of frame); the Ackermann steering is at the front (left of frame).

### 15.2 Iso, top-left

![iso top left](cad/iso_top_left.png)

Mirror of 15.1. The servo mount is at the front-left, ahead of the steering linkage.

### 15.3 Top

![top](cad/top.png)

Top-down footprint (184 × 96 mm plus the front steering protrusion). The battery runs along the chassis centreline.

### 15.4 Front

![front](cad/front.png)

Front view — the front wheel mounts and the LEGO steering linkage.

### 15.5 Side

![left side](cad/left.png)

Side view — wheelbase, chassis height, and the spacer gap between the two plates.

### 15.6 Exploded view

![exploded](cad/exploded.png)

Top to bottom: the sensor/top plate lifts off the top; the four printed spacers (`39s`, `32s`) stand at the corners; the battery and its cradle sit between the plates; and the combined `Body21` bottom plate — with the motor, the closed differential, and the motor mount — sits at the base, with the differential dropped clear below it.

### 15.7 Drivetrain detail

![drivetrain](cad/drivetrain.png)

Motor with its D-shaft and 13 T drive gear, the two halves of the closed differential housing separated vertically (upper half raised to show the split), and the motor mount below. Spacers and plates are hidden for clarity.

---

## Appendix A — Cross-reference to the WRO 2026 Future Engineers documentation rubric

The rubric scores Criterion 1 (**Mobility & Mechanical Design**) on drive/steering choices, torque/speed justification, mechanical reasoning, structure & mounting, why components were chosen, and testing/iteration evidence. Map:

| Rubric expectation (Criterion 1, top band) | Where covered |
|---|---|
| Drive & steering mechanism choices | §4 (RWD + differential), §6 (Ackermann) |
| Torque / speed justification | §2 (full derivation), §3 |
| "Why this component, not that one" | §3.2 (motors), §5.2 (wheels), §6.2 (servos), §7.4 (CAD) |
| Mechanical structure, mounting, stability | §7 (two-plate chassis), §9 (CoG & roll margin) |
| Testing / iteration affecting performance | §10 (v1→v4), §11 (theory vs reality), §13 (test plan) |
| Diagrams / CAD renders | §15 (renders), §3.3 (power block diagram) |
| **Reproducibility** (Criterion 5: CAD files, ≥5000-char docs, can another team rebuild it?) | §8.3 (multi-plate 3MF), §14 (assembly steps), this document |
| Power & sense management (Criterion 2) | [`software-architecture-and-obstacle-strategy.md`](software-architecture-and-obstacle-strategy.md) §9 |
| Software & obstacle strategy (Criterion 3) | [`software-architecture-and-obstacle-strategy.md`](software-architecture-and-obstacle-strategy.md) §3–§5 |
