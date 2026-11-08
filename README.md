<!-- WEB-ONLY:BEGIN -->

<img src="docs/diagrams/WatchFirstBanner_pastel.png" alt="Quick 5 min video. We recommend you watch it to get a quick understanding of our robot and documentation." width="100%"/>

[<img src="docs/diagrams/SummaryVideoThumbnail.png" alt="Thumbnail of the 5 minute Greenbotics summary video; click to open the YouTube playlist" width="700"/>](https://m.youtube.com/playlist?list=PLMdippUF4xxo)

Quick Highlights of our documentation are available in [Presentation PDF](docs/Engineering_Journal/Greenbotics_Presentation.pdf) which was also used in this video.
<!-- WEB-ONLY:END -->

# Table of Contents

<!-- TOC:BEGIN -->

1. [Introduction](#1-introduction)
   - 1.1 [Summary and Performance Video](#11-summary-and-performance-video)
   - 1.2 [Vision](#12-vision)
   - 1.3 [The Team](#13-the-team)
   - 1.4 [Vehicle Photos](#14-vehicle-photos)
2. [Mobility and Mechanical Design](#2-mobility-and-mechanical-design)
   - 2.1 [Drivetrain Selection](#21-drivetrain-selection)
   - 2.2 [Vehicle Dimensions](#22-vehicle-dimensions)
   - 2.3 [Steering Selection](#23-steering-selection)
   - 2.4 [Differential Assembly Selection](#24-differential-assembly-selection)
   - 2.5 [Wheel Selection](#25-wheel-selection)
   - 2.6 [Speed Torque Calculation (Drive Motor Selection)](#26-speed-torque-calculation-drive-motor-selection)
   - 2.7 [Steering Motor Selection](#27-steering-motor-selection)
   - 2.8 [3D Printed Parts](#28-3d-printed-parts)
   - 2.9 [Mobility Inaccuracies and Mitigation](#29-mobility-inaccuracies-and-mitigation)
   - 2.10 [Experiments](#210-experiments)
3. [Power and Sensor Architecture](#3-power-and-sensor-architecture)
   - 3.1 [Power Budget](#31-power-budget)
   - 3.2 [Power Strategy](#32-power-strategy)
   - 3.3 [Power Verification](#33-power-verification)
   - 3.4 [Electrical Noise and Mitigation](#34-electrical-noise-and-mitigation)
   - 3.5 [Electrical Safety: Fuse](#35-electrical-safety-fuse)
   - 3.6 [Wiring Diagram and PCB](#36-wiring-diagram-and-pcb)
   - 3.7 [Sensors](#37-sensors)
   - 3.8 [Microcontroller: Raspberry Pi 5 as Single Controller](#38-microcontroller-raspberry-pi-5-as-single-controller)
   - 3.9 [Calibration Procedures](#39-calibration-procedures)
   - 3.10 [Failure Point Analysis](#310-failure-point-analysis)
   - 3.11 [Iteration Evidence](#311-iteration-evidence)
4. [Software Architecture and Obstacle Strategy](#4-software-architecture-and-obstacle-strategy)
   - 4.1 [Design Philosophy](#41-design-philosophy)
   - 4.2 [System Architecture](#42-system-architecture)
   - 4.3 [Open Challenge Flow Chart and Algorithm](#43-open-challenge-flow-chart-and-algorithm)
   - 4.4 [Obstacle Challenge: State Machine and Algorithms](#44-obstacle-challenge-state-machine-and-algorithms)
   - 4.5 [Edge Cases](#45-edge-cases)
   - 4.6 [Interference Due to Surroundings](#46-interference-due-to-surroundings)
   - 4.7 [Parameter Tuning](#47-parameter-tuning)
   - 4.8 [RPM Control: PI Controller with Feed-Forward](#48-rpm-control-pi-controller-with-feed-forward)
   - 4.9 [Robot Testing](#49-robot-testing)
   - 4.10 [Performance Optimizations](#410-performance-optimizations)
   - 4.11 [Troubleshooting and Debugging](#411-troubleshooting-and-debugging)
5. [Systems Thinking and Engineering Decisions](#5-systems-thinking-and-engineering-decisions)
   - 5.1 [System Overview](#51-system-overview)
   - 5.2 [Shared Constraints](#52-shared-constraints)
   - 5.3 [Cross-Subsystem Decision Case Studies](#53-cross-subsystem-decision-case-studies)
   - 5.4 [Risk and Failure Modes](#54-risk-and-failure-modes)
   - 5.5 [Iteration and Testing Cycle Summary](#55-iteration-and-testing-cycle-summary)
   - 5.6 [Cross-Subsystem Troubleshooting](#56-cross-subsystem-troubleshooting)
6. [Reproducibility and GitHub Organization](#6-reproducibility-and-github-organization)
   - 6.1 [Repository Structure and Module Map](#61-repository-structure-and-module-map)
   - 6.2 [Robot Build Instructions](#62-robot-build-instructions)
   - 6.3 [Software Setup and Running the Robot](#63-software-setup-and-running-the-robot)
   - 6.4 [Testing Workflow](#64-testing-workflow)
   - 6.5 [Parts List (Bill of Materials)](#65-parts-list-bill-of-materials)
   - 6.6 [Engineering Journal](#66-engineering-journal)
   - 6.7 [Matlab Journal](#67-matlab-journal)

<!-- TOC:END -->

---

# 1 Introduction

We are team Greenbotics and are competing in WRO 2026 Future Engineers category. This is our second year after a successful participation in WRO 2025, where we were one of the 5 teams to hit a [full score](https://scoring.wro-association.org/en/event/scoring/293) in all 4 rounds. You can find our documentation from 2025 [here](https://github.com/Devansh-awat/greenbotics).

<img src="docs/diagrams/WRO_2025_score.png" alt="WRO 2025 Future Engineers scoreboard showing a full score for Greenbotics in all four rounds" width="70%"/>

---

## 1.1 Summary and Performance Video

**We recommend you to watch the summary video to get a quick understanding of our robot and documentation.**
- [Summary Video](https://youtu.be/YS-oyNQQUGs)
- [Open Challenge Performance Video](https://youtu.be/OW5V_LRIKNU)
- [Obstacle Challenge Performance Video](https://youtu.be/RbtGV2t9GcI)

---

## 1.2 Vision

Even though we had a robot from WRO 2025 that performed perfectly across all rounds, we decided to challenge ourselves to achieve **Speed With Accuracy**. This led us to rethink many aspects like steering, gears, chassis, motors, sensors , safety features and efficiency of software algorithms. We achieved 2x speed while maintaining 100% accuracy with this redesign. **This document explains in detail the reasons for all choices, our experiments, tradeoffs, algorithms and iterations.**

<img src="docs/diagrams/Vision_2026.png" alt="Vision 2026" width="850">

---

## 1.3 The Team

<table style="width: 100%; text-align: center; border-collapse: collapse;">
   <tr>
    <td style="padding: 10px;">
      <img alt="Greenbotics team photo" src="t-photos/GreenboticsTeamPic.jpeg" width="450" style="margin-center:20px;"/>
    </td>
    <td style="padding: 10px;">
     <img alt="Greenbotics team members in a light-hearted group photo" src="t-photos/GreenboticsClowns.jpeg" width="350" style="margin-center:20px;"/>
    </td>
  </tr>
</table>

Devansh Awatramani and Rakshith Rao are Grade 10 students at The Riverside School, Ahmedabad. Devansh Harivallabhdas is a Grade 11 student at Ahmedabad International School, Ahmedabad.

Paresh Gambhava is our coach from The Robotronics Club, Ahmedabad

---

## 1.4 Vehicle Photos

<table>
  <tr>
    <td align="center">
      <img src="v-photos/Left.png" alt="Robot vehicle, left side view"   width="400"><br>
      <b>Left</b>
    </td>
       <td align="center">
      <img src="v-photos/Right.png" alt="Robot vehicle, right side view"   width="400"><br>
      <b>Right</b>
    </td>
    </tr>
    <tr>
    <td align="center">
      <img src="v-photos/Front.png" alt="Robot vehicle, front view" width="300"><br>
      <b>Front</b>
    </td>
    <td align="center">
      <img src="v-photos/Back.png" alt="Robot vehicle, back view"  width="300"><br>
      <b>Back</b>
    </td>
   </tr>
</table>

<table>
  <tr>
   <td align="center">
      <img src="v-photos/Top.png" alt="Robot vehicle, top view" width="700"><br>
      <b>Top</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="v-photos/Bottom.png" alt="Robot vehicle, bottom view" width="700"><br>
      <b>Bottom</b>
    </td>
  </tr>
</table>

---

# 2 Mobility and Mechanical Design

This section describes the Mechanical design of the robot including its chassis, drivetrain and gearbox. It also mentions our experiments, speed & torque calculations, component selection criteria, 3-D designed parts and Mechanical Inaccuracies / Mitigation strategies.

---

## 2.1 Drivetrain Selection

WRO rules do not allow a differential drive robot. This steers us towards a real vehicle like design that consists of a front wheel steering. These vehicles have multiple drive options

|  | All Wheel Drive | Front Wheel Drive | Rear Wheel Drive |
| :---- | :---- | :---- | :---- |
|  Driving power | All wheels | Front wheels | Rear wheels |
| Physical complexity | Transfer rotational force to a wheel that is changing its angles with respect to chassis | Transfer rotational force to a wheel that is changing its angles with respect to chassis | Simpler design. Rear axle \- rotational force. Front axle \- turning force |
| Mechanical complexity | Drive motor mounted on steering mechanism. Requires powerful servo due to extra weight | Drive motor mounted on steering mechanism. Requires powerful servo due to extra weight | **Separates mechanical responsibilities.** Steering and Drive motors mounted on separate axles |
| Highlights | Off roading, steep inclines, muddy road | Pulling over obstacles  | Smooth roads, better turns |

We chose RWD for its simpler design and smoother turns.

<table>
  <thead>
    <tr>
      <th>Differential Gear</th>
      <th>Diagram</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>The rear wheels have a differential gear to prevent inner wheels from skidding when turning. As shown in the diagram, during turns, the outer wheel covers more distance(wo) than inner wheels (wi). In absence of differential gear, the inner wheels would skid.
      </td>
      <td >
       <img src="docs/diagrams/mobility/Differential_gear.png" alt="Diagram of a turn in which the outer wheel covers more distance than the inner wheel, which the differential gear allows" width="300">
      </td>
    </tr>
  </tbody>
 </table>

---

## 2.2 Vehicle Dimensions

The defining constraint in this vehicle is its turning radius for its parallel parking. The turning radius is defined by the dimensions of the vehicle.

<table>
   <thead>
    <tr>
      <th>Length Impact</th>
      <th>Width Impact</th>
    </tr>
  </thead>
  <tbody>
      <tr>
     <td>
        <img src="docs/diagrams/mobility/Length_dimension_impact.jpeg" alt="Diagram showing how vehicle length sets the turning radius, R = L / sin(theta)" width="400">
      </td>
      <td>
        <img src="docs/diagrams/mobility/Width_dimension_impact.jpeg" alt="Diagram showing how vehicle width increases the outer clearance radius, R(outer) = R(center) + W/2" width="400">
      </td>
    </tr>
    <tr>
      <td>
        <em>R = L / sin(θ)</em><br>
        <strong>Turning Radius(R) scales proportionally with the Length(L) of the vehicle.</strong>
      </td>
       <td>
        <em>R(outer) = R(center) + W/2</em><br>
       <strong> Width does not change the turning radius, but increases the outer clearance radius.</strong>
      </td>
      </tr>

  </tbody>
</table>

We strive to keep both the Length and Width as minimum as possible to achieve smallest possible turning radius.
Length is the minimum length to accommodate the differential gear assembly, drive motor and the steering assembly back to back.  
Width is the minimum width for the differential assembly, couplings and the wheels attached back to back.  
This results into a 23 cm long, 12 cm wide and 28 cm high vehicle. The Vehicle dimensions are annotated in [Vehicle Photos](#14-vehicle-photos).

---

## 2.3 Steering Selection

**Ackermann vs Parallel**  
We used Parallel steering in our last year's robot. We realised certain manoeuvres such as entering parking space and tight turns between two inner blocks and inner walls caused tyre slip. We improved upon this aspect for this year's robot by using Ackermann steering. In Ackermann steering, the inner wheel turns slightly more than the outer wheel. During a turn all four wheels trace arcs around a common center on the extended rear axle centerline as shown in below diagram. The inner wheel arc has a smaller radius than the outer wheel arc, so it turns more sharply to avoid tyre slip. This improves maneuverability especially during cornering.

| Ackermann Steering Concept |Our Robot Design |
| :---: | :---: |
| <img src="docs/diagrams/mobility/ackermann_steering.png" alt="Ackermann steering concept: the inner wheel turns more than the outer wheel so all wheels trace arcs around a common center on the rear axle line" height="200" width="200"> | <img src="docs/diagrams/mobility/robo_ackermann.png" alt="The Ackermann steering linkage on the Greenbotics robot" height="500" width="400"> |

### 2.3.1 Ackermann Geometric Calculations

We calculated the Steering arm length using the Ackermann Geometry Calculations as below.

<img src="docs/diagrams/mobility/AckermanMeasurements.png" alt="Ackermann geometry calculation used to find the steering arm length" width="1000" style="border: 1px solid #ccc;">

 We verified that our actual steering arm length of **25mm closely matches the calculations.**

 <img src="docs/diagrams/mobility/steering_arm_measurement.jpeg" alt="Ackermann geometry steering arm calculation verification" width="500">

### 2.3.2 Turning Radius Calculations

After assembling servo motor and Ackermann steering, the max steering angle was measured to be 33° for the outer wheel. This was used in below calculations to calculate turning radius, which was further used to calculate robot position from where it should initiate parking.

<img src="docs/diagrams/mobility/ParkingMeasurements.jpeg" alt="Turning radius and parking offset calculation for the parallel parking start position" width="900">

**Errata:** The formula incorrectly uses R_outer instead of R_rear_outer. The turning radius is of the rear outer wheel.

We verified that **Actual Turning Radius and Parking Offset of the actual Robot match closely with the calculated values.** And below is the video recording of our verification experiment.

<img src="docs/diagrams/mobility/RobotParking1.gif" width="500" alt="Robot Parking with calculated Turning Radius">

---

## 2.4 Differential Assembly Selection

We used LEGO differential gear assembly in our last year's robot. While this gave us adequate performance, there were few drawbacks to it

1. Gear damage: Plastic gears chip away after continued use.  
2. Backlash: When the robot makes micro adjustments during parking switching from forward to reverse, the motor rotates slightly before the wheels actually move. This makes software control of the robot inconsistent. If we move the motor a bit more, the robot sometimes hits the walls, if we make the motor move a bit less, it doesn't move.

Both these problems can be resolved with metal differential gears. To allow for precise control, we chose the largest possible gear ratio that could fit in the chassis. We chose a metal differential gear with a 38:13 ratio of ring gear to pinion gear. This gear ratio gives higher torque providing reliable transmission even at lower speeds. There is loss of top speed, but we do not need to race the car so that is fine.

| Feature | Plastic Differential (e.g., LEGO) | Metal Differential |
| :---- | :---- | :---- |
| **Backlash**  | High (Rough control at low speed during parking) | Low (Precise control at low speed during parking) |
| **Friction** | Plastic-plastic \- higher friction (Prone to low-speed jerky motion) | Metal-metal \- lower friction (Smooth low-speed crawl) |
| **Rigidity / Prone to damage** | Teeth Chip away after continued usage | Rigid and stable |
| **Robot Image with Gear** |<img src="docs/diagrams/mobility/Plastic Differential.png" alt="Robot rear axle fitted with the LEGO plastic differential gear" width="300">|<img src="docs/diagrams/mobility/MetalDifferential.jpeg" alt="Robot rear axle fitted with the 38:13 metal differential gear" width="300">|

---

## 2.5 Wheel Selection

We used LEGO Spike Prime wheels [56mm diameter, 14 mm wide]. These narrow bicycle-like wheels give smoother turns with less friction. A bigger wheel would amplify the backlash causing imprecise movements during parking.
We also tried 3D printing our own custom wheels, and coating it with a cricket bat's rubber grip. But we realised we couldn't match the fit and finish of a pre-fabricated LEGO wheel.

[For Wheel Attempts refer Section 5.5.1 3D Iterations](#551-3d-chassis-iterations)

---

## 2.6 Speed Torque Calculation (Drive Motor Selection)

We used a LEGO EV3 medium motor in our last year's robot and its speed was the limiting factor for our robot. We were running the motor at 100% speed and couldn't go faster.  
Old robot rpm at wheel

```
rpm_wheel_old = rpm_motor_old / differential_gear_ratio_old

rpm_wheel_old = 250 / (28 / 20) = 250 / 1.4 = 179
```

We wanted the new robot to go at least 50% faster with enough headroom so we aimed for 100% higher rpm at wheel

```
rpm_wheel_new = rpm_motor_new / differential_gear_ratio_new

rpm_wheel_new = 2 * rpm_wheel_old = 2 * 179 = 358

358 = rpm_motor_new / (38 / 13)
rpm_motor_new = 358 * (38 / 13) = 358 * 2.92 = 1045
```

We explored different motors from [Pololu](https://www.pololu.com/product/4861) as they offer a wide range of gear ratios. We ran below Speed Torque calculations with some motors and shortlisted Pololu 4861 that provides 0.71 kg·cm torque @ 1800 rpm. This motor runs at 12V removing the need for a step down converter and associated efficiency losses.

| Criteria | LEGO EV3 Medium Motor |TT-GM25 | Pololu 4861 |
| :---- | :---- | :---- |:---- |
| Power supply match | Needs buck converter (12 V → 9V), small conversion loss | Needs buck converter (12 V → 6V), small conversion loss | Runs directly off 11.1 V LiPo, no converter required |
| Mechanical reliability | Plastic gears —  can slip or chip away and has higher backlash | Metal Gearbox | All-metal gearbox — More reliable and has lower backlash |
| No load speed | 250 rpm | 1350 rpm | 1800 rpm |
| Stall torque | 1.22 kg·cm | 0.19 kg·cm | 0.71 kg·cm |
| Rated stall current | 0.8 A | 2.1 A | 1.8 A |

The Pololu motor natively matches the power source and has better mechanical reliability.

### 2.6.1 Speed Torque Calculations

| Symbol | Meaning | Value |
| :---- | :---- | :---- |
| m | Robot mass | 0.8 kg |
| g | Gravitational acceleration | 9.81 m/s² |
| W | Robot weight \= m × g | 7.85 N |
| D\_w , r\_w | Wheel diameter / radius | 0.056 m / 0.028 m |
| N\_nl | Motor no-load speed at gearbox output (11.1 V) | 1800 RPM |
| T\_stall\_gb | Stall torque at gearbox output (extrapolated) | 0.71 kg·cm \= 0.0696 N·m |
| I\_stall | Stall current | 1.8 A |
| G\_ext | External gear ratio (13:38 metal differential) | 38 / 13 \= 2.923 |
| μ | Wheel-to-mat friction coefficient (assumed) | 0.6 (rubber-type wheel on flex banner) |
| C\_rr | Rolling resistance coefficient (assumed) | 0.03 |
| C\_rr,static | Higher than C\_rr due to stiction | 0.05 |
| T\_cont\_gb | Recommended continuous torque at gearbox output | 25% of T\_stall\_gb (manufacturer guideline: keep current ≤ 25% of stall) |

**No load speed**

```
v_nl = (π × D_w × N_nl) / (60 × G_ext)

v_nl = (π × 0.056 × 1800) / (60 × 2.923) = 1.81 m/s
```

**A) Starting Torque v/s Stall Torque rating**

Can the motor move the robot from a dead stop? The motor is in stall condition at the moment it starts and the stall torque should exceed the initial resistance(static friction) that the robot needs to begin moving.

**B) Running Torque v/s Continuous Torque rating**

Is the torque required for continuous driving comfortably within motor's output to avoid constantly overloading the motor that could result in motor breakdown

<img src="docs/diagrams/mobility/TorqueCalculations.png" alt="Starting torque and running torque calculations for the drive motor with margins against stall and continuous torque ratings">

**C) Running speed v/s No load speed**

The Pololu 4861 datasheet ([https://www.pololu.com/file/0J1829/pololu-25d-metal-gearmotors-rev-2-0.pdf](https://www.pololu.com/file/0J1829/pololu-25d-metal-gearmotors-rev-2-0.pdf)) shows that torque and speed are linearly related.

<img src="docs/diagrams/mobility/Polulu_datasheet.png" alt="Pololu 4861 datasheet graph showing the linear relationship between motor torque and speed" width="600">

Using the fraction of stall torque with the running torque from (b)

```
N_gb = N_nl × (1 − T_req,gb / T_stall_gb)

N_gb = 1800 × (1 − 0.0230/0.71) = 1800 × 0.9676 = 1741.7 RPM

N_wheel = N_gb / G_ext = 1741.7 / 2.923 = 595.8 RPM

v = π × D_w × N_wheel / 60 = π × 0.056 × 595.8 / 60 = 1.75 m/s
```

**Conclusion:** As the running torque is ~ 3% (0.0230/0.71) of the stall torque, the motor operates close to its no-load condition during normal driving and its running speed of 1.75 m/s is very close to its no-load speed of 1.81 m/s. As the current drawn stays low there is a large torque reserve to manage turns without stalling and sustain battery for a longer time.

**Experiment Metrics**
|Theoretical No Load Speed| Theoretical Running Speed |Observed Running Speed|
|---|---|--|
|1.81 m/s|1.75 m/s|1.54 m/s|

**Assumptions**

Our calculations assumed no loss at differential gear. However a differential gear would have efficiency of **η=85%** due to frictional losses. But, its impact wouldn't be of concern to us as we have a very high margin.

**Summary**

| Check | Question answered | Torque req'd at gearbox (kg·cm) | Torque available (kg·cm) | Margin |
| :---- | :---- | :---- | :---- | :---- |
| A: Starting Torque v/s Stall Torque rating | Can it start moving from rest? | 0.0383 | 0.71 (stall) | 18.5× |
| B: Running Torque v/s Continuous Torque rating | Can it sustain cruising without overheating? | 0.0230 | 0.1775 (25% cont.) | 7.7× |

**The Pololu motor has adequate torque margin to start the robot from rest, sustain cruising without overheating and to provide the max speed we could possibly use.**

---

## 2.7 Steering Motor Selection

For our last year's robot, we used an SG90 servo motor for front wheel steering for precise steering control. While it was adequate, we explored other alternatives.

| SG90 | EMAX ES08A II (2026 Nationals) | Savox SV-1232MG (2026 Internationals) |
| :---- | :---- | :---- |
| Softer Plastic gears prone to wear and stripping | Higher Grade Plastic and Robust Motor | Heavy-duty metal gears, aluminum case, robust motor |
| deadband drift \- steering drifts over time | tighter deadband \- precise steering positioning | tighter deadband (3-5 µs) - precise steering positioning |
| speed \- 0.10 sec/60° | speed \- 0.10 sec/60° | speed - 0.04 sec/60° |
| torque \- 1.6 kgf·cm | torque \- 2.0 kgf·cm | torque - 6.5 kgf·cm |

We did have one instance of SG90 breaking last year, so we chose the EMAX servo motor for the 2026 Nationals, primarily for higher durability and secondarily for similar or better steering precision. For the 2026 Internationals, we moved to the Savox SV-1232MG for its metal gears and significantly higher speed and torque.

---

## 2.8 3D Printed Parts

Our robot structure is entirely 3-D designed. Here are the various 3-D parts that connect together to give structure to our robot. Robot assembly instructions are in [Section 6.2](#62-robot-build-instructions)

**[Click Here to see how we iterated to reach these final parts](#551-3d-chassis-iterations)**

  <img src="docs/diagrams/mobility/3DPartsList.png" alt="List of the 3D-printed parts that make up the robot chassis" width="800">

**[Link for all 3-D modelled parts photos ](schemes/)**

**[Link for all 3-D parts  STL files](models/chassis/)**

---

## 2.9 Mobility Inaccuracies and Mitigation

### 2.9.1 Backlash

Backlash is the play between two meshing gears due to which there is a momentary dead zone where one gear moves but the other doesn’t. This impacts drive as well as steering mechanisms.

**Problem:**

1. Drive mechanism: When the robotic car brakes, it continues rolling forward slightly until the gear teeth lock. The overall robot backlash was measured by moving the robot slightly until the motor gear shaft moves and noted to be 6mm.  
2. Steering mechanism: When the servo motor rotates, the wheel doesn’t turn for the initial few degrees due to backlash. The overall steering backlash was measured by turning the wheels slightly until the servo engaged and noted to be 4°.

**Mitigation:**

1. Drive mechanism: We would need an external encoder mounted at the wheel to completely remove backlash effects. However we work around this problem by keeping a buffer of 15mm between robot and wall during precision movements like parking.  
2. Steering mechanism: The robot doesn’t need small precision turns. Moreover the camera vision algorithm uses PI control to reduce the error between target heading and current heading. This mitigates the backlash in the steering system by overcorrecting to account for the backlash.

### 2.9.2 Battery Voltage Variation

The battery voltage reduces with time as the car runs.

**Problem:**

1. The naive way to drive a robotic car is by issuing a PWM signal to the drive motor for a time duration that is proportional to the distance we want to drive. However this will cause inaccuracies in the distance covered as the voltage sags over time.  
2. The naive way to control the speed of a robotic car is by issuing a PWM signal at a specific percentage that has been tuned at a specific battery voltage. However this will cause variations in speed as the voltage sags over time.

**Mitigation:**

1. We control the drive mechanism by encoder distance. This moves the car for the intended distance and is independent of the battery voltage, because the car will drive more/less duration depending on the actual distance covered.  
2. We **control the speed by using PI control to reduce the error between target rpm and the current encoder rpm. This mitigates the speed variations** caused by voltage sag.

---

## 2.10 Experiments

We ran some experiments to determine our robot precision. This data helps us calibrate robot speed for various scenarios like open challenge, obstacle challenge and parking section.

### 2.10.1 Detection Distance vs. Speed

We ran the robot in a straight line until it found an obstacle 20 cm in front of it. The robot started braking at the point the sensor triggered. This measures the time taken by the robot to come to a complete halt. This helps us determine how slowly the robot should move at critical points e.g. parking section.

<img src="docs/diagrams/mobility/DetectionDistanceExperiment.png" alt="Chart of stopping time and stopping distance against robot speed" width="100%">

**Conclusion** This experiment proves that at higher speed the time taken to come to a full stop as well as distance covered before it actually stop increases, so we need to ensure robot is moving at sufficiently slow speed for it to stop and turn accurately in parking.

### 2.10.2 Encoder Precision Experiments

We used the encoder specifications and gear ratio to calibrate the encoder as per [Section 3.9.4](#394-encoder-calibration). Then we measured the actual distance travelled at a range of target distances to verify its accuracy.

| Observation \#  | Encoder measured distance (cm)  | Observed distance practically (cm)  | Δ distance (cm) |
| :---- | :---- | :---- | :---- |
| 1  | 100  | 92  | 8 |
| 2 | 50 | 47 | 3 |
| 3 | 25 | 24.3 | 0.7 |
| 4 | 10 | 11 | \-1 |

Raw runs: [Robotic Car Precision Experiment Data](docs/experiments/RoboticCarBacklashExperimentData.xlsx)

**Conclusion** Encoder distances are accurate at smaller distances which can be effectively used in parking.

---

# 3 Power and Sensor Architecture

An 11.1V 1500mAh 16.65 Wh LiPo battery powers all the electronics on our robot. It drives a 25W power converter providing 5.2V to the Raspberry Pi 5 and a 15W adjustable converter to provide 8V to the Servo motor, while the battery directly provides 11.1V to the Drive motor. The RPi 5 further powers the camera and sensors via its 3.3V GPIO rail.

<img src="docs/diagrams/powerNsense/power_arch_layered.png" alt="Power architecture: LiPo battery feeding a 5.2 V converter for the Raspberry Pi 5, an 8 V converter for the servo, and 11.1 V directly to the drive motor" width="700">

---

## 3.1 Power Budget

We referred to components documentation to find out their voltage and current specifications.

| Devices on 5V power bus | Volts (V) | Idle Current (A) | Typical Current (A) | Peak Current (A) |
| :---- | :---- | :---- | :---- | :---- |
| Raspberry Pi 5 | 5.0 | 0.600 | 0.750 | 1.6 |
| Camera module 3 wide | 3.3 | 0.250 | 0.280 | 0.3 |
| BNO086 IMU | 3.3 | 0.008 | 0.012 | 0.015 |
| VL53L4CD ToF sensor (x4) | 3.3 | 0.020 | 0.080 | 0.16 |
| Total load on 5V power bus | 5.0 | 0.878 | 1.122 | 2.075 |

| Devices on 8V power bus | Volts (V) | Idle Current (A) | Typical Current (A) | Peak Current (A) |
| :---- | :---- | :---- | :---- | :---- |
| Savox Servo- SV1232MG | 8.0 | 0.005 | 0.200 | 2.2 |

| Devices on 11V power bus | Volts (V) | Idle (A) | Typical (A) | Peak Current (A) |
| :---- | :---- | :---- | :---- |:---- |
| Motor driver - TB6612FNG (Logic/VCC) | 5.0 | 0.001 | 0.002 | 0.003 |
| Motor - Pololu 4861 | 11.1 | 0.000 | 0.400 | 1.7 (at 11.1V) |
| Total load on 11V power bus | 11.1 | 0.001 | 0.402 | 1.703 |

\* Current values sourced from component datasheets and other sources.

Using values from the above table, we did **calculation for the need of current and power for our robot**, considering all circuit components.

| Calculation | Idle Current | Typical Current| Peak Current |
| :---- | :---- | :---- |:---- |
| **5V Bus** | | | |
| Output power = Voltage × Current | 5V × 0.878A = 4.39W | 5V × 1.122A = 5.61W | 5V × 2.075A = 10.375W |
| Input power = Output power / Efficiency (95%) | 4.39W / 0.95 = 4.62W | 5.61W / 0.95 = 5.91W | 10.375W / 0.95 = 10.92W |
| **8V Bus** | | | |
| Output power = Voltage × Current | 8V × 0.005A = 0.04W | 8V × 0.2A = 1.6W | 8V × 2.2A = 17.6W |
| Input power = Output power / Efficiency (95%) | 0.04W / 0.95 = 0.04W | 1.6W / 0.95 = 1.68W | 17.6W / 0.95 = 18.53W |
| **11V Bus** | | | |
| Output power = Voltage × Current | 11.1V × 0.001A = 0.011W | 11.1V × 0.402A = 4.462W | 11.1V × 1.703A = 18.903W |
| Input power = Output power / Efficiency (97.5%) | 0.011W / 0.975 = 0.011W | 4.462W / 0.975 = 4.577W | 18.903W / 0.975 = 19.388W |
| **Battery Total** | | | |
| Power drawn = 5V input + 8V input + 11V input | 4.62 W + 0.04W + 0.011W = **4.67W** | 5.91W + 1.68W + 4.577W = **12.17W** | 10.92W + 18.53W + 19.388W = **48.838W** |
| Current drawn = Power / Battery voltage (11.1V) | 4.67W / 11.1V = **0.421A** | 12.17W / 11.1V = **1.096A** | 48.838 / 11.1V = **4.40A** |

---

## 3.2 Power Strategy

### 3.2.1 Battery Runtime Estimation

```
Battery specifications 
Nominal voltage = 11.1V  
Full charge voltage = 12.6V  
Capacity             = 1500mAh  
Discharge rating = 75C  
Max discharge current = capacity * C rating  
                                     = 1.5Ah * 75  
                                     = 112.5A  
Headroom v/s C rating = max discharge current / current drawn from battery  
                                     = 112.5A / 1.096A  
                                     = 102 x (burst that the battery can handle)

Total energy stored = capacity * nominal voltage  
                                = 1.5Ah * 11.1V  
                                = 16.65Wh

Estimated runtime = Total energy / Power drawn from battery (typical)  
                              = 16.65Wh / 12.17W  
                              = 1.37 hours
```
Applying a safety margin of 30% to avoid over discharging the battery, we comfortably get a runtime of over an hour for typical power consumption.
Moreover, we have added a **Battery Level Indicator** (Voltmeter) which allows us to be aware of the battery voltage at all times and we can ensure the battery doesn’t discharge deeply during practice and that it’s fully charged before the competition.

### 3.2.2 Voltage Converter

**25 W/5V Converter**
The 25W/5V converter provides USB compatible output suited for RPi 5's USB power input.  
```
RPi's operational power requirement = 5.91W     	Converter's output power = 25W  
```
This is **more than 4X the required power** to account for spikes.

**15 W/8V Converter**
The 15W/8V converter provides power to Savox Servo Motor  
```
Savox Servo's operational power requirement = 1.68W     	Converter's output power = 15W  
```
This is **more than 8X the required power** to account for spikes.

---

## 3.3 Power Verification

### 3.3.1 Full Circuit Loop Measurement

A multimeter was put in series in the LiPo battery path and the robot run on a raised platform to measure the typical operating current.

| Condition | Measured current (A) | Theoretical current (A) |
| :---- | :---- | :---- |
| Idle operation | 0.350 to 0.420 (across multiple readings) | 0.421 |
| Typical operation | 0.780 to 0.990 (across multiple readings) | 1.096 |

### 3.3.2 5V Power Bus: Onboard Telemetry

We used pmic_read_adc diagnostic command from Raspberry Pi 5 to read real-time voltage and current measurements from its built-in Power Management IC. The telemetry total power almost matches the theoretically calculated power for RPi and sensors connected to it.

[For Telemetry- See Power Profile Script](src/tools/power_profile.py)

<table>
  <thead>
    <tr>
      <th>Multimeter Measurement</th>
      <th>Telemetry Output</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td align="center">
        <img src="docs/diagrams/powerNsense/IMG_1251.jpeg" alt="Multimeter measuring the current drawn by the robot" width="400">
      </td>
      <td align="center">
        <img src="docs/diagrams/powerNsense/Telemetry.png" alt="Raspberry Pi 5 PMIC telemetry output showing the 5 V bus voltage and current" width="400">
      </td>
    </tr>
  </tbody>
</table>

---

## 3.4 Electrical Noise and Mitigation

**Electrical noise sources**
- The robot circuitry has a mix of different loads and we need all of these to perform reliably in spite of the interference they could cause with each other. These loads are:  
  - motors with current spikes when stopped suddenly.  
  - digital circuitry of Raspberry Pi  
  - sensitive sensors on I2C bus

**Mitigation**
- Separation of power rails right at the battery with capacitors on each rail reduces coupling between drive motor, servo motor and RPi. This reduces the likelihood of RPi resets or disruptions due to surge on any motor's input current path. The max spike current of the drive motor (1.8A stall current) can safely be provided by the 11.1V 1.5Ah LiPo battery with a 75C discharge rating (1.5Ah \* 75 \= 112A)  
- PCB ground plane is a continuous layer of copper on PCB connected to ground. It provides a low-resistance path to return current. A near zero resistance avoids any voltage drops on the ground ensuring that the ground is near 0V everywhere on the board.

<img src="docs/diagrams/powerNsense/Power_Capacitors.png" alt="Power architecture with capacitors on each power rail for electrical noise mitigation" width="700">

Apart from electrical noise, the robot also encounters visual interference from sources outside the field. This is covered in [Section 4.6](#46-interference-due-to-surroundings).

---

## 3.5 Electrical Safety: Fuse

An important aspect of safety and fire hazard prevention is short circuit protection. We connect an automotive DC fuse between the battery supply and the power switch. Motor current can spike for short duration during sudden direction reversal and capacitors charging during power up can cause a short duration spike. The fuse should be able to sustain such spikes with sufficient margin to prevent robot failure during the competition and safeguard against short circuit hazards.

**Step A: Fuse Thermal Limit**

- We use Littelfuse 0287010.PXCN rated at 10A 32V DC with nominal I²t of 115 A²s  
- Transients must not exceed 20% of nominal I²t to prevent thermal fatigue over multiple spikes

$AllowedTransientEnergy=0.20\times 115{A}^{2}s\approx 23{A}^{2}s$

**Step B: Worst-Case Circuit Transient**

Suppose the car executes a sudden motor reversal where current surges to **$5A$** across both the drive motor and servo simultaneously for a short duration of **$100ms$ ($0.1s$)**:

$Circuit{\ I}^{2}t={I}^{2}\times t=(5A{)}^{2}\times 0.1s=25\times 0.1=2.5{A}^{2}s$

Even factoring in capacitor charging during power up:

$CapInrush{\ I}^{2}t\approx (25A{)}^{2}\times 0.002s=625\times 0.002=1.25{A}^{2}s$

**Step C: Conclusion**

$CircuitWorstCase\ {I}^{2}t\ (2.5{A}^{2}s)\ll FuseThermalLimit(115{A}^{2}s)$

- **False Tripping due to spikes:** **0% probability.** Our worst-case transients are less than **3%** of the fuse's melting rating.  
- **Short Circuit Protection:** If battery terminals short directly, the 75C LiPo dumps over **100A**. At 100A:

     ${t}_{melt}=\frac{115{A}^{2}s}{(100A{)}^{2}}=0.0115s\approx 11.5ms$  
- The fuse opens almost instantaneously, well before the LiPo battery catches fire.

This makes the 10A fuse an ideal choice to **protect against short circuits**.

---

## 3.6 Wiring Diagram and PCB

In our first draft of the robot, there were many criss-crossing wires between many different components. A lack of clean arrangement for the wires made the robot very messy to handle. During practice runs, wires would routinely get loose, making it difficult to troubleshoot errors.

To solve this, we designed a PCB which cleanly connected all of our components together, making the robot far cleaner and reliable.
Here's  simplified pin layout diagram for the wiring.

<img src="docs/diagrams/powerNsense/Wiring_Diagram.drawio.png" alt="Simplified wiring diagram showing the pin layout of the robot" width="700">

The complete Circuit diagram below shows all power and signal connections between the battery, voltage converters, Raspberry Pi 5, motor driver, servo, camera, IMU, and ToF sensors.

<img src="docs/diagrams/powerNsense/CircuitDiagram_cropped.png" alt="Complete circuit diagram of the battery, voltage converters, Raspberry Pi 5, motor driver, servo, camera, IMU and ToF sensors" width="400">

[Link For Complete Circuit Diagram](schemes/CircuitDiagram.png)

[Link for all PCB KiCAD files](models/PCB)

---

## 3.7 Sensors

### 3.7.1 Sensor Placement Summary

| Sensor | Position | Height | Angle | Mounting Method | Justification |
|---|---|---|---|---|---|
| Camera Module 3 Wide | Rear-center, on pillar | 26 cm | 40 ° down | 3D-printed bracket on rear post | Forward view over chassis; avoids front blind spots; short CSI cable to RPi |
| VL53L4CD #1 | Front | 5.5 cm | 0° (horizontal, forward) | Recessed inside chassis body | Detects front parking wall at short range |
| VL53L4CD #2 | Rear | 6.7 cm | 180° (horizontal, backward) | Recessed inside chassis body | Detects rear parking wall at short range |
| VL53L4CD #3 | Left side | 5.5 cm | 90° left | Recessed inside chassis body | Unused, kept for surprise rule |
| VL53L4CD #4 | Right side | 5.5 cm | 90° right | Recessed inside chassis body | Unused, kept for surprise rule |
| BNO086 IMU | Center chassis | 11.6 cm | Flat  | PCB-mounted, close to center of gravity | Minimum vibration; accurate heading readings |

 <img src="docs/diagrams/powerNsense/sensor_fov_coverage.png" alt="Sensor placement Top View" width="500">

 <img src="docs/diagrams/powerNsense/Sensor_FOV_SideView.png" alt="Sensor placement Side View" width="100%">

---

### 3.7.2 Camera

**Specification Comparison**

| Specification | Raspberry Pi Camera Module 3 Wide | Raspberry Pi HQ Camera + 180° Fisheye lens |
| :---- | :---- | :---- |
| Resolution | 11.9 MP | 12.3 MP |
| Focusing | Auto | Manual |
| Field of View (FOV) | 120° Diagonal (102° Horizontal) | 180° Diagonal (180° Horizontal) |
| HDR support | Up to 3 MP, 30 fps | No |
| Aspect ratio | 16:9 (wider, optimized for videos) | 4:3 (squarish, optimized for still pictures) |

**Why We Chose Camera Module 3 Wide**

- The Camera Module 3 Wide provides auto focus which keeps moving targets sharp.
- The 120° FOV is sufficiently wide and at the same time gives negligible edge distortion.
- The HDR would be useful for low light conditions if encountered.

**Experimentation**
- In the HQ Camera, objects towards the edges become curved and stretched, causing occasional failures in object contour detection. The distortion can be corrected, but the process is computationally intensive, reducing the control loop speed and slows down reaction time.
- In the HQ Camera, objects towards the edges seem compressed and smaller, causing contour area to drop below the detection threshold.
- The HQ camera was too heavy to be mounted at the rear top. Mounting it in the front lower center still covers the field due to its higher FOV, but it reduces the perception of depth as when mounted high, further objects appear higher up in the image but when mounted in front, all objects appear at a similar height.

|Camera Image from Fisheye Camera|Camera Image from RPi 5 Wide Camera|
| :---: | :---:
|<img alt="Sample frame from the fisheye HQ camera, showing stretched and curved objects near the edges" src="docs/diagrams/powerNsense/fisheye_cam.jpg" width="300" >|<img alt="Sample frame from the Raspberry Pi Camera Module 3 Wide" src="docs/diagrams/powerNsense/normal_cam.jpg" width="400">|

**Placement**

1. Mount the camera on the pillar
2. The angle of the camera mount was designed so that the front edge of the robot is visible in the camera. This is essential to avoid blind spots in the front.

<img alt="Camera mount angle that keeps the front edge of the robot visible in the camera field of view" src="docs/diagrams/powerNsense/Camera_FOV.png" width="300" >

---

### 3.7.3 Distance Sensor

**Specification Comparison**

| Specification | VL53L4CD | VL53L1X | VL53L8CX |
| :---- | :---- | :---- | :---- |
| **Zone Architecture** | Single Zone | Single Zone *(Programmable ROI)* | **Multi-Zone (4x4 or 8x8 grid)** |
| **Maximum Range** | Up to 1.3 meters (1300 mm) | **Up to 4.0 meters (4000 mm)** | **Up to 4.0 meters (4000 mm)** |
| **Minimum Range** | **Down to 1 mm** | ~40 mm | ~20 mm |
| **Field of View (FoV)** | 18° Diagonal | 15° to 27° (Adjustable via ROI) | **65° Diagonal** (Wide angle) |
| **Max Sampling Rate** | **Up to 100 Hz** | Up to 50 Hz | Up to 60 Hz |
| **Ambient Light Immunity** | Moderate | Standard | **Excellent** (Uses histogram processing) |
| **Primary Focus** | Ultra-short range precision & speed | Mid-to-Long range point sensing | Scene mapping, multi-target tracking |

**Why We Chose VL53L4CD**

- Our robot uses distance sensors for parallel parking — it must detect parking walls at very short range to avoid collision.
- We need high precision and accuracy at short ranges (down to 1mm minimum range).
- This model has a high 100 Hz update rate, providing faster reaction times compared to VL53L1X (50 Hz) and VL53L8CX (60 Hz).
- Its only disadvantage is its max range is 1.3m unlike others, but that is sufficient for our purpose since we only need to detect walls within parking distance.

**Experimentation**

To figure out the accuracy of our sensor at various distances, we measured its reading vs actual physical distance measurements.

<img alt="Graph of VL53L4CD ToF sensor measurement error against actual distance" src="docs/diagrams/powerNsense/sensor-err-vs-distance.png" >

**Conclusion** The accuracy of the ToF sensor drops at very short distance with a possible blind spot. As a result, we refined our design by placing our sensors inside of the robot chassis to account for the measured blind spot.

**Placement**
Even though the VL53L4CD's datasheet mentions a minimum range of 1mm, its ranging error is higher at short distances of < 20mm. To avoid this issue altogether, we have mounted the sensor recessed inside the robot body on all sides, as much as we physically could. This ensures that the closest distance it needs to measure is more than 15 to 20 mm.

<img alt="VL53L4CD ToF sensor recessed inside the robot chassis" src="docs/diagrams/powerNsense/RecessedToF.jpeg" width="300" >

---

### 3.7.4 IMU

**Specification Comparison**

| Specification / Feature | BNO055 | BNO086 |
| :---- | :---- | :---- |
| **Heading Drift** | High (drifts significantly; loses calibration by the 3rd lap) | Very low drift of 0.5° per minute with dynamic calibration |
| **Dedicated "Game Rotation Vector"** | No | **Yes** (6-DOF: Gyro + Accel) |
| **Hardware Tare (Zeroing)** | No *(Must manually calculate offset)* | **Yes** *(Native firmware command)* |
| **Calibration Architecture** | Continuous Background | **Initial Calibration + Runtime Stillness calibration** |

**Why We Chose BNO086**

The BNO086 features 3 sensors (accelerometer, gyroscope, and magnetometer) and dynamically fuses sensor data. This is primarily used in parking.

- **Accuracy:** Very low drift of 0.5° per minute means the robot can make accurate parking maneuvers even in the 3rd lap.
- **Magnetic Immunity:** The Game Rotation Vector disables the magnetometer. This avoids interference from proximity to motors.
- **Alignment Ease:** Can issue a Tare command once the car is straight in the lane, instantly resetting heading to zero without manual offset calculation.
- **Field Adaptability:** BNO086 lets the host device tell the sensor its exact motion state to improve sensor accuracy by calibrating it for stillness before parking.

**Placement**

IMU is best placed at the center of the robot. We have tried to place it as close to the center as possible given the chassis and PCB limitations.

---

## 3.8 Microcontroller: Raspberry Pi 5 as Single Controller

We used Raspberry Pi 5 as the master brain for our robot. It single handedly manages the navigation loop and using the following important features we have eliminated the need for a co-processor.

- **Used RP1 Southbridge Chip as Hardware Co-Processor to Avoid OS Delays:** A standard Linux OS constantly pauses programs to switch between threads, which can cause skipped encoder pulses or stuttering motor signals. We solved this by offloading our low-level tasks to the Pi 5's built-in RP1 chip, which handles them at the hardware level.

- **Used RP1's Native Hardware PWM Channels for Stable Motor Control:** We connected our motor drivers directly to the RP1's built-in PWM channels. Because these signals are generated by dedicated hardware, they stay perfectly stable and jitter-free for smooth speed control without relying on the busy main CPU.

- **Zero Dropped Encoder Pulses:** We wired our motor encoders into the RP1's Programmable I/O (PIO) blocks. These are independent hardware counters that run completely outside the main CPU, meaning they track every single pulse in the background with microsecond precision.

This design gave us the following **benefits**

- **Less Wiring and No Communication Lag:** By running everything on a single Raspberry Pi 5 instead of adding a secondary board like an ESP32, we simplified our electronics and saved space. This completely eliminated the data lag that usually happens when two different systems try to talk to each other.

- **Accurate Tracking Under Heavy Load:** Because the PIO silicon continuously counts and stores pulses in hardware buffers, we never lose track of our distance. Even when the main processor is heavily loaded with intense image processing tasks, the robot's navigation loop can just read the exact counts whenever it needs them.

---

## 3.9 Calibration Procedures

This section documents how sensors and motors are calibrated.

| Component | What to calibrate | When to calibrate |
| :---- | :---- | :---- |
| Camera | Color tuning | Variation in block colors or field lighting |
| IMU | Calibration of accelerometer and gyroscope | Automatic, during robot start up |
| Encoder | Theoretical calculation on basis of gearbox and wheel size | One time, after assembling the gearbox and wheel |
| Distance sensors | Distance offset calibration | One time for each sensor |
| Servo motor | Center, left and right angles | One time, when steering is assembled |
| Drive motor | Calibration of speed / distance | Automatic, during run time as closed loop control |

### 3.9.1 Camera HSV Threshold Calibration

We use [color_tuning.py](src/tools/color_tuning.py) for live trackbar-based HSV picker:
1. Power up robot on the actual mat under actual lighting
2. Run the tool and a window shows H/S/V sliders per color
3. Pan the robot across the mat so each pillar and line passes through the camera at real angles
4. Adjust bounds until the mask for each target is contiguous and black everywhere else
5. Copy final values into the `HSV_RANGES` dict in [tuning.py](src/obstacle_challenge/tuning.py) used by the main code.

<img src="docs/diagrams/powerNsense/HSV_tuning.png" alt="HSV tuning tool showing trackbars and the resulting color mask" width="500">

**Key rule for BLACK tuning:** Keep S (saturation) low, not V. If you let S go wide, the dark blue mat and dark sides of red pillars get classified as wall, causing non existent walls in mid-corridor.

[Link for HSV Color Calibration Code](src/tools/color_tuning.py)

### 3.9.2 IMU Calibration

The BNO086 does not require complex manual calibration out of the box for basic motion or relative orientation. The BNO086 runs background calibration continuously  to adjust for minor temperature shifts and sensor drift when it is powered up and stationary.

- The BNO086 is enabled for dynamic calibration when the robot is powered up.  
- The sensor triggers calibration only when the robot is stationary. We ensure the robot is kept stationary for a few seconds before pressing the run button.  
- The sensor calibration is disabled when the run button is pressed.

Tare: At startup, the robot is kept parallel to the wall before pressing the run button. The code sets this initial orientation as 0 so that parking is parallel relative to the initial orientation.

[Link for IMU Calibration Troubleshooting code to see calibration status](src/tools/calibrate_bno.py)

### 3.9.3 Distance Sensor Calibration

For Distance calibration, we measure the distance of an object kept 20 cm away. The difference between actual and measured is used as offset in the code, so that software can handle the error created by the sensor reading. Remove the protective cover from new sensors before calibrating.

[Link To Experiment to measure Sensor Accuracy](#373-distance-sensor)

### 3.9.4 Encoder Calibration

Encoder was initially configured with a theoretical multiplier as per wheel diameter, gear ratio, pulses per rotation (PPR) to convert the encoder pulses to distance. This was slightly off so we adjusted the multiplier after running the robot for a fixed distance of 15 cm, aligning with the range used in parking.

### 3.9.5 Servo Motor Calibration

- Assemble the steering mechanism with servo. Change the angle till the wheels are pointing straight and save that angle as `SERVO_CENTER_OFFSET`
[Code Reference](https://github.com/Devansh-awat/greenbotics-2026/blob/2755262822ab1231237eff9866e831d93ff45a32/src/obstacle_challenge/config.py#L92)  
- Turn the wheels to their extreme left and right, till the assembly can safely move without touching the chassis. Keep these values as `clamped_input` [Code Reference](https://github.com/Devansh-awat/greenbotics-2026/blob/2755262822ab1231237eff9866e831d93ff45a32/src/motors/servo.py#L51-L52)

### 3.9.6 Drive Motor Calibration

- We don’t need any drive motor calibration because we run under closed loop that adjusts for battery voltage. [Refer Sec 2.9.2](#292-battery-voltage-variation)

---

## 3.10 Failure Point Analysis

This section details what can go wrong and what we do about it.

| Scenario | Impact | Mitigation |
| :--- | :--- | :--- |
| Lighting variations | Robot crashes into obstacles or wall | HSV tuning to adapt to new surroundings |
| Battery voltage drops | Robot slows down unexpectedly, LiPo could become permanently damaged if it goes below 3V/cell | Voltmeter added to make sure the robot voltage doesn’t go too low |
| ToF sensor dusty | Sensor gives false readings and increase in blind spot | Clean the sensor surface |
| IMU drift | Slanted parking or slanted path when going towards parking blocks | Magnetometer is turned off in IMU to reduce drift occurrences. Recalibrate IMU if it still drifts as per process described in this doc. |
| Camera misses frame | Robot can’t make latest navigation decision | Continue with decision based on previous frame. RISK: Robot could crash if robot in front of obstacle when it misses frame |
| Sensors fail to initialise (Only distance sensor, IMU/Camera never faced any issue) | Robot can navigate obstacles but fail during parking | Sensors are re-initialised with 3 retries |

---

## 3.11 Iteration Evidence

This section shows how the power/sensor design changed over time based on testing.

|Component|Iteration|Details|
|---|--|--|
|Camera|Mounting angle and height changed|[For Camera Mounts Iteration refer 3D Iterations](#551-3d-chassis-iterations)|
|ToF Selection| VL53L1X changed to VL53L4CD to reduce blind spot|[For ToF comparison refer section 3.7.3](#373-distance-sensor)|
|ToF Sensor|Mount position changed to recessed|[For ToF placement reasoning refer Section 3.7.3](#373-distance-sensor)|
|Battery|LiPo 11.1V 2100mAh changed to LiPo 11.1V 1500mAh after Power Budget calculation|[For Power Budget Calculation Refer to Section 3.1 ](#31-power-budget)|

---

# 4 Software Architecture and Obstacle Strategy

The WRO Future Engineers competition has two challenge rounds. Our software architecture handles both using common sensor and motor modules, with the obstacle challenge code being a superset of the open challenge.

| Mode | Entry Point | Purpose |
|------|-------------|---------|
| **Open Challenge** | `src/open_challenge/main.py` | Three full laps on an empty track with random inner walls|
| **Obstacle Challenge** | `src/obstacle_challenge/main.py` | Three laps obeying red/green traffic signs + parallel parking |

Both modes share the same hardware-abstraction modules (`src/sensors/*`, `src/motors/*`) and follow the same architectural template that runs a multi-threaded sense/think/act loop at ~50 fps on the Raspberry Pi 5. The obstacle code adds pillar-aware steering and parking on top of the same wall-following base.

**The arena:** A 3 x 3 m mat with movable inner walls, orange and blue floor lines marking each section boundary, and (in obstacle mode) red and green traffic sign pillars that must be passed on a fixed side, passing right from red,  and left from green. A magenta parking block marks the parking corridor where the robot must parallel-park after 3 laps.

---

## 4.1 Design Philosophy

We chose a **camera-first approach** for obstacle detection, a **multi-threaded architecture** for responsiveness, and **proportional control** for smooth driving.

- **Why camera-first:** The WRO track has colored pillars (red and green) that the robot must pass on specific sides. Only a camera can detect color at a distance. ToF sensors tell us how far walls are, but cannot tell us pillar color.
- **Why multi-threaded:** Reading the camera, IMU, and ToF sensors sequentially would slow the loop to ~15 fps. By reading them in parallel threads, the main loop gets fresh data every frame without waiting.
- **Why single RPi 5:** Instead of using a separate microcontroller for motor control, we use the RPi 5's RP1 hardware PWM and PIO blocks. This eliminates communication lag between two boards (see [Power and Sensor Architecture Section 3.8 for details](#38-microcontroller-raspberry-pi-5-as-single-controller)).
- **Why proportional control (not lane-switching):** Our earlier code in WRO 2025 used discrete lane-switching such as "if red pillar, move to right lane." This was brittle because gyro drift accumulates over 3 laps, and what the robot thought was "right lane" gradually drifted sideways. Our current code drives everything off camera-derived geometric error with proportional gains. Steering changes smoothly with the visual error, adapts continuously instead of waiting for a state change, and works regardless of where exactly a pillar is positioned.

### 4.1.1 Why We Rejected YOLO and Neural Networks

We considered training a YOLO classifier for shadow-robust pillar detection. We decided against it because:
- Annotation cost : even a few hundred labeled frames is several hours of work across lighting conditions
- Inference speed : YOLO on Pi 5 CPU will not hit our 50+ fps budget without adding a Hailo/Coral AI HAT (extra hardware, cost, power)

Instead we rely on tight HSV ranges, carefully placed ROIs, and the priority state machine, so ML classifier is not needed.

---

## 4.2 System Architecture

### 4.2.1 Threading Model

Our software runs 4 background threads so the main navigation loop never waits on hardware. Three threads continuously reads one sensor and the main loop grabs the latest value whenever it needs it.

<img src="docs/diagrams/software/software_system_architecture.drawio.png" alt="System architecture and threading model: camera, IMU and sensor threads feeding the main navigation loop, plus the video writer" width="1000">

### 4.2.2 Code Module Map

This is a high-level map of how files are organized within src folder.

```
src/
├── experiments/          # Standalone data-collection scripts for tuning/characterizing robot behavior
├── logs/                 # Logging setup/configuration for run logs
├── motors/               # Drive motor and steering servo control (PWM, RPM closed-loop)
├── obstacle_challenge/   # Obstacle Challenge entry point, config, and decision/control logic
├── open_challenge/       # Open Challenge entry point and config
├── sensors/              # Hardware-abstraction drivers for camera, IMU, distance sensors, and encoder
├── tests/                # Regression Tests for code refactoring
├── threads/              # Background hardware-sensor thread management
├── tools/                # Calibration and diagnostic utilities (HSV tuning, capture pipelines, sensor tests)
└── vision/               # Shared frame-processing pipeline and worker pool
```

---

## 4.3 Open Challenge Flow Chart and Algorithm

The open challenge uses only the WALL FOLLOW and CORNER TURN behaviours from our architecture. With no traffic signs on the track, the robot drives three laps using the wall-following controller to stay centered, detects corners via the close-black ROI, and counts orange lines to know when 3 laps are complete.

The flow is detailed in the flow chart below:

<img src="docs/diagrams/software/open_challenge_flow.drawio.png" alt="Flow chart of the Open Challenge program" width="900">

The same direction detection, turn counting, and performance optimizations described in later sections apply to both modes. The obstacle challenge code is a superset, adding pillar-aware steering (Sec 4.4.5) and parking (Sec 4.4.8) on top of this same wall-following base.

---

## 4.4 Obstacle Challenge: State Machine and Algorithms

The following sections describe the obstacle challenge, which builds on the open challenge core by adding a priority state machine for pillar avoidance and a parking routine.

**State Machine**

We implement a **flat priority state machine** that is evaluated fresh every frame. We chose this design because it makes the robot's behavior fully predictable from a single frame's sensor data — if you know what the camera and sensors see right now, you know exactly what the robot will do next.

<img src="docs/diagrams/software/navigation_state_machine.drawio.png" alt="Navigation state machine showing the priority order of the robot states" width="900">

### 4.4.1 States and Priority Order

| Priority | State | What the Robot Does | Entry Condition |
|----------|-------|--------------------|-----------------|
| — | **INITIAL MANEUVER** | Detect direction, leave parking zone, and navigate past the first traffic sign | Race start (button press)|
| P0 | **PROCESS NEXT FRAME** | Capture a new camera frame, run the vision pipeline, and evaluate which priority state to enter | After INITIAL MANEUVER completes; re-entered every loop iteration |
| P1 | **AVOID HEADON** | Hard steer to ±35° to where there's more space | Close-black area exceeds 3000px threshold  |
| P2 | **PASS TRAFFIC SIGN** | Target-line geometry steers the robot to keep right from red pillars and left from green pillars | Pillar detected in main camera frame |
| P3 | **CORNER TURN** | Forces a hard steer to turn into the corner |When one outer ROI drops below 100px |
| P4 | **WALL FOLLOW** | PD controller balances left/right wall areas to keep robot centered in corridor | Default state when no higher-priority trigger fires |
| — | **PARKING** | 6-step parallel park using gyro + ToF + camera | Turn counter reaches 13 (3 laps complete) |

When multiple triggers are true in the same frame, the highest priority wins. All states return to PROCESS NEXT FRAME after completion, except PARKING which leads to STOP.

### 4.4.2 INITIAL MANEUVER: Details

The INITIAL MANEUVER state runs once at the start of the race and handles three tasks in sequence:

**1. Direction Detection (camera-based):**
At startup, the robot determines whether the track is clockwise or counter-clockwise using the camera for the first 10 frames:
- Left wall area > Right wall area → outer wall is on left → **clockwise**
- Right wall area > Left wall area → outer wall is on right → **counter-clockwise**

**2. Leave the Parking Zone:**
The robot starts inside the parking corridor against the outer wall. It must exit this corridor before the main navigation loop can begin. The robot undergoes a sequence of tuned steps to exit the parking in controlled manner.

**3. Navigate Past the First Traffic Sign:**
The first pillar can be directly in the robot path, after exiting the parking. To manage these situations, the robot has pre-tuned instructions to pass the traffic sign as per its color and driving direction. Then the main loop takes over to generically handle traffic signs for the next 3 laps.

---

### 4.4.3 PROCESS NEXT FRAME: Computer Vision Pipeline

The images below show what the robot's vision system does to each camera frame — from raw input to final detection output:
<table>
  <colgroup>
    <col style="width: 15%;"> <!-- Step -->
    <col style="width: 35%;"> <!-- What Happens -->
    <col style="width: 50%;"> <!-- Visual (much larger share for bigger images) -->
  </colgroup>
  <thead>
    <tr>
      <th>Step</th>
      <th>What Happens</th>
      <th>Visual</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><strong>1. Raw Frame</strong></td>
      <td>Camera captures 640×360 image of the track</td>
      <td>
        <img src="docs/diagrams/software/pipeline/01_original.png" alt="Step 1: raw 640x360 camera frame of the track" >
      </td>
    </tr>
    <tr>
      <td><strong>2. Convert to HSV</strong></td>
      <td>Colour space that separates hue from brightness — makes colour detection lighting-robust</td>
      <td>
        <img src="docs/diagrams/software/pipeline/02_hsv.png" alt="Step 2: the camera frame converted to HSV color space"  >
      </td>
    </tr>
    <tr>
      <td><strong>3. Colour Masks</strong></td>
      <td>Each colour gets its own binary mask (white = detected). Walls, pillars, and lines are isolated</td>
      <td>
        <div style="display: flex; gap: 4px;">
          <img src="docs/diagrams/software/pipeline/03_mask_black.png" alt="Step 3: binary mask of the black walls" style="width: 50%; height: auto;">
          <img src="docs/diagrams/software/pipeline/03_mask_green.png" alt="Step 3: binary mask of the green pillars" style="width: 50%; height: auto;">
        </div>
      </td>
    </tr>
    <tr>
      <td><strong>4. Detect & Annotate</strong></td>
      <td>Find contours in each mask → report position, area, and colour to the steering logic</td>
      <td>
        <img src="docs/diagrams/software/pipeline/05_final_annotated.png" alt="Step 4: final annotated frame with the detected walls, pillars and lines">
      </td>
    </tr>
  </tbody>
</table>

**ROI Zones**

<img src="docs/diagrams/software/roi_overlay_annotated.png" alt="Camera frame with the region of interest (ROI) zones overlaid" >

### 4.4.4 Avoid HeadOn

If the total area in the close-black ROI exceeds 3000px, a wall is exactly in front. We hard steer to ±35° toward whichever side has more space. This is the only place the wall law ignores its own proportional output.

### 4.4.5 PASS TRAFFIC SIGN: Target-Line Geometry

**Goal:** Steer the robot to keep right from red pillars and keep left from green pillars.

**How it works:**

We define a virtual target line from a corner of the frame to the top center. The robot steers to bring the actual line towards the target line so the pillar stays on the correct side.

<img src="docs/diagrams/software/target_line_geometery.png" alt="Target-line geometry: a virtual line from a corner of the frame to the top center that the robot steers the pillar towards" width="50%">

**Per-frame Steering angle calculation**

```
current_angle = atan2(block_x - origin_x, origin_y - block_y)  // θ = tan_inv(Δx/Δy)
steering_angle = (current_angle - IDEAL_ANGLE) * Kp

where
IDEAL_ANGLE(red) = +42.5 degrees  // tuned for red pillar
IDEAL_ANGLE(green) = -40.5 degrees  // tuned for green pillar
Kp = 1.5  // proportional constant
```

**Evaluation of multiple algorithms**
Our previous year's robot used a **fixed vertical line** as the target line and actual line. If you notice the camera image, the straight walls are seen in camera as inclined lines which means the straight path of robot visually appears as inclined lines due to the depth of vision. Fixed line caused the block to come very close to the robot and to avoid this, we used to change the target line when block used to come very close which was a hacky approach.

Inclined line using angle calculation for **Target line geometry** causes the delta between target and actual to remain almost constant even when the robot approaches the pillar. The robot now steers smoothly like a real world driver would avoid an obstacle. The code is cleaner as there aren't any hacky approaches now making it reliable.

We built a **calibration tool** (`drive_straight_tune_target.py`) that drives the robot straight past a pillar while tracking its centroid frame-by-frame. The path the centroid traces is **not vertical** — because of the camera's forward tilt, a fixed pillar drifts horizontally across the frame as it gets closer. The angle-based law accounts for this drift naturally, making the robot pass cleanly at every range.

**Edge Cases**

**Inner-wall guard:** When `wall_inner_right > 3000` (or left), the block-following angle is clipped to a one-sided range so steering can only turn further *away* from the wall, never into it. This prevents wall contact when passing a pillar near a corner.

**Magenta-coordinated path:** When a magenta parking block is visible along with a pillar, the steering target becomes the midpoint between them which allows us to ensure we do not hit any of the blocks.

**Close Block Avoidance Pillar:** If a pillar gets dangerously close (in the "close block" ROI), normal steering cannot avoid it. We do a hard reverse maneuver.

**Close Block Avoidance Magenta Parking:** Magenta close-blocks are only treated as evasion targets after 5 seconds from race start. Early magenta near the camera is the parking corridor entrance, which we don't want to dodge.

### 4.4.6 Wall Following: PD Controller

**Goal:** When no pillars are visible, keep the robot centered between the left and right walls.

**How it works:**

We use **four ROIs, not two.** Each side of the frame is split into an outer band and an inner band:
- **Outer ROIs** (x: 0..135 and 505..640) sit near frame edges — they feed the wall-balancing law as a simple area difference
- **Inner ROIs** (x: 140..240 and 400..500) sit closer to frame center — they detect a wall that is dead ahead and very close

**Why inner ROIs matter:** When the robot rounds a corner, the far wall slides out of view. If only outer ROIs existed, the area difference would go to zero and the robot would drive straight into the corner wall. The inner ROI catches the close wall and triggers a forced turn before contact.

**Formula:**
```
left_area = wall_left_outer + wall_inner_left
right_area = wall_right_outer + wall_inner_right
wall_error = left_area - right_area
angle = wall_error * Kp + wall_derivative * Kd + bias

Where:
  Kp = 0.0006
  Kd = 0.0003
  bias = +1 (constant offset)
```

### 4.4.7 Corner Turns and Lap Count Logic

**Corner Turn**

When one outer ROI drops below 100px while the opposite side still sees a wall, the robot is entering a corner. We amplify the remaining wall area (2x + 25000) to force a harder turn into the corner instead of understeering.

**Lap Count Logic**

Count orange floor lines to know when 3 laps (12 turns) are complete.

**How it works:**
- Each frame, check if orange is detected in the line ROI
- Store last 4 detections in a queue: `[oldest, ..., newest]`
- Count a turn ONLY when pattern is `[False, True, True, True]` which is a fresh rising edge that persisted for 3 frames
- After counting, apply 50-frame cooldown (prevent double-counting the same line)

**Why did we do this?** At high speed, a single orange line stays in the ROI for ~10 frames. Without the 4-frame rising-edge check and cooldown, one line could be counted multiple times.

### 4.4.8 Parking Algorithm

After completing 3 laps (12 turns), the robot enters the parking sequence. We have two versions: `parking()` for clockwise and `parking2()` for counter-clockwise tracks. The robot uses the IMU for precise turns and small segments of straight driving during parking maneuvers.

**Parking Phases (Clockwise Example)**

<img src="docs/diagrams/software/parking_sequence.png" alt="The phases of the parallel parking maneuver" width="100%">

| Phase | Action | End Condition |
|-------|--------|---------------|
| 1 | Drive forward till orange line | Encoder distance reached (14 cm) |
| 2 | Reverse at -60° steering, heading → INITIAL+90° | Back ToF < 130mm (near outer wall) |
| 3 | Drive forward along wall, camera-based wall-following (Kp=0.8), count magenta pillars | 2nd magenta pillar passed + 0.62s coast |
| 4 | Steer into parking at +55°, heading → INITIAL+100° | Back ToF < 160mm |
| 5 | Forward/reverse maneuver to align parallel | Back ToF ≤ 95mm AND heading ≈ INITIAL±180° |

**Parking Sensors Used**

- **Back ToF:** Knows when robot is close to rear wall of parking slot
- **Front ToF:** Monitors distance to wall during corridor driving
- **IMU heading:** Controls all steering angles precisely
- **Camera:** Follows black wall edge during Phase 3, counts magenta pillar passes

**Goal:** Use the IMU heading to maintain a straight line or execute precise turns.

**IMU Code Reference:**
```
heading_error = target_heading - current_heading
steering = heading_error × Kp

Where:
  Kp = 0.85 (normal driving)
  Kp = 1.0 to 2.0 (parking maneuvers)
```

The gyro handles 360° wrap-around: if the error is > 180°, it wraps to the shorter direction.

Used for:
- Keeping straight during initial maneuver
- 90° turns at corners
- All parking phases (precise angle targets)

---

## 4.5 Edge Cases

| Edge Case | What Could Go Wrong | How We Handle It |
| :--- | :--- | :--- |
| **Same frame processed twice** | Waste processing time | Skip frame if `frame_counter` unchanged |
| **Magenta parking block during startup** | Robot reversal within parking section | 5-second gap on startup before close block reversal activates for magenta parking blocks |
| **Inner wall very close during block follow** | Robot crashes into wall | Servo angle clamped to steer away from wall |
| **Close black wall ahead (no blocks)** | Crash into front wall | If `close_black_area > 3000` → force ±35° turn based on direction |
| **Block disappears mid-avoidance** | Sudden speed change | Grace frames: hold last block speed for 5 frames after block disappears |
| **Wheel stall (blocked/stuck)** | Robot stops moving | RPM controller detects zero RPM → stall recovery |
| **No walls visible, but orange/blue line visible** | Robot will not turn and crash | Force ±35° steering based on track direction |
| **More than 1 Traffic sign in frame** | Robot may follow the wrong block | Closest of the two on y axis is given priority. |

---

## 4.6 Interference Due to Surroundings

The area external to the field can interfere in image processing due to elements outside our control such as carpets or judges walking with red shoes and green pants (yes we did face that in Singapore Internationals in 2025!)

**Mitigation:**
We create a mask which includes white, red, green, orange, blue and magenta. We find the biggest contour in this, and use this to idenfity where the arena is. Later on, we only look for detections in this mask, so that outside interference does not occur.

For parking, we take a similar approach, finding the edge of white mat and black wall, but when the magenta comes into view, blocking the view of the edge, we then use gyro to go straight until the edge can be seen again.

<img src="docs/diagrams/software/Skyline.png" alt="Detection of the edge of the mat (skyline) so that objects outside the field are ignored" width="600">

---

## 4.7 Parameter Tuning

### 4.7.1 All Control Parameters

All tuned parameters are cleanly organized and available in

[Open Challenge Tuned Parameter List](src/open_challenge/config.py)

[Obstacle Challenge Tuned Parameter List](src/obstacle_challenge/config.py)

### 4.7.2 Tuning Process

For tuning, first calibration as per [Section 3.9](#39-calibration-procedures) needs to be done. Then follow the below steps for software tuning.

**Target Line Tuning** Look at the video to see if the robot is oscillating a lot. If it's oscillating too much, increase the Kd. If it takes too long for the target and actual lines to coincide, increase Kp

**Wall Following Tuning** For wall following, look at the robot, and if it is taking too slow turns, then increase the Kp. If it is oscillating wildly, reduce Kp and increase Kd. If it is oscillating mildly, then increase Kd. If it is overturning, decrease Kp

---

## 4.8 RPM Control: PI Controller with Feed-Forward

**Goal:** Maintain consistent wheel speed regardless of load (turns, battery sag).

**How it works:**
- Motor encoder counts pulses via RP1 PIO
- Actual RPM is compared to target RPM
- PI controller adjusts PWM duty cycle
- Feed Forward estimates the exact duty cycle needed to move at the required RPM instead of waiting for the Integral term to catch up

```
Target RPM = 0.7 * MAX_WHEEL_RPM (normal driving)
Kp = 0.20
Ki = 0.50
Ramp limit = 25 RPM increase per frame (prevents wheel spin)
```

### 4.8.1 Why PIO for the Encoder

The motor encoder is driven through the **PIO block** on the RPi 5's RP1 I/O controller. PIO is a small, deterministic state-machine engine sitting next to the main CPU which handles encoder pulses entirely in hardware while the CPU does nothing.

**Why this matters:** Linux is not a real-time OS. The scheduler can preempt our Python loop for tens of milliseconds. If we counted encoder edges from Python, every preemption gap would silently drop counts and reported distance would drift. The PIO state machine sits *outside* the Linux scheduler and **never misses a count** regardless of CPU load.

Two modes of operation:
- `motor.start_rpm_control(target_rpm, direction)` — drives the robot at target RPM (most commonly used)
- `motor.move(distance_cm)` — PID loop that ramps speed up, then decelerates as encoder approaches target distance (used in special situations)

### 4.8.2 Control Loop Diagram

<img src="docs/diagrams/software/rpm_control_loop_v2.drawio.png" alt="RPM control loop: encoder feedback and PI controller driving the drive motor on the Raspberry Pi 5" width="600">

---

## 4.9 Robot Testing

### 4.9.1 Open Challenge Testing

In Open Challenge, the inner walls are randomized to change the track width. This may lead to some tracks being narrow with 600 mm track width or stay wide to be of 1000mm. To test maximum combinations for the robot in limited time, we selected  field that gave us all kinds of turns- big to small, small to small, small to big and big to big in both directions. We tested the robot from different starting sections to cover maximum possibilities. Below is our run log.

<img src="docs/diagrams/software/OpenTestingLog.jpeg" alt="Open testing Log" width="600">

In first round we had couple of close corners and missed orange line count so we tuned our robot and then ran the full set again.

**Test Summary**

| Metric | Round 1 | Round 2 | Total |
| :--- | :--- | :--- | :--- |
| **Run Count** | 16 | 16 | **32** |
| **Average Time** | 21.46 sec | 19.79 sec | **20.63 sec** |

**Competition Practice Round Format Fitment**
```
Average round time ~ 20-25 seconds

per minute - 2 rounds and in 4 mins - 8 rounds

S1-S4 clock and Anti-clockwise in Practice Round 1 and S5-S8 in Practice Round 2
```

### 4.9.2 Obstacle Challenge Testing

The only way to achieve high accuracy for the obstacle challenge was to test against multiple field layouts. This coverage had to be complete and also possible to repeat during practice runs at the competition. Our obstacle testing strategy was spread across 3 phases:

1. **6 difficult layouts**, chosen as per our experience, to validate initial accuracy after hardware assembly and software integration. These were run over several weeks to stabilize the software algorithms.  
2. **24 full coverage layouts**, chosen to test all possible parking exit combinations as that is where most issues arose in phase 1.  
3. **7 optimized layouts**, chosen to combine most combinations from phase 2 with the aim to execute them in 4 timed practice slots at the competition

| Phase |Description | Test scenarios reference | Dates | Runs |Average Time |
| :---- | :---- | :---- | :---- |:---- |:---- |
| 1 | 6 difficult layouts | MS1–MS6, both directions, 2 iterations | 28 Jul – 9 Aug | 24 | **47 sec** |
| 2a | 24 full coverage layouts | C1–C24 \+ C8a, both directions | 21–22 Aug | 55 | **40.2 sec** |
| 2b | Re-verification of (2a) | Re-run of (2a) after fixes | 22 Aug | 50 | Not Measured |
| 3 | 7 optimized layouts | M1–M7, both directions, 3 iterations | 22–24 Aug | 40 |Not Measured|
| **Total** |  |  |**28 Jul – 24 Aug** | **169 runs** | **42 sec**|

Note- After Phase 2a we did not measure time as our focus shifted to accuracy and we were not changing speed any more.

#### 4.9.2.1 Phase 1: 6 Difficult Layouts

Before testing combinations systematically, we built six layouts by experience and printed them on the mat layout using [`generate_tough_configs.py`](test/scripts/generate_tough_configs.py) to cover complex scenarios like
<table style="text-align: center; border-collapse: collapse;">
  <tr>
    <td style="padding: 8px; vertical-align: top;">
      <strong style="font-size: 14px;">1) Tight outer:</strong><br><br>
      <img alt="Phase 1 test layout: tight outer turn" src="test/master_test_layouts/phase1/Tight_Outer_Turn.png" width="80" style="border: 2px solid black;">
    </td>
    <td style="padding: 8px; vertical-align: top;">
      <strong style="font-size: 14px;">2) Tight inner:</strong><br><br>
      <img alt="Phase 1 test layout: tight inner turn" src="test/master_test_layouts/phase1/Tight_Inner_Turn.png" width="80" style="border: 2px solid black;">
    </td>
    <td style="padding: 8px; vertical-align: top;">
      <strong style="font-size: 14px;">3) Maximum zigzag:</strong><br><br>
      <img alt="Phase 1 test layout: maximum zigzag" src="test/master_test_layouts/phase1/ZigZag.png" width="80" style="border: 2px solid black;">
    </td>
    <td style="padding: 8px; vertical-align: top;">
      <strong style="font-size: 14px;">4) Blocks right outside parking</strong><br><br>
      <img alt="Phase 1 test layout: blocks right outside the parking exit" src="test/master_test_layouts/phase1/ToughParkingExit.png" width="80" style="border: 2px solid black;">
    </td>
  </tr>
</table>

Layouts are in [`test/master_test_layouts/phase1/`](test/master_test_layouts/phase1/) and runs were logged on printed sheets of [`WRO_FE_2026_Master_Test_Tracker_v2.pdf`](test/master_test_layouts/phase1/WRO_FE_2026_Master_Test_Tracker_v2.pdf).

Below is an image showing how this round of testing was carried out.

<img src="docs/diagrams/software/TestSheet.png" alt="Phase 1 run sheet for MS1, Maximum Zigzag, both directions" width="900">

Multiple issues were corrected during this phase like wall contacts, near crashes during reversing and even a full crash (MS1). For every failure, we did methodical [Troubleshooting and Debugging](#411-troubleshooting-and-debugging) and improved our tuning parameters as detailed in section [Parameter Tuning](#47-parameter-tuning)

#### 4.9.2.2 Phase 2: 24 Full Coverage Layouts

We realised most issues in phase 1 were during the parking exit. We covered all possible combinations of parking exit by making below combinations:

| Exit Pillar | Color | Adjacent pillar | Layouts |Count|
|---|---|---|---|---|
| P5 | 5G or 5R | P6 (R\G) , P7 (R\G) | C1–C8 | 8 |
| P4 | 4G or 4R | P1 (R\G) , P2 (R\G) | C9–C16 | 8 |
| P3 | 3G or 3R | P1 (R\G) , P2 (R\G) | C17–C24 | 8 |

This gives 24 combinations. We placed the rest of the pillars to cover the difficult combinations like tight outer, tight inner and maximum zigzag from phase 1. We **did not follow 1 pillar randomization rule in most scenarios as the aim was to find tough combinations for the robot.** Each combination was run clockwise and anti-clockwise.

<table>
<tr>
<td><img src="test/field_logs/field_seat_numbering.jpeg" alt="Printed playfield with the 21 pillar seats numbered by hand" width="430"></td>
<td><img src="test/field_logs/coverage_design_C1-C24.jpeg" alt="Handwritten derivation of the 24 full coverage layouts" width="430"></td>
</tr>
<tr>
<td align="center"><em>The 21 numbered seats. Seats 3, 4, 5 are the parking side, left to right.</em></td>
<td align="center"><em>Working out the 24 combinations from the three Parking Pillar Positions</em></td>
</tr>
</table>

Test run Results that were captured in this phase are as below:
<table>
<tr>
<td><img src="test/field_logs/Test_log_phase2_pg1.jpeg" alt="Handwritten Phase 2 run log with layouts and run times" width="430"></td>
<td><img src="test/field_logs/Test_log_phase2_pg2.jpeg" alt="Handwritten Phase 2 issue log and re-verification sign-off" width="430"></td>
</tr>
<tr>
<td align="center"><em>Full layout and run times for all 24 combinations, 21–22 August.</em></td>
<td align="center"><em>Defects from the first testing round, and the fix re-verification pass.</em></td>
</tr>
</table>

Typed layouts, times and defects are in [`test/field_logs/coverage_matrix.md`](test/field_logs/coverage_matrix.md).

**Issue list: 21–22 August testing round**

| Layout | Direction | What happened | Fix verified 22 Aug |
|---|---|---|---|
| C1 | ACW | Hit the left parking marker | ✓ (still a slight touch during final reverse) |
| C2 | ACW | Hit the inner wall in all three laps | ✓ |
| C3 | CW | Hit the left parking marker while reversing; it fell over | ✓ |
| C6 | ACW | Hit the right parking marker in two laps | ✓ |
| C8 | ACW | Hit the left parking marker while parking | ✓ |
| C8 | CW | Hit the green pillar at seat 13 in laps 2 and 3 | ✓ |
| C9 | ACW | Hit the green pillar at seat 16, then the inner wall before parking | ✓ |
| C10 | CW | Touched the left parking marker in lap 1, hit it in lap 2, reversed into it in lap 3 and knocked it down | ✓ |

**What this phase told us:**

- **Parking was our weakest area.** Six of the eight defects happened in parking lot, not in dodging obstacles.
- **Two greens in a row at the exit caused wall contact.** Tight inner turn immediately on parking exit caused issues.

After tuning the robot, we re-tested the failed scenarios, but did not record every re-test, before moving to the final phase.

#### 4.9.2.3 Phase 3: 7 Optimized Layouts

The goal of this phase was to come up with limited layouts which test the majority of the complex combinations with least changes to the field so that we could get max coverage during practice day.

Some of the phase 2 combinations involve a tough clockwise path whereas others involve a tough anti-clockwise path. Since every field is run both ways, **one field can carry a hard anti-clockwise start and a hard clockwise start at once**. So we carefully merged these along with choosing the toughest combinations for the other pillars.

Details of the merging and exact pillar sequences are mentioned in [`test/field_logs/coverage_matrix.md`](test/field_logs/coverage_matrix.md).

Seven layouts in both directions gives 14 runs, which is our stability check that we repeated three times, on three different days before the Nationals:

| Session | When | Runs | Result |
| :---- | :---- | :---- | :---- |
| First test | 22 August, night | 12 (6 of 7 layouts, both directions) | All 12 passed; minor zigzag issues and close turns |
| Full retest | 23 August, afternoon | 14 (all 7 layouts, both directions) | All 14 passed; four notes — parking alignment issues on M3, M5 and M7, close pass to the corner pillar (seat 15\) on M2 |
| Final confirmation | 24 August | 14 (all 7 layouts, both directions) | All 14 passed; no pillar hit and no other parking issues |

**Total across the three Phase 3 sessions: 40 obstacle rounds with zero traffic signs knocked down and proper parking done.**

**Fitting a 4 minute practice slot.** A run takes 40 to 45 seconds, so four runs take about 160 to 180 seconds. We do not move pillars between the clockwise and anti-clockwise run of the same layout, so a practice slot holds two layouts, four runs and **exactly one field changeover**. That changeover is the part we wanted to shorten.

[`optimize_sequence.py`](test/scripts/optimize_sequence.py) counts the number of pillar placements, removals or swaps and checks all 7 layouts from phase 3  to create a sequence with the lowest adjustments.  [`generate_sequence_pdf.py`](test/scripts/generate_sequence_pdf.py) prints the two layouts on a page with the exact moves between them, for example:

```
M1 -> M2 (4 moves): Place 18R; Remove 19(R); Swap 10(R->G), 15(G->R)
```

<img src="test/master_test_layouts/phase3/pillar_sequence_transitions_page1.png" alt="Page 1 of the pillar sequence run-book, showing the M1 to M2 field transition" width="900">

*Page 1 of [`pillar_sequence_transitions.pdf`](test/master_test_layouts/phase3/pillar_sequence_transitions.pdf) — our competition run-book showing two layouts, run clockwise and anti-clockwise (4 runs, \~160–180 seconds), plus one optimized changeover, that fit inside a 4 minute practice slot.*

#### 4.9.2.4 The Scripts Behind This Testing

Five scripts in [`test/scripts/`](test/scripts/) support this process end to end from building the hard layouts, through verifying our own seat-numbering convention, to planning the field changeovers for competition day. Details of these scripts are in [`test/README.md`](test/README.md).

#### 4.9.2.5 Limits of Our Measurements

- **We logged incidents, not counts, from Phase 2 onward.** The Phase 1 metric of pillar touches was not meaningful for us. In Phase 2 even a single touch meant the test case failed and needed correction. So we recorded where the miss happened and did not keep count of pillars hit per round.  
- **We stopped recording run times after Phase 2\.** Once the speed was fixed it was no longer the variable under test, and accuracy became the target.

---

## 4.10 Performance Optimizations

These design choices keep the loop running at 50+ fps on a Raspberry Pi 5:
- **Operate on cropped frame:** Crop the frame before performing BGR to HSV conversion. This saves 30% of conversion cost, and the stripped-out top/bottom rows are irrelevant for robot navigation.
- **Skip processing empty masks:** Check if a mask is empty using `countNonZero` before finding contours. Most color masks are empty, saving significant time if processing is skipped.
- **Process a smaller frame:** Process a 640x360px image instead of 2304x1296 (10 times fewer pixels to process).
- **Multiprocessing:** Utilize all 4 cores on the RPi 5 to reduce workload by having one core process black masks while another handles color masks.
- **Update servo angle only if changed:** Check if the servo angle has changed from the last frame; only command the servo to move if an actual change occurred.

---

## 4.11 Troubleshooting and Debugging

Every run writes a self-contained folder with:
- Annotated MP4 (ROIs, contours, target lines, FPS, turn counter, computed angle)
- Full stdout/stderr log (state transitions, ToF readings, heading deltas)
```
├── 2026-10-27_21-26-08
│   ├── obstacle_output.txt       Logs of Obstacle Run
│   ├── obstacle.mp4              Annotated run of the 3 laps
│   ├── parking.mp4               Annotated video of parking 
│   ├── raw.mp4                   Raw video as captured by camera
│   └── raw.mp4.overlay.jsonl     Annotation Data used in annotating raw video
```

| Regular Obstacle Run Annotated Video | Failure Scenario in Annotated Video |
| :---: | :---: |
| <img src="docs/diagrams/software/regular_run.gif" width="300" alt="Regular Obstacle Run Annotated Video"> | <img src="docs/diagrams/software/obstacle_last5s_crash.gif" width="300" alt="Failure Scenario in Annotated Video"> |

These detailed logs and annotated videos help us to do the post-run analysis to fix the issue observed in the prior run.

---

# 5 Systems Thinking and Engineering Decisions

This section looks at the robot as one system that combines mechanical, electrical and software aspects. Mobility and Mechanical Design (Section 2), Power and Sensor Architecture (Section 3), and Software Architecture (Section 4) each explain why a specific component or algorithm was chosen in isolation. This section looks at the places where a decision in one area forced something in another. We discuss the constraints all three subsystems share, the trade-offs we only discovered once the full robot was tested together, and the failures that needed more than one round of debugging to actually fix.

---

## 5.1 System Overview

This system diagram illustrates how power, data, and mechanical dependencies actually flow between subsystems on the robot.
- Power subsystem provides power to the circuitry across three power rails - 5V, 8V and  11.1 V.
- Sensors sense the environment and provide signals to the Software. (**Sense**)
- Software processes what the sensors see and makes the robot think before taking actions.(**Think**)
- Mobility acts upon the software signals and executes the physical movement.(**Act**)

**Here is a sample cycle**
- The camera sees the surroundings and provides raw frames to RPi 5
- The encoder provides pulses to the RP1 controller which provides the accumulated counts to RPi 5
- The software does image processing to determine which state it is in and decide on the robot movements
- The software provides appropriate PWM signals for the servo motor and the drive motor to the RP1 controller
- The RP1 controller provides the signals to the servo motor and to the motor driver.
- The motor driver passes on appropriate external voltage as per PWM signal to the motor.

<img src="docs/diagrams/systemsThinking/master_system_diagram_simplified.drawio.png" alt="System integration diagram showing how the power, sensor, software and mobility subsystems connect" width="1100">

---

## 5.2 Shared Constraints

A few constraints don't belong to any single subsystem. They impact the robot's design across subsystems.

### 5.2.1 The No-Differential-Drive Rule

WRO's rule against a differential-drive robot is the constraint for multiple aspects:

Mobility: It pushed us towards a front-steering vehicle. For a front steering vehicle, the parallel parking requirement makes turning radius dictate the length of the chassis (R = L / sin(θ)). The outer radius grows further with the width (R(outer) = R(center) + W/2). Hence we tightly packed back to back the differential, drive motor, and steering assembly, building a compact robot

Power: The power and PCB components also need to be tightly packed to fit this compact sized robot.

### 5.2.2 Vehicle Weight Distribution

<table>
  <tr>
    <td valign="top" width="50%" style="padding-right: 15px;">
    It seems intuitive that a heavier robot needs more driving force, and therefore a more powerful motor. It also seems intuitive that more weight gives better traction and grip. So how do we balance the two? <br><br>
    Both forces, driving and traction, turn out to be proportional to mass. Adding weight to improve traction therefore gains nothing, because it raises the driving force by the same amount and demands more motor power. The right approach is to avoid dead weight and instead concentrate the weight over the drive axle, where traction is needed.<br><br>
    Our robot places most of its weight at the rear by mounting the motor next to the metal gearbox, which adds further weight over the rear drive axle.<br><br>
    We have also kept the robot’s overall weight as less as possible, which helps in several ways:<br><br>
    1. Mobility: A lighter robot needs a less powerful motor.<br>
    2. Power: A less powerful motor draws less current, so a smaller battery is enough.<br>
    </td>
    <td valign="top" width="50%" style="padding-left: 15px; border-left: 1px solid #ddd;">
      <b>F<sub>traction,max</sub> = μN<sub>w</sub></b><br><br>
      where, N<sub>w</sub> = normal force acting directly on the driven wheels<br>
      μ = coefficient of friction<br><br>
      For a vehicle with total mass <i>m</i> distributed such that a fraction <i>W<sub>d</sub></i> of the weight rests on the drive axle(s):<br><br>
      <b>N<sub>w</sub> = W<sub>d</sub>mg</b><br>
      <b>F<sub>traction,max</sub> = μW<sub>d</sub>mg</b><br>
      <b>F<sub>drive</sub> ≤ F<sub>traction,max</sub></b><br><br>
      Since, F<sub>drive</sub> = ma<br><br>
      <b>ma ≤ μW<sub>d</sub>mg</b><br>
      <b>a<sub>max</sub> = μW<sub>d</sub>g</b><br>
      Thus, maximum acceleration is mass-independent
    </td>
  </tr>
</table>

### 5.2.3 Power Budget

The full power budget identified in Power and Sensor Architecture gives us a runtime of over an hour. Building a robot within this constraint also impacted

Software: The image detection algorithm could have been improved with YOLO, but it would have required an AI HAT on the RPi increasing the power budget further, which would require a higher powered battery, hence increasing the weight.

Mobility: A higher weight slows down the robot.

Thus multiple decisions are interlinked. Hence we limited ourselves to a software algorithm that could run on the RPi 5 itself.

### 5.2.4 Control Loop Timing

The robot **Sense - Think - Act** control loop needs to be really fast to make quick decisions the moment it sees things else the robot may bang into obstacles or the wall. This single constraint dictates:

Software architecture: If we collect the sensor data in the main control loop, it would slow down the system drastically. We run a multi threaded system where dedicated threads collect & process the Camera frames, IMU readings, Sensor readings. This keeps the control loop free to think and act efficiently. While the main thread is doing this, the other threads collect and process the data in parallel. This architecture works because RPi 5 has a quad core CPU where 4 threads can run in parallel.

Power and Sensor architecture: Sensors need fast sampling rate to be able to quickly feed back changes in distances during parking to take advantage of a fast control loop. Similarly, a camera with a fast fps would feed back environment changes rapidly.

Mobility: The servo motor needs fast reaction time so that it can quickly act on the steering instructions.

### 5.2.5 Surprise Rule Readiness Constraints

- We cleaned up and organised our main control loop of over 800 lines into a modular functional structure so that we could easily and quickly attempt a surprise rule at the WRO 2026 Internationals. [Link to GitHub](https://github.com/Devansh-awat/greenbotics-2026/commit/58e9a06d53069c2477347f23d81708d0ec8e82ec)
- We also made speed a global config so that we can easily adjust our robot speed with a single config. [Link to GitHub](https://github.com/Devansh-awat/greenbotics-2026/blob/db2b09ddc2027b6339b1da074000733b988c35a6/src/open_challenge/config.py#L6)
- Added left and right ToF sensors which could be used in case the need arises in the surprise rule. 

---

## 5.3 Cross-Subsystem Decision Case Studies

These are decisions that cannot be fully explained from inside a single subsystem document. Each decision only makes sense once you look at what two or three subsystems are impacted at the same time.

### 5.3.1 One Raspberry Pi 5 Instead of a Dual-Controller Architecture

We used RP1, the RPi's onboard IO controller to provide jitter free PWM output and accurate Encoder pulse counts. This avoids the typical scheduling jitter associated with offloading these to a separate thread on Linux.

The alternative to use a second microcontroller dedicated to motor tasks was rejected because of the native features of RPi 5.

The decision is a systems decision, because three different constraints point the same way:

- **Mobility** needed precise encoder counting for sensitive parallel parking maneuvers. Losing pulses during micro-adjustments would make parking difficult.

- **Power and Sensors** had fewer boards to wire, power, and mount.

- **Software** needed the control loop to run at fast fps. An onboard controller avoids the lag associated with cross-board communication.

Using the RPi 5's RP1 controller answers all three at once: it counts every encoder edge in the onboard controller regardless of CPU load, hardware PWM never stutters even when the vision pipeline is busy, and there is only one board's worth of wiring and power distribution.

### 5.3.2 Speed vs. Accuracy: Why a Lower Speed Improved Lap Time

**Trigger:** Increasing the drive speed from 40% to 70% kept accuracy at the same 100% across 5 runs each. The robot avoided all obstacles in both scenarios. But the overall lap time got worse, not better, despite the higher speed.

**Diagnosis:** At 70% speed, the robot was reaching pillars before it could be finished steering to the correct side, triggering the close-block emergency evasion routine more often than at 40%. Each evasion is a full stop-reverse-recover cycle, and costs more time than the higher speed saved.

**Decision:** We wanted 100% accuracy with lower laptime. So we moved to 50%, where accuracy as well as lap times both came to their best. **Exactly like a real world car driver would follow a healthy speed limit and does not drive rashly.**

**Verification:** Medium speed produced a faster overall lap time than fast speed which led to reversals.

| Speed | No. of backoffs (avg) | Time taken in seconds (avg) |
| :---- | :---- | :---- |
| 40% | 0 | 51 |
| 70% | 3-4 | 59 |
| **50%** | **0** | **45** |

---

## 5.4 Risk and Failure Modes

This section outlines the engineering process we followed when the robot broke mid-project. These failures needed debugging across multiple aspects of the system, sometimes software, sometimes hardware and sometimes both!

### 5.4.1 Drive Motor Burnout: TT-GM25 to Pololu 4861

**Trigger:** The TT-GM25 drive motor began stalling intermittently, initially once in a few days, then finally four times during a single run.

**Diagnosis:**
**1)** We put a multimeter across the driver output when the robot stalled four times and noticed it was getting 6.5V. This confirmed that software and wiring was fine.

**2)** The motor terminals still showed continuity with occasional breaks.
**3)** We opened the motor casing and found dark marks on the steel shaft that remained even after cleaning it, probably signs of physical wear inside the motor.

<img src="docs/diagrams/systemsThinking/MotorBurnt.jpeg" alt="Burnt TT-GM25 drive motor with dark wear marks on its steel shaft" width="200">

**Alternatives considered:** Replace it with another TT-GM25 of the same design, which repeats the same risk.

**Decision:** We moved to a motor from a reliable brand instead of staying with the same design, and reused the speed/torque comparison from [Section 2.6](#26-speed-torque-calculation-drive-motor-selection) to pick the Pololu 4861 specifically, rather than just any Pololu model.

**Verification:** The Pololu 4861 has run for over three months of testing with zero stalls, against a TT-GM25 that was already failing within weeks.

### 5.4.2 ToF Sensor Dropout During Parking

**Trigger:** Roughly once in every 20 parking runs, the robot failed to stop during a parking maneuver and hit the wall instead of braking.

**Diagnosis:** Run logs showed the distance sensor reading None at the moment of failure. This confirmed that the sensor was not responding and it wasn't a logic error in the parking routine.

**Alternatives considered:**

**1)** We assumed initialisation failure at startup time, so added a retry until successful check during the sensor initialisation at startup. Still it failed, implying that sensor was starting successfully, but dropping out mid run.

**2)** We added a fallback initialisation during sensor reads. If a sensor failed in the middle of a run, it would be re-initialised. Still it failed, confirming this wasn't a software problem.

**3)** The actual cause turned out to be hardware. The Raspberry Pi GPIO pin driving that sensor defaults to an internal Pull-Down, while the sensor needs a Pull-Up.

**Decision:** Reconfigured the GPIO pin as Pull-Up in the Pi's config.

**Verification:** Zero recurrence over 200+ runs against a roughly 1-in-20 failure rate in spite of the two software-level fixes.

### 5.4.3 Wheel Wobble Showing Up as Camera Jitter

**Trigger:** No wobble was visible in the robot's physical driving, but the camera feed showed a wavy motion while the robot was moving.

**Diagnosis:** Traced to the wheel-axle fit inside the 3D-printed gearbox, which does not hold the wheel centred as precisely as the LEGO chassis it replaced. That play was too small to notice by watching the chassis, but was visible when closely observing the shaft. This is amplified as the camera is mounted higher up on a pillar.

**Alternatives considered:** A software correction would have increased the main loop processing time, besides the issue wasn't significantly affecting driving.

**Decision:** Tried to fix it mechanically by adding a plastic bracket fixed with the gearbox that supports the wheel with a second axle point.

<table style="width: 100%; text-align: center; border-collapse: collapse;">
  <tr>
    <th>Wheel Bracket (Back)</th>
    <th>Wheel Bracket (Side)</th>
  </tr>
  <tr>
    <td style="padding: 10px;">
      <img src="docs/diagrams/systemsThinking/Wheel_bracket_back.jpeg" alt="Wheel bracket fitted to the gearbox, back view" style="width: 200px; height: 200px;">
    </td>
    <td style="padding: 10px;">
      <img src="docs/diagrams/systemsThinking/WheelBracketSide.jpeg" alt="Wheel bracket fitted to the gearbox, side view" style="width: 200px; height: 200px;">
    </td>
  </tr>
</table>

**Verification:**

A comparison of the videos before and after the fix shows some improvement in wobble, though it is not completely eliminated.

|Before|After|
| :---: | :---:
|<img src="docs/diagrams/systemsThinking/MoreeWobble.gif" alt="Before the fix: camera view with more wheel wobble" width="400">|<img src="docs/diagrams/systemsThinking/LessWobble.gif" alt="After the fix: camera view with less wheel wobble" width="400">|

**Unresolved but known risk and limitation**
The camera image is still wobbly. However this slight wobble causes 1-2 frame misses sporadically. In case a few frames are missed when the robot is close to a block, it may hit the block. However we haven’t ever encountered this problem so we are leaving this as a known issue.

### 5.4.4 Hardware Inaccuracies and Software Mitigations

Limitations in hardware cause inaccuracies which required special handling in software.
The drive motor has backlash which affects parking accuracy. This is compensated by measuring the effect of backlash on the vehicle and keeping 2x buffer for parking.

The servo motor backlash and voltage sag adds 3 inaccuracies which we can compensate by closed loop driving
- Servo motor backlash: target steering inaccuracy (details discussed in [Section 2.9](#29-mobility-inaccuracies-and-mitigation) )
- Voltage sag: target distance inaccuracy
- Voltage sag: target speed inaccuracy

Here is how all the 3 inaccuracies are handled with a closed loop system.

<img src="docs/diagrams/systemsThinking/closed_loop_concept.png" alt="Closed-loop control concept that compensates steering backlash and battery voltage sag" width="700">

---

## 5.5 Iteration and Testing Cycle Summary

### 5.5.1 3D Chassis Iterations

We have worked on this robot since Jan 2026 and gone over multiple iterations with respect to various components. These changes required changes to the 3D parts as well and we designed multiple variations of each of our chassis parts to accommodate these changes.

<img src="docs/diagrams/systemsThinking/3D_Graveyard_1.png" alt="Earlier discarded iterations of the 3D-printed chassis parts, part 1" width="700" >

<img src="docs/diagrams/systemsThinking/3D_graveyard_2.png" alt="Earlier discarded iterations of the 3D-printed chassis parts, part 2" width="700">

From WRO 2025 we have 2 complete robot iterations, where first we moved from Lego Chassis to 3 D printed chassis, but kept the bigger back Wheels then we moved to the current design with all 4 thinner wheels, and battery at the back for better stability. Here's how our 3 robots look:

<img src="docs/diagrams/systemsThinking/3_robos.png" alt="Robot Iterations Full chassis" width="700">


### 5.5.2 Other Design Iterations

Pulled from all three subsystem sections plus the case studies and failures above, this table is a compact record of what changed, why, and what evidence backs the change.

| Decision | Changed From → To | Why | Evidence / Status |
| :---- | :---- | :---- | :---- |
| Steering | Parallel → Ackermann | Tyre slip during tight maneuvers between inner blocks and walls | [Refer Sec 2.3 for Details](#23-steering-selection) |
| Drive motor | LEGO EV3 → TT-GM25 | LEGO motor was run at 100% speed and had no headroom to increase robot speed. | [Refer Sec 2.6 for Details](#26-speed-torque-calculation-drive-motor-selection) |
| Drive motor | TT-GM25 → Pololu 4861 | TT-GM25 failed in a month. Chose Pololu motor for reliability | [Refer Sec 5.4 for Details](#541-drive-motor-burnout-tt-gm25-to-pololu-4861)|
| Steering motor | SG90 → EMAX ES08A II  | Need more precision and robustness | [Refer Sec 2.7 for Details](#27-steering-motor-selection)|
| Steering motor | EMAX ES08A II → SAVOX SV 1232MG  | Low reaction time to execute turns | [Refer Sec 2.7 for Details](#27-steering-motor-selection)|
| Pillar-avoidance steering law | Fixed vertical target line → angle-based target-line geometry|The delta between the robot and obstacle reduced as the robot approached the pillar with Fixed-line delta, forcing hacky edge-case handling| [Centroid-tracking calibration tool](src/tools/drive_straight_tune_target.py)  [Section 4.4.5](#445-pass-traffic-sign-target-line-geometry)|
| Drive speed | Fixed speed (40 or 70%) → Medium Speed with lower lap time | Fixed high speed caused repeated back-off recoveries, worsening lap time |[Details for Speeds](#532-speed-vs-accuracy-why-a-lower-speed-improved-lap-time) |
| ToF sensor init | Boot-time check only → runtime re-init → GPIO Pull-Up fix | Wall collisions in ~1-in-20 parking runs | [Details for ToF sensor Dropout](#542-tof-sensor-dropout-during-parking) |
| Wheel-gearbox alignment | Bare 3D-printed gearbox → added axle-support bracket | Wobble at the wheel showed up as jitter in the camera feed | [Details of Wheel Gearbox alignment issue](#543-wheel-wobble-showing-up-as-camera-jitter)|

---

## 5.6 Cross-Subsystem Troubleshooting

When the robot fails, methodical troubleshooting is required that spans hardware and software:

| Sr. No. | Problem | Troubleshooting |
|---|---|---|
| 1 | The robot does not power on. No LEDs turn on | Check if power switch is in ON position<br>Check if fuse is blown |
| 2 | The robot powers off abruptly<br>The robot is extremely slow or erratic | Check battery voltage on the onboard voltmeter > 10V |
| 3 | The robot doesn’t move ahead | Check wires connecting drive motor to PCB<br>Run [`src/motors/motor.py`](src/motors/motor.py) (`python3 -m src.motors.motor`, with the wheels off the ground) to test the motor is moving |
| 4 | The robot doesn’t turn | Check wires connecting servo motor to PCB<br>Run [`src/motors/servo.py`](src/motors/servo.py) (`python3 -m src.motors.servo`) to test the servo is moving |
| 5 | The robot hits the wall during parking | Check wires connecting sensors to PCB<br>Run [`src/tools/sensor_test.py`](src/tools/sensor_test.py) (`python3 -m src.tools.sensor_test`) to test sensors are working |
| 6 | The robot hits the blocks during obstacle challenge | Check [Section 4.11](#411-troubleshooting-and-debugging) for software troubleshooting |

---

# 6 Reproducibility and GitHub Organization

Our robot is fully reproducible and this section outlines all the details required to build the robot from scratch. It explains our git file structure, how to build the robot, how to setup the software and how to test it.

---

## 6.1 Repository Structure and Module Map

This section gives a high-level map of the GitHub repository. It highlights the purpose of each folder and what kind of files reside in it. Each folder also has its own README which details the purpose of files within that folder.

```
greenbotics-2026/
├── docs/           # Diagrams and photos used in this README, experiment data, and the printable Engineering Journal PDF
├── matlab/         # MATLAB Open and Obstacle Challenge prototype and simulation report assets
├── models/         # 3D-printable chassis STL files and KiCad PCB project
├── schemes/        # Wiring diagram, chassis renders, mount reference images
├── src/            # Python code that runs on the Raspberry Pi 5 (open/obstacle challenge, motors, sensors, tools, etc.)
├── t-photos/       # Team photo(s)
├── test/           # Obstacle field testing: layouts, run logs, and the scripts that generate them
├── v-photos/       # Vehicle photos (top/bottom/front/back/left/right + base plate)
├── video/          # Link(s) to performance videos
├── CHANGELOG.md    # Version history of the code
├── README.md       # Project Documentation
└── .gitignore
```

### 6.1.1 Folder Guide

| Folder | Short Description | README |
|---|---|---|
| `docs/` | Supporting diagrams (with `.drawio` sources) and photos used in this README, experiment data (`experiments/`) and the printable Engineering Journal PDF (`Engineering_Journal/`). | [docs/README.md](docs/README.md) |
| `matlab/` | MATLAB Open Challenge algorithm prototype and simulation report images. | [matlab/README.md](matlab/README.md) |
| `models/` | 3D-printable chassis STL files and the KiCad PCB project (schematic + layout) for the custom PCB. | [models/README.md](models/README.md) |
| `schemes/` | Wiring diagram, chassis build-step renders, and 3D-part mount reference images. | [schemes/README.md](schemes/README.md) |
| `src/` | Python code that runs on the Raspberry Pi 5 — Open Challenge and Obstacle Challenge entry points, motor/sensor/vision hardware-abstraction modules, calibration tools, and standalone experiment scripts. | [src/README.md](src/README.md) |
| `t-photos/` | Team photos. | [t-photos/README.md](t-photos/README.md) |
| `test/` | Obstacle field testing (Section 4.9.2): printed test layouts and run-sheet, handwritten and typed run logs, input configs, and the Python scripts that generate the layouts and the competition-day run-book. | [test/README.md](test/README.md) |
| `v-photos/` | Vehicle photos — Top, Bottom, Front, Back, Left, Right, Base Plate  | [v-photos/README.md](v-photos/README.md) |
| `video/` | Link(s) to Open Challenge and Obstacle Challenge performance videos (YouTube). | [video/README.md](video/README.md) |

---

## 6.2 Robot Build Instructions

**Step 1: Print the 3D parts**

- STL files for wheel shaft, bottom chassis, camera mount, differential gear box top case, motor shaft, bumper, top chassis, servo horn and wheel bracket are in models/chassis/.  
- The print settings for Bambu Lab A1 are saved in the 3MF project file [models/chassis/All_3D_Parts.3mf](models/chassis/All_3D_Parts.3mf).  
- If you use a different printer and cannot import above settings, follow the settings below:

**Global Slicer Settings**

- **Material:** PLA  
- **Layer Height:** 0.2mm  
- **Infill Pattern:** Gyroid  
- **Walls:** 2 Wall Loops  
- **Infill density:** 15%

**Part Overrides**

- **Camera Mount:** 25% Infill  
- **Bearing to Wheel Shaft:** 4 Wall Loops  
- **Motor Shaft:** 100% Infill | 4 Wall Loops  
- **Bumper:** TPU(Flex) | 7% Infill

After you have printed all parts, you should have all of the parts shown in below image:

<img src="docs/diagrams/mobility/3DPartsList.png" alt="List of the 3D-printed parts that make up the robot chassis" width="1000">

**Step 2: Assemble the Ackermann steering**

Build the steering assembly using the LEGO parts as shown in schemes/

Ackermann steering was built using LEGO components and a custom 3D part for Variable Servo Horn, which was **specially designed through trial and testing** so that it may accommodate a shifting LEGO screw in the backside for Ackermann. Light sanding of the holes should be done to ensure assembly moves freely, before attaching servo motor.

<table style="width: 100%; text-align: center; border-collapse: collapse;">
   <tr>
    <td style="padding: 10px;">
      <img src="docs/diagrams/gitNReproducibility/Ackerman.jpeg" alt="Ackermann steering assembly built from LEGO parts" width="500">
    </td>
    <td style="padding: 10px;">
      <img src="docs/diagrams/gitNReproducibility/AckermanAssembly.jpeg" alt="Ackermann steering assembly with the variable servo horn attached to the servo" width="500">
    </td>
  </tr>
</table>

**Step 3: Assemble the Robot**

- Use appropriate screws at each step to affix individual parts. Use below image to as visual guide for each step.
- Assemble the 3D printed parts as per Step 3A and attach Pololu 4861 motor and the Metal Differential Gear in the rear cavity.  
- Insert the Ackermann steering assembly in the front cavity.  
- Assemble the 3D printed parts as per Step 3B and attach Savox SV-1232MG.
- Assemble the 3D printed parts as per Step 3C and 3D.
- Attach front and rear wheels.

<img src="docs/diagrams/gitNReproducibility/3D_assembly.png" alt="Assembly of the 3D-printed robot parts" width="800">

**Step 4: Fabricate the PCB**

- Fabricate the PCB from the KiCad files in models/PCB/ using KiCad.

<img src="docs/diagrams/gitNReproducibility/PCB.png" alt="Custom PCB designed in KiCad" width="300">

**Step 5: Solder the electronics with safe sequence**

- Solder all headers to the PCB.  
- Solder the power modules.
- Attach Raspberry Pi 5.  
- Attach the battery and verify the Raspberry Pi 5 boots.  
- Solder the startup switch and LED and verify with a simple test program.  
- Attach the motor driver module and verify motor control with a test program. [`src/motors/motor.py`](src/motors/motor.py) [`src/motors/servo.py`](src/motors/servo.py)
- Attach the sensor modules and verify sensor readings with a test program. [`src/tools/sensor_test.py`](src/tools/sensor_test.py)
- Attach the IMU module and verify orientation readings with a test program. [`src/tools/calibrate_bno.py`](src/tools/calibrate_bno.py)
- Attach the Raspberry Pi 5 wide-angle camera.

<img src="docs/diagrams/gitNReproducibility/PCB_soldered.jpeg" alt="Components soldered onto the custom PCB" width="600">

**Step 6: Install the software**

- See [Software Setup](#63-software-setup-and-running-the-robot) below for detailed steps on setting up software

**Step 7: Verify robot runs\!**

- Run the robot as per steps in [Testing Workflow](#64-testing-workflow) and it will run smoothly\!  
- If the rear wheel shaft doesn’t move smoothly with your hands, apply grease to the gear teeth using the greasing hole underside the robot chassis.

---

## 6.3 Software Setup and Running the Robot

### 6.3.1 Flash and Configure Raspberry Pi OS

- Install Raspberry Pi OS (Trixie, 64-bit) on the Raspberry Pi 5.
- Connect to Wi-Fi and confirm internet access, then update the system:
```bash
sudo apt update
sudo apt upgrade
```

### 6.3.2 Install Git and Clone Greenbotics Repo

```bash
sudo apt install -y git
git clone https://github.com/Devansh-awat/greenbotics-2026.git
cd greenbotics-2026
```

### 6.3.3 Install System-Level Dependencies

```bash
sudo apt install -y python3-pip python3-gpiozero python3-lgpio \
  python3-picamera2 python3-numpy python3-opencv python3-tk  \
  python3-pil.imagetk python3-libcamera i2c-tools libi2c-dev python3-dev
```

### 6.3.4 Install Python Dependencies

```bash
pip3 install --break-system-packages -r src/requirements.txt
```

### 6.3.5 Enable I2C Bus and PWM

Enable I2C bus using following command
```
sudo raspi-config nonint do_i2c 0 
```

Enable PWM, i2c-2 and configure pull ups by adding the below at the end of `/boot/firmware/config.txt` using `nano`

```
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=100000
dtoverlay=i2c3-pi5,pins_14_15
gpio=4,5,14,15=pu
dtoverlay=pwm-2chan
gpio=23=pu
```
Then `sudo reboot`

### 6.3.6 Run the Code

```bash
cd /path/to/greenbotics-2026

# Open Challenge
python3 -m src.open_challenge.main

# Obstacle Challenge
python3 -m src.obstacle_challenge.main
```

---

## 6.4 Testing Workflow

Having built the robot using [Section 6.2](#62-robot-build-instructions) and [Section 6.3](#63-software-setup-and-running-the-robot) above, follow below instructions to test it on the field.

**Open Challenge test procedure**
1. Place robot in the starting section on a WRO FE mat with its walls.
2. Run `python3 -m src.open_challenge.main`.
3. Confirm: 3 laps completed, correct direction, no wall contact, stop in finish section.

**Obstacle Challenge test procedure**
1. Place traffic signs (red/green) in a valid randomized configuration.
2. Run `python3 -m src.obstacle_challenge.main`.
3. Confirm: correct avoidance (go from right when there's red pillar, go from left when there's green pillar), no pillar contact, 3 laps completed, parking attempted.

---

## 6.5 Parts List (Bill of Materials)

This is a consolidated list of every major component used to build the robot, as described across the Mobility, Power & Sensor, and Reproducibility sections of this document.

| Category  | Component | Link | Quantity | Price (INR) |
| :---- | :---- | :---- | :---- | :---- |
| Compute  | Raspberry Pi 5 | [link](https://robu.in/product/raspberry-pi-5-model-4gb/) | 1 | 13409 |
| Drive Motor  | Pololu 4861 (25D metal gearmotor, 12V, 1800 RPM, with Encoder) | [link](https://www.pololu.com/product/4861) | 1 | 5195 |
| Steering Motor  | Savox SV-1232MG | [link](https://www.teamsavox.com/product/sv-1232mg/) | 1 | 8500 |
| Differential  | Mirana Metal differential gear, 38:13 ratio | [link](https://www.amazon.in/dp/B0F1CQ1T1R) | 1 | 799 |
| Wheels  | LEGO SPIKE Prime medium wheels, 56mm | [link](https://www.brickowl.com/ca/catalog/lego-wheel-56-x-14-with-medium-azure-tire-39367) | 4 | 1000 |
| Chassis & Structural Parts  | Custom 3D-printed parts (PLA) |  | multiple | 2000 |
| Steering Linkage  | LEGO Ackermann assembly \+ Variable Servo Horn |  | multiple | 250 |
| Battery  | Bonka 11.1V 1500mAh LiPo | [link](https://xyzhobby.com/product/bonka-u2-series-1500mah-75c-3s1p-11-1v-lipo-battery-xt-60-plug-1-pcs/) | 1 | 2655 |
| Voltage Regulator  | 25W step-down converter (12V → 5.2V) | [link](https://robu.in/product/24v-12v-to-5v-5a-power-module-dc-dc-xy-3606-power-converter/) | 1 | 139 |
| Voltage Regulator  | 15W adjustable step-down converter (12V → 8V) | [link](https://electronicspices.com/product/4-5-24v-to-0-8-17v-dc-to-dc-step-down-converter) | 1 | 69 |
| Motor Driver  | TB6612FNG | [link](https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/) | 1 | 149 |
| Custom PCB  | Greenbotics designed PCB | [link](https://pcbpower.com/) | 1 | 3000 |
| Camera  | Raspberry Pi Camera Module 3 Wide | [link](https://robu.in/product/raspberry-pi-camera-module-3-wide/) | 1 | 4173 |
| Distance Sensor  | VL53L4CD ToF sensor | [link](https://robu.in/product/smartelex-distance-sensor-1-3-meter-vl53l4cd/) | 4 | 1676 |
| IMU  | BNO086 | [link](https://robu.in/product/7semi-bno086-9-dof-orientation-vr-imu-fusion-breakout-qwiic/) | 1 | 2619 |
| Battery Monitor  | Voltmeter | [link](https://robu.in/product/0-28inch-3-5-30v-two-wire-dc-voltmeter-red/) | 1 | 119 |
| Misc  | Wires, PCB headers, resistors, capacitors, fuse, switch |  | multiple | 1000 |
| TOTAL (INR)|  |  |  | **₹46752** |
| TOTAL (USD) |  |  |  | **$491** |

---

## 6.6 Engineering Journal

The Engineering Journal is a printer friendly PDF export of the soft copy version of this README.md. You can find it in [docs/Engineering_Journal](docs/Engineering_Journal/Engineering_Journal.pdf).

Summary of our documentation is available in [Presentation PDF](docs/Engineering_Journal/Greenbotics_Presentation.pdf).

---

## 6.7 Matlab Journal

The Matlab code and journal is in the [matlab folder](matlab/) of this repository.
