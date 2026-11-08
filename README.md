# so101-calib

M2 of the [SO-101 series](https://github.com/hujunhan/so101-core). It calibrates the arm and its cameras and
splits the end-effector error into its sources:

- **Servo system identification**: latency, gravity sag (steady-state error), deadband and backlash.
- **Camera intrinsics**: the wrist camera with a ChArUco board; the ZED 2 checked against its factory
  calibration.
- **Hand-eye calibration**: eye-in-hand (wrist camera) and eye-to-hand (ZED 2).
- **An error budget**: end-effector error split into camera, kinematics and servo terms.

Demo target: click a point in the ZED image and the gripper tip goes there; an error heat map over the
workspace, in mm.

> **Status (2026-11-07):** S1 done: servo timing, a static error model over the workspace at P = 32 (checked with a 100 g payload), and feed-forward compensation that cuts the tracking error on M1's circle 3-5x (0.6-0.7 mm median). S2 (kinematic calibration) tried and stopped: the uncalibrated model is about 8 mm RMS off, and the touch method could not get below 3.5 mm, see below. S3 done: wrist camera calibrated (0.23 px); the ZED 2's factory calibration confirmed (depth metric to ~0.3 %). S4 done: the elbow's zero is off by -7.3 deg (one offset; held-out error 4x lower, S2's touches 8 -> 4.5 mm); wrist camera and ZED located, table height checked to ~1 mm. Next: S5 (error budget, click-to-point).

## What M1 hands over

From [so101-kinematics](https://github.com/hujunhan/so101-kinematics) (README, "Handed to M2"):

- **The model and the arm disagree.** A torque-off ruler check gave 8.66 cm by FK for 10 cm on the table, with
  the heights consistent to 1.6 mm. A calibration zero offset of a few degrees on the elbow or wrist would explain
  it; two points cannot say which.
- **A joint model of the servos**, fitted on four drawing runs: about 100 ms delay on every joint (part of it is
  the measurement), 0.9 deg of elbow sag, 0.1-0.4 deg of deadband. Not yet compensated: a feed-forward run
  (command ~100 ms ahead, add the sag) is the direct test.
- Cold-start IK seeding and the compromise for an unreachable pitch: open, and needed later for grasping (M5).

## Plan

| Session | Content | Output |
|---|---|---|
| **S1** | Servo identification through `SafeArm`: step and sine tests per joint; delay, sag against pose and load, deadband, backlash; the feed-forward test from M1 | Servo model, compensation result |
| **S2** | The kinematic error: touch points on an iPad grid, fit joint zero offsets. **Tried and stopped** (see the S2 section) | Uncalibrated error measured; no offsets applied |
| **S3** | Cameras: wrist-camera intrinsics (ChArUco); ZED 2 factory calibration check; ZED SDK vs raw UVC | Intrinsics in YAML |
| **S4** | Hand-eye: eye-in-hand (wrist) and eye-to-hand (ZED 2) | `T_<to>_<from>` in YAML |
| **S5** | Error budget and demo: click in the ZED image, the tip goes there; heat map over the workspace | Demo, heat map |
| **S6** | Write-up: README, `docs/DESIGN.md` | |

## S1a result: servo timing and static error (one pose)

`so101-servo run` holds the arm at a test pose (`[0, -34, 34, 34, 0]` deg, tool ~29 cm out, ~13 cm up) and
moves one joint at a time: steps of 0.5-1.5 deg, then 4 deg sines at 0.2-1.0 Hz. Commands go out at 30 Hz;
`SafeArm.follow(dense=True)` reads the arm every 1.0 ms in between (median). Full tables and plots:
[results/s1/servo_20261106_184915.md](results/s1/servo_20261106_184915.md).

| Joint | First motion after a step (ms) | Error at sine top / bottom (deg) | Center (static offset) | Half-width (band) |
|---|---:|---:|---:|---:|
| shoulder_pan | 56 | +0.37 / -0.43 | -0.03 | 0.40 |
| shoulder_lift | 48 | -0.07 / -1.66 | -0.87 | 0.80 |
| elbow_flex | 55 | -0.07 / -3.01 | -1.54 | 1.47 |
| wrist_flex | 53 | +0.17 / -0.83 | -0.33 | 0.50 |
| wrist_roll | 52 | +0.46 / -0.37 | +0.04 | 0.41 |

Error = commanded - measured at the turning points of the sine. They do not change with frequency
(0.2-1.0 Hz), so they are static, not a delay.

- **The servo first moves 50-55 ms after a command** on every joint, and is halfway at about 70 ms.
  M1's "100 ms delay" was this plus half a tick from the staircase command (17 ms) plus the deadband below.
- **Every joint has a band of about +-0.4 deg** where it stops and stays (pan and roll, which carry no gravity
  load, show it alone). 0.5 deg steps often do not move the joint at all.
- **Under gravity the band is offset and wider.** Moving with gravity, lift, elbow and wrist reach the command;
  moving against it they stop short: the elbow by 3.0 deg, the lift by 1.7 deg. M1's single "0.9 deg sag"
  number averaged over this. At M1's drawing start pose the elbow error was -1.7 to -1.9 deg, the same sign and
  size as here.
- **The deadband makes a speed-dependent lag.** The command must get ~0.4 deg ahead before the servo moves:
  90 ms at 5 deg/s, 18 ms at 25 deg/s. This is why the sine "delay" grows at low frequency (wrist_roll: 157 ms
  at 0.2 Hz, 111 ms at 1 Hz), and on the elbow the sine fit is no longer a delay at all.
- The servos run with LeRobot's position gain P = 16 (factory 32; LeRobot lowered it "to avoid shakiness").

### Same test at P = 32

[results/s1/servo_20261106_185955_p32.md](results/s1/servo_20261106_185955_p32.md)

| Joint | Center P16 -> P32 (deg) | Half-width P16 -> P32 | Error against gravity P16 -> P32 | First motion (ms) | Rest jitter P32 (deg p-p) |
|---|---:|---:|---:|---:|---:|
| shoulder_pan | -0.03 -> -0.03 | 0.40 -> 0.09 | - | 56 -> 49 | 0.00 |
| shoulder_lift | -0.87 -> -0.41 | 0.80 -> 0.27 | -1.66 -> -0.68 | 48 -> 47 | 0.00 |
| elbow_flex | -1.54 -> -0.81 | 1.47 -> 0.74 | -3.01 -> -1.55 | 55 -> 50 | 0.00 |
| wrist_flex | -0.33 -> -0.15 | 0.50 -> 0.26 | -0.83 -> -0.41 | 53 -> 40 | 0.00 |
| wrist_roll | +0.04 -> +0.03 | 0.41 -> 0.16 | - | 52 -> 45 | 0.09 (one count) |

- **Doubling P halves the gravity error** (elbow 3.0 -> 1.55 deg, lift 1.7 -> 0.7 deg), as expected for a
  proportional servo: the torque that holds the arm needs half the error.
- **The rest band also shrinks, by 2-4x.** Before the run I expected it would not, after a report of a
  built-in 10-count deadband on the STS3215 ([SO-ARM100 #134](https://github.com/TheRobotStudio/SO-ARM100/issues/134)).
  On these servos (`CW_Dead_Zone` = `CCW_Dead_Zone` = 1 count) the band depends on the gain, so it is mostly
  friction against P x error, not a fixed deadband.
- **No oscillation at rest, no temperature rise** in the 4 min run (32-34 C before and after). Holding torque
  against gravity is the same at any P (only the error needed to make it changes), so steady heating should
  not change; oscillation would, and none showed.
- **One sign of less damping:** shoulder_pan, which turns the whole arm, overshoots 1.5 deg steps by up to
  ~0.35 deg and then drifts back. The other joints do not.
- The sine "delay" drops from ~115 to ~78 ms at 1 Hz on pan and roll, because the band the command must cross
  first is smaller.

## S1b result: static error over the workspace (P = 32)

`so101-statics run` visits 16 poses chosen to spread the gravity torques (tool 12-44 cm forward, 9-51 cm up;
`so101-statics preview` renders them to [results/s1/statics_poses.png](results/s1/statics_poses.png)). At each
pose every arm joint approaches from 3 deg above and from 3 deg below, twice, settles, and is read for 1 s.
The holding torque tau of each joint comes from the URDF masses (Pinocchio). Full table and plot:
[results/s1/statics_20261106_192427.md](results/s1/statics_20261106_192427.md).

Model: error (command - measured) = c * tau + side * (b + k * |tau|) + o, with side = +1 from above, -1 from below.

| Joint | c (deg/N m) | Band b + k\|tau\| (deg) | o (deg) | RMS error before | after (constant band) | after |
|---|---:|---|---:|---:|---:|---:|
| shoulder_pan | - | 0.12 | -0.01 | 0.14 | - | 0.07 |
| shoulder_lift | 1.65 | 0.09 + 0.68 \|tau\| | +0.10 | 0.62 | 0.18 | 0.09 |
| elbow_flex | 2.14 | 0.03 + 1.25 \|tau\| | +0.22 | 0.66 | 0.14 | 0.08 |
| wrist_flex | 2.14 | 0.13 + 0.66 \|tau\| | +0.11 | 0.21 | 0.06 | 0.05 |
| wrist_roll | - | 0.14 | -0.02 | 0.15 | - | 0.05 |

- **The spring model holds over the workspace.** One compliance per joint (1.65-2.14 deg/N m) and the URDF
  torques explain the error at all 16 poses, including where the shoulder torque changes sign. What is left is
  0.05-0.09 deg RMS, about one encoder count (0.088 deg). Each pose was held to within 0.03 deg (std).
- **The band grows with the load.** With a constant band the fit leaves 0.14-0.18 deg RMS on lift and elbow;
  letting the half-width grow with |tau| halves that. Gear friction grows with the torque the gears carry. On
  the elbow the band is almost all load-dependent (0.03 + 1.25 |tau|): moving with gravity the joint nearly
  reaches the command (slope 2.14 - 1.25 = 0.9 deg/N m), moving against it it stops short (3.4 deg/N m).
- **Consistent with S1a.** At the S1a test pose (elbow tau = -0.43 N m) the model gives -0.71 deg center and
  0.57 deg half-width for the elbow; the S1a sines at P = 32 gave -0.81 and 0.74 (measured while moving).
- The constant offsets o (+0.10 to +0.22 deg) are small but not zero; a torque offset the URDF does not have
  (cables) could cause them. Not investigated.
- The servo's Present_Load register follows the model torque on the shoulder (r = -0.84) but only loosely on
  the elbow and wrist (-0.41, -0.53): the model torque is the better input.
- One temperature read of 41 C on wrist_roll was a single glitch (35 C before and after); all servos stayed at
  32-36 C.

### With a 100 g payload

Same 16 poses with a 99.7 g box (9 x 6.5 x 1.8 cm) taped flat to the outside of the fixed jaw, its end flush
with the jaw tip; its centre of mass enters the torque model as a point mass at (-3.4, 0, -5.9) cm in the
gripper frame. The payload widens the torque range by about half (lift down to -1.06 N m, elbow -0.70, wrist
-0.24). Table: [results/s1/statics_20261106_214336_100g.md](results/s1/statics_20261106_214336_100g.md).

The test that matters: the model fitted **without** the payload, applied to the payload run with the new
torques.

| Joint | RMS error | no-payload model | its mean residual | refit on both runs | c, both runs (deg/N m) |
|---|---:|---:|---:|---:|---:|
| shoulder_pan | 0.17 | 0.08 | +0.01 | 0.07 | - |
| shoulder_lift | 0.84 | 0.14 | +0.04 | 0.11 | 1.62 |
| elbow_flex | 0.91 | 0.21 | +0.14 | 0.11 | 1.79 |
| wrist_flex | 0.34 | 0.08 | +0.04 | 0.06 | 1.81 |
| wrist_roll | 0.16 | 0.06 | +0.01 | 0.06 | - |

- **The model carries over to a load it was not fitted on**: 0.84-0.91 deg RMS on lift and elbow drop to
  0.14-0.21 deg. So the error is the servo's response to torque, not something tied to those poses.
- **The elbow compliance was overestimated** without the payload (2.14), where its torque spanned only
  -0.45 to -0.15 N m; the payload run alone gives 1.98 and both runs together 1.79, leaving 0.11 deg RMS.
  Fitted on both runs, the three gravity joints have similar compliances (1.6-1.8 deg/N m).
- Pan and roll are unchanged (band 0.12-0.16 deg), as expected without gravity torque.
- Temperatures 32-36 C; one 42 C read on wrist_roll was again a single glitch.
- For S1c: use the fit on both runs.

## S1c result: feed-forward compensation on M1's circle (P = 32)

`so101-track run` draws M1's circle (radius 3.6 cm, plane 20 cm in front of the base, in the air) with four
command variants, each adding one part of the S1 model, at 2 and 5 cm/s, interleaved, twice (16 circles, all
complete). The model is fitted on both S1b runs ([results/s1/servo_model.json](results/s1/servo_model.json)).
The arm is read about every 1 ms and compared with the target at the time of each read (M1 read once per tick,
right after the command, which added a tick of apparent lag). Tool positions are FK of the measured and the
target joints: how well the servos follow, not absolute accuracy (S2). Full table and plot:
[results/s1/track_20261106_215728.md](results/s1/track_20261106_215728.md).

| Variant | Command | 2 cm/s median / max (mm) | 5 cm/s median / max (mm) | Predicted median (virtual servo) |
|---|---|---:|---:|---:|
| none | target (M1) | 2.0 / 5.4 | 3.8 / 6.8 | 2.3 / 4.6 |
| spring | + c tau + o | 2.0 / 4.1 | 4.2 / 5.5 | 2.4 / 4.8 |
| band | + side (b + k \|tau\|) | 1.4 / 2.5 | 3.5 / 4.6 | 1.6 / 4.1 |
| full | the above, 80 ms ahead | **0.6 / 1.8** | **0.7 / 1.7** | 0.2 / 0.5 |

- **Full compensation cuts the tracking error 3-5x**: median 2.0 -> 0.6 mm at 2 cm/s, 3.8 -> 0.7 mm at
  5 cm/s; the max 5.4-6.8 -> 1.7-1.8 mm. With it the error hardly depends on speed.
- **The model predicted the ranking and roughly the size beforehand.** `so101-track plan` runs the commands
  through a virtual servo (the S1 model with a 65 ms delay); the real arm matches it within 0.5 mm except
  that "full" stays at 0.6-0.7 mm instead of 0.2-0.5: what the model does not have.
- **The parts do different jobs.** spring removes the constant offset (mean tool offset 1.0-1.4 -> 0.3 mm,
  mostly vertical) and halves the static error after stopping (0.93 -> 0.40 deg), but at this pose the sag
  is small (elbow 0.4 deg) and the delay dominates while moving, so the median does not change. band removes
  the error that flips with direction. Only the lead removes the delay, which is most of the error at 5 cm/s.
- **The lead is slightly long**: the residual delay is -18 ms (2 cm/s) and -10 ms (5 cm/s), so 65-70 ms would
  fit better (realigned medians 0.45-0.5 mm; not run).
- Repeats agree within 0.05 mm. wrist_roll, which does not move on the circle, stays 0.15 deg off: it rests
  in its band where move_to left it (no motion, so no side to compensate). Temperatures 36-38 C.
- Better than M1's numbers (circle 3.8 / 6.1 mm median) already without compensation: P = 32 and reads that
  do not add a tick of lag.

## S2: kinematic calibration, tried and stopped

Goal: fit the joint zero offsets (LeRobot's calibration takes each zero from a hand-placed "middle" pose), so
that FK matches the real tool. `so101-touch` records the joint angles while a pointer touches the dots of a
grid shown on an iPad (5 cm pitch, exact pixels), then fits the offsets of shoulder_lift, elbow and wrist_flex,
the pointer tip and the grid placement by least squares. The pointer was a toothpick taped to the fixed jaw.
Data: `results/s2/` (all four runs, including the aborted ones).

| Run | Method | Touches | RMS / max, offsets 0 (mm) | RMS / max, fitted (mm) | Offsets lift / elbow / wrist (deg) | Pointer tip in the gripper frame (mm) |
|---|---|---:|---:|---:|---|---|
| 22:41 | limp arm, guided by hand | 27 | 8.1 / 14.6 | 3.8 / 10.3 | -1.5 +- 0.6 / -4.6 +- 0.8 / -2.2 +- 0.7 | [-11.0, -2.2, -118.6] |
| 23:09 | limp arm, guided by hand | 12 | 8.5 / 12.2 | 3.5 / 7.7 | -3.3 +- 0.9 / -6.1 +- 1.6 / -1.4 +- 1.0 | [-14.0, 0.2, -110.8] |
| 23:29 | teleop (leader arm), P = 32 | 2 | - | - | stopped | - |
| 23:34 | teleop (leader arm), P = 32 | 1 | - | - | stopped | - |

What happened:

- **The uncalibrated arm is about 8 mm RMS off (max 12-15 mm)** in the touched region (tool 25-35 cm out, on
  the table), even with the pointer and the grid placement fitted in its favour. This is the one external
  accuracy number M2 has; it agrees with M1's ruler check (10 cm read as 8.66 cm).
- **The fit left 3.5-3.8 mm, against about 1 mm expected** from the simulation in `so101-touch plan`. 2-2.7 mm
  of it was vertical although the tip was on the glass. Adding link lengths, axis tilts or joint scales to the
  model did not help (3.8 -> 3.7 mm), so the residual is random per touch, not a missing parameter.
- **The two hand runs disagree** by about 2 deg on lift and 1.6 deg on elbow (about 1-2 standard errors), and
  **the fitted pointer tip moved 8.7 mm between them**, 7.8 mm of it along the toothpick: the toothpick slid in
  its tape. It can also move within a run, which no fit can follow.
- On a limp arm the user had to hold the arm and push the last millimetres; the forces bend the printed parts
  and open play that the encoders do not see. Teleoperation was meant to remove that (the follower holds itself
  at P = 32, and the S1 servo model flags a push), but two attempts were stopped after 1-2 touches because the
  toothpick still was not fixed firmly enough.

**Decision: stop here** (DECISIONS.md, 2026-11-06). Nothing was written to the joint map; the arm runs with
LeRobot's calibration. With a cheap printed arm, joints that settle anywhere within a 0.1-0.7 deg band (S1b) and
a pointer that cannot be fixed rigidly, the result would be offsets known to a degree or two and a model still
several mm off; that is not worth more sessions. What remains for the rest of M2:

- **Absolute accuracy, uncalibrated: about 8 mm RMS, up to ~15 mm.** The error budget (S5) uses this.
- Part of it is a smooth shift that a hand-eye calibration against this arm (S4) absorbs for camera-guided
  work, which is what M3 needs. Repeatability (S1: about 0.1 deg per joint after the model, under 1 mm at the
  tip) matters more there than absolute accuracy.
- **Repeatability, from S1b's encoders** (FK of the measured joints, so not counting flex the encoders do not
  see): the same pose reached the same way twice lands 0.2-0.4 mm apart (median; max 2-4 mm); reached from
  opposite sides it lands 7 mm apart (median, up to 16-21 mm), and gravity sags the tip 3-5 mm (up to 12-17 mm
  stretched out or loaded). S1's model removes most of both: 0.6-0.7 mm on M1's circle (S1c).
- If it is ever revisited: a rigid pointer (a printed tip screwed to the gripper, or the fixed jaw tip, whose
  position is in the mesh, 6.3 mm beyond the tool frame), teleoperation, or the camera measuring the arm in S4.

## S3 result: camera intrinsics (2026-11-07)

`so101-cam` shows an 8 x 5 ChArUco board (25 mm squares, pixel for pixel at 264 ppi; ruler-checked) on an
iPad, takes views automatically when the board is still and the pose is new, and calibrates with OpenCV
(5-parameter distortion). The ZED 2 is read over UVC (DECISIONS); its factory calibration was downloaded from
Stereolabs and compared. Reports: [wrist](results/s3/wrist_20261107_001917.md),
[ZED](results/s3/zed_20261107_002443.md); parameters: `results/s3/*_intrinsics.yaml`.

| Camera | Views | RMS / cross-validated (px) | fx, fy (px) | cx, cy (px) | Notes |
|---|---:|---|---|---|---|
| wrist (USB, 1280 x 720) | 21 | 0.23 / 0.24 | 732.0, 730.9 (+- 2.3) | 680.1, 339.9 | 82 x 52 deg; distortion up to ~50 px in the left corners |
| ZED left, ours | 17 | 0.33 / 0.37 | 535.2, 535.2 (+- 2.0) | 638.0, 349.1 | |
| ZED left, factory | 17 | 0.43 on our views | 534.7, 534.9 | 638.1, 348.4 | |
| ZED right, ours | 16 | 0.34 / 0.49 | 549.2, 549.2 (+- 4.5) | 654.3, 353.2 | fewer, partial views |
| ZED right, factory | 17 | 0.52 on our views | 534.0, 534.1 | 647.7, 354.2 | |

| ZED stereo | Baseline (mm) | 25 mm squares measured by triangulation |
|---|---:|---:|
| ours | 122.8 | 25.01 mm |
| factory | 120.0 | 25.08 mm (+0.3 %) |

- **The ZED's factory calibration holds; M2 uses it.** Our left eye agrees with it within 0.1 % in focal length
  and 1 px in the centre, and the factory parameters reproject our views almost as well as our own fit does
  on views it was not fitted on (0.43 vs 0.37 px). Triangulating the board with the factory pair measures its
  25 mm squares as 25.08 mm: depth from the ZED is metric to about 0.3 % (the iPad's scale was checked with a
  ruler to about 0.5 %). Our right eye (fx 549, 3 sigma from the factory's 534) and our baseline (122.8 mm) are
  less certain: the right eye saw the board partly, near the image edge, in many views.
- **The wrist camera is calibrated for the first time**: 0.23 px, and 0.24 px on views left out, so it does
  not overfit. Its centre is 40 px right of the image centre, and its distortion moves the image corners by up
  to ~50 px, so undistorting matters near the edges. The left image corners had few board views: a bootstrap
  over the views puts the ray direction there to ~14-21 px (1.5-3 deg), against ~1 px in the centre and 4-5 px
  at the other edges. Enough for objects in front of the gripper; 5-6 more views in the left corners would fix
  it (`so101-cam calibrate` merges folders).
- Two detector fixes came from the first real frames: refining missed markers (`tryRefineMarkers`; 10.7 -> 19.8
  of 20 markers with iPad glare and the gripper in view), and rejecting views whose corners lie on one line (a
  ZED view with 7 corners in a row and 1 below wrecked the right eye's fit).
- Simulation guided how to capture: 15 views covering the whole image are as good as 30, while views only in
  the centre are ~10x worse at the edges (the distortion is extrapolated there).

## S4 result: hand-eye and joint offsets (2026-11-07)

`so101-handeye run`: the iPad board lies still on the table; the user poses the limp arm once over it, then the
arm moves itself through 8 small and 16-20 spread poses (collision-checked against the iPad) and the wrist
camera takes a view at each. Two runs, the iPad placed anew for each. `so101-handeye fit` fits one wrist camera
in the gripper (X), one board placement per run (Y), and joint zero offsets, to all corner pixels; offset
models are compared on views left out (4-fold) and on a whole run left out (X and offsets from one run, only
the board refitted on the other). Report: [handeye_20261107_221446_x2](results/s4/handeye_20261107_221446_x2.md)
(first run alone: [handeye_20261107_215410](results/s4/handeye_20261107_215410.md)); parameters:
`results/s4/handeye.yaml`.

| Offsets fitted | Left-out run (px) | Board centre, camera vs FK: median / max (mm) | lift / elbow / wrist (deg) |
|---|---:|---|---|
| none | 19.4 | 8.5 / 15.0 | - |
| **elbow** | **4.7** | 1.5 / 3.9 | - / **-7.28 +- 0.11** / - |
| lift + elbow | 4.7 | 1.4 / 3.6 | +1.5 +- 0.9 / -8.1 +- 0.5 / - |
| lift + elbow + wrist | 4.7 | 1.3 / 3.4 | +1.0 +- 0.9 / -6.2 +- 0.7 / -1.7 +- 0.4 |

- **The elbow's zero is off by -7.3 deg; nothing else is needed.** One offset predicts a run it was not fitted
  on 4x better (19.4 -> 4.7 px). Adding lift and wrist_flex offsets does not predict better; they only trade
  against each other and against the camera mount (fitted per run they swung by up to 3 deg: the first run
  alone gave lift / elbow / wrist +0.6 / -5.3 / -2.8, the second -0.2 / -7.1 / +0.4). LeRobot sets each zero
  from a pose the user judges by eye, so degrees are expected.
- **S2's touches confirm it independently.** With the elbow offset fixed, refitting only the pointer and the dot
  grid's placement, S2's hand-guided touches drop from 8.1 / 8.5 to 4.3 / 4.7 mm RMS (two runs), and the error
  in distances between touched dots from 8.2 / 9.2 to 3.9 / 3.1 mm, close to the ~3.5 mm S2's own fits reached
  (the toothpick slipped). Different method, different day.
- **The table height checks out.** Board height minus the measured iPad + case (11 mm) puts the table at -1.4 and
  -3.1 mm in the base frame; the URDF's base bottom is at -2.4 mm.
- **Wrist camera in the gripper frame** (X): [-4.0, 70.9, -3.3] mm, looking ~25 deg off the gripper axis.
- **The ZED** (left eye) is at [38.6, 27.3, 38.7] cm in the base frame, looking down at ~63 deg, fitted to its
  views of both placements (1.0 px). From each run alone it lands 8 mm and 1.3 deg apart, which moves a pixel's
  ray-table point by 2.6 mm median (5 mm max) over the workspace: the tilt of a 20 x 12 cm board at 50 cm is
  weakly fixed by one view. More placements (S5) tie it down. The pose holds until the ZED or the arm moves.
- Usable views: 15 / 24 and 19 / 28. The jaws cover the lower centre of the wrist image (now in the pose
  planner) and the iPad glass reflects the room at oblique views. The iPad must be dimmed (~50 %) or the ZED
  saturates it.
- Not yet applied to so101-core's joint map: M2's own code uses `results/s4/handeye.yaml`.

## Hardware

- The SO-101 follower and leader, as in M0/M1 (device paths in so101-core's git-ignored `config/devices.yaml`).
- ZED 2 on a fixed mount (works as a plain UVC stereo camera, 2x 1280x720 at 30 fps). Its current view shows the
  room; it needs reframing before any image goes into this public repo.
- Wrist camera (configured as `wrist`).
- A rigid ChArUco board (printed, mounted flat): still to get.

## Decisions

See [DECISIONS.md](DECISIONS.md).
