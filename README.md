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

> **Status (2026-11-06):** S1a and S1b done: servo timing, and a static error model over the workspace at P = 32, checked with a 100 g payload. Next: S1c (compensation).

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
| **S2** | The kinematic error: touch points on a printed grid, fit joint zero offsets (and check link lengths) | Offsets, before / after error |
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

## Hardware

- The SO-101 follower and leader, as in M0/M1 (device paths in so101-core's git-ignored `config/devices.yaml`).
- ZED 2 on a fixed mount (works as a plain UVC stereo camera, 2x 1280x720 at 30 fps). Its current view shows the
  room; it needs reframing before any image goes into this public repo.
- Wrist camera (configured as `wrist`).
- A rigid ChArUco board (printed, mounted flat): still to get.

## Decisions

See [DECISIONS.md](DECISIONS.md).
