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

> **Status (2026-11-06):** repository set up; session order agreed (S1 first). Nothing calibrated yet.

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

## Hardware

- The SO-101 follower and leader, as in M0/M1 (device paths in so101-core's git-ignored `config/devices.yaml`).
- ZED 2 on a fixed mount (works as a plain UVC stereo camera, 2x 1280x720 at 30 fps). Its current view shows the
  room; it needs reframing before any image goes into this public repo.
- Wrist camera (configured as `wrist`).
- A rigid ChArUco board (printed, mounted flat): still to get.

## Decisions

See [DECISIONS.md](DECISIONS.md).
