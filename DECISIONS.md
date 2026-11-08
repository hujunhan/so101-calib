# Decisions

Format: date, decided, why, options. Open questions inherited from the series plan (so101-core):

- ZED 2: the ZED SDK, or the raw UVC stereo stream with OpenCV.
- The ChArUco board: size, printing and mounting.

## 2026-11-06: session order

- **Decided:** keep the drafted order, S1 (servo identification) first, then S2 (joint zero offsets).
- **Why:** S1 needs no camera and no board, so it can start now. Its tests compare commanded against measured
  joint angles, so they do not depend on the kinematic model being right.
- **Options:** S1 first; S2 first (the 8.66 cm vs 10 cm error is the largest one known, and the check runs
  torque-off).

## 2026-11-06: S1 measures between commands (dense reads in so101-core)

- **Decided:** add `follow(dense=True)` to so101-core's `SafeArm`: it keeps the 30 Hz commands, the safety
  filter and the stall watch, and reads the arm repeatedly until the next command is due (so101-core
  b6f76e9). so101-kinematics was bumped to the same so101-core commit so the two pins agree.
- **Why:** a torque-off probe measured one position read at 1.0 ms (median, 6 servos), so about 30 reads fit
  in one 33 ms tick. M1 read once per command, so its ~100 ms delay was known only to one tick, and part of
  it was the measurement itself.
- **Options:** dense reads in so101-core (reusable by later repos); extra reads inside `follow`'s `on_tick`
  callback, in this repo only (works, but bends a logging hook into a sampler).

## 2026-11-06: test the servo gain P = 32 before S1b

- **Decided:** rerun S1a with position gain P = 32 (the STS3215 factory value) and compare with P = 16
  (LeRobot's setting) before the pose sweep (S1b). `SafeArm(position_p=...)` in so101-core c38261b; the
  default stays 16.
- **Why:** S1a showed a +-0.4 deg rest band on every joint and up to 3 deg of one-sided error against gravity.
  A position servo makes torque from error times gain, so both should shrink with a higher P. S1b and S1c
  depend on P, so measuring them at a gain we then change would be wasted.
- **Options:** A/B at P = 32 first; S1b at P = 16 as planned (matches LeRobot's default, which M3 will use).
- **Watch:** oscillation at rest (jitter column) and temperature after each joint.

## 2026-11-06: the rest of M2 runs the servos at P = 32

- **Decided:** position gain P = 32 (factory value) for all M2 measurements and demos from S1b on.
  `so101_calib.SERVO_P`; S1a's tool keeps `--p` so the P = 16 baseline can be rerun.
- **Why:** at P = 32 the error against gravity halves (elbow 3.0 -> 1.55 deg) and the rest band shrinks 2-4x;
  no oscillation at rest and no temperature rise in a 4 min run. The error / gravity-torque ratio is about
  the same on lift, elbow and wrist (3.1-3.6 deg/Nm at P = 16, 1.5-1.9 at P = 32), so a joint behaves like a
  spring whose stiffness is set by P.
- **Options:** P = 32; P = 64 (errors would roughly halve again, but shoulder_pan already overshoots 1.5 deg
  steps by ~0.35 deg at 32); P = 16 (LeRobot's default "to avoid shakiness", errors twice as large).
- **Caveats:** the shakiness reports are from teleoperation with noisy commands, which M2 has not tried. M3
  will choose its own P; M2's data will show what each choice costs.

## 2026-11-06: S2 references: an iPad dot grid, touched with a pointer fixed to the gripper

- **Decided:** S2 fits the joint zero offsets from touches: torque off, the user guides a pointer to dots of a
  grid shown on an iPad (exact pixel pitch), placed at several heights and tilts; the joint angles at each
  touch are recorded. The pointer is something thin and stiff fixed to the gripper; its offset is fitted
  with the joint offsets.
- **Why:** LeRobot's calibration takes each joint's zero from a hand-placed "middle" pose, while the URDF
  (so101_new_calib) assumes the true middle, so offsets of a few degrees are expected (M1: 10 cm read as
  8.66 cm). Known grid positions give three coordinates per touch, more than a plane constraint; touches on
  one plane alone do not separate the parameters (Ikits and Hollerbach; Zhuang et al.), so heights, tilts and
  pointer angles vary. The iPad needs no printer, its pitch is exact, and S3 can reuse it.
- **Options:** iPad grid; printed paper grid (needs a printer; printers scale by ~1%); ruler marks on the table
  (0.5-1 mm marking error). Pointer: a pointer in the gripper (sharp, offset fitted); the fixed jaw tip (from
  the mesh, but a few mm wide).
- **Limit:** a shoulder_pan offset turns the whole arm about the vertical axis, exactly like a grid turned on
  the table, so it cannot be told apart here; it is fixed with the camera frames in S4.

## 2026-11-06: S2 touches by teleoperation, not by hand on the limp arm

- **Decided:** record the touches with the follower under servo control (P = 32), driven from the leader arm;
  nobody touches the follower. `so101-touch record` does this by default; `--limp` keeps the first method.
  Fit set cut to 12 touches (three dots, pointer steep and shallow, roll -60 and +60 deg; as precise as 27
  in simulation).
- **Why:** two hand-guided runs (27 and 12 touches) left 3.5-3.8 mm RMS after the fit, against ~1 mm
  expected, and 2-2.7 mm of it vertical although the tip was on the glass: at each touch the arm's shape
  differed from what the encoders report. No geometric extension (link lengths, axis tilts, joint scales)
  reduced it (3.8 -> 3.7 mm), so it is random per touch. The user had to push the last millimetres, and a
  limp arm must be held; forces then bend printed parts and open play the encoders do not see. Under servo
  control the arm carries only its weight, as in use, and the S1 servo model flags a pointer that pushes
  (command - measured beyond spring plus band).
- **Options:** teleoperation; a pin test first (tip in a hole, push, measure; tells the size of the effect
  but not a better method); calibrate later with the camera (S4, the arm moving itself); skip (offsets of
  ~2-5 deg shift the tip ~40 mm in the base frame, mostly a smooth shift that a camera calibration against
  this arm would partly absorb).
- **Context:** no published end-effector accuracy for the SO-101 was found. One project found a 10.5 deg
  shoulder_lift zero error through a wrist-camera hand-eye calibration and fitted joint zeros with the board
  on the table plane (github.com/enesbirlik/so101-hybrid-manipulation, PR 2).

## 2026-11-06: stop S2 (kinematic calibration); no offsets applied

- **Decided:** stop the touch calibration. Nothing is written to the joint map; the arm keeps LeRobot's
  calibration. M2 carries the uncalibrated accuracy (about 8 mm RMS, up to ~15 mm, in the touched region) into
  the error budget.
- **Why:** the user's call after four runs: a toothpick pointer slides in its tape (the fitted tip moved
  8.7 mm between runs), hand-guided touches left 3.5-3.8 mm per touch, and two teleoperated runs stopped after
  1-2 touches for the same pointer problem. On a cheap printed arm the result would be offsets known to a
  degree or two; camera-guided work (M3) relies on repeatability and on a hand-eye calibration against this
  arm, which absorbs part of the error.
- **Options:** stop; continue with a rigid pointer (a printed tip screwed to the gripper, or the fixed jaw
  tip) and teleoperation; fit the offsets later from the camera in S4 with the arm moving itself.

## 2026-11-06: S3 reads the ZED 2 as a UVC camera; factory calibration from Stereolabs

- **Decided:** use the ZED 2 as a plain UVC camera (2560x720 side by side, left half = left eye), like
  LeRobot's OpenCV cameras that M3 will record with. No ZED SDK. The factory calibration
  (`SN<serial>.conf`, downloaded once from calib.stereolabs.com with the user's consent) is kept in the
  git-ignored `captures/zed/` and compared with our own ChArUco calibration; the serial is never committed.
- **Why:** M3 records through LeRobot's OpenCV cameras, so that is the path that has to be right. The SDK
  (neural depth, tracking) is a closed multi-GB CUDA dependency; Ubuntu 24.04 + CUDA 13 builds exist, but one
  report has the tools and Python API failing with a ZED 2i on that setup. S5's click-to-point needs no depth
  map for objects on the table (a ray from the pixel to the table plane).
- **Options:** UVC only; UVC plus trying the SDK for depth; the SDK as the main path.

## 2026-11-07: S4 hand-eye from one session: wrist camera, ZED and joint offsets

- **Decided:** the iPad board lies still on the table. The user poses the limp arm once so the wrist camera sees
  it; then the arm (torque on, P = 32, SafeArm) moves itself through ~20 poses around it. One nonlinear fit of
  the reprojection error gives the wrist camera in the gripper (X) and the board in the base (Y), with and
  without joint zero offsets (lift, elbow, wrist_flex); offsets are kept only if they predict held-out views
  better. The ZED sees the same still board before and after: base -> ZED = Y composed with the ZED's view of
  the board, so no marker on the gripper is needed.
- **Why:** a nonlinear reprojection fit of X and Y together beats closed-form hand-eye solutions (Ali et al.,
  Sensors 2019); the closed form only starts it. A camera measures without touching, so the joint offsets that
  S2 could not get from hand-guided touches may come out here at no extra cost. The user has no printer for a
  gripper marker, and the iPad is too big to carry.
- **Options:** poses automatic around a taught pose, or by teleoperation; offsets fitted or not; the ZED by the
  shared board, or by a marker on the gripper.

## 2026-11-07: S4 keeps one joint offset (elbow), fitted over two runs

- **Decided:** fit all S4 runs together (one camera mount and one set of offsets, a board placement per run) and
  keep the simplest offset model whose error on a left-out run is within 5 % of the best: elbow_flex only,
  -7.28 deg. Written into so101-core's joint map the same day (user's decision), with SafeArm limits that keep the
  LeRobot reading in range too.
- **Why:** per-run fits of lift, elbow and wrist offsets swung by up to 3 deg between runs while predicting each
  other equally well: lift and wrist trade against the camera mount and each other. The elbow alone predicts a
  run it was not fitted on as well as all three (4.7 px against 19.4 px with none), is known to 0.1 deg, and
  S2's touch data and the table height agree with it.
- **Options:** all three offsets (first run's choice); elbow only; write the offset into so101-core now.
