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
