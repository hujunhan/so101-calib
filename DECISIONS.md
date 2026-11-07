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
