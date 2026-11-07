# Working notes for Claude

M2 of the SO-101 series. Plan, status and results: README.md. Decisions: DECISIONS.md.
The series plan (M0-M5) is in the so101-core README. M1 (so101-kinematics) is done: its README ("Handed to M2")
and docs/DESIGN.md say what this repo starts from.

## Current state
- 2026-11-06: S1a done (README), at P = 16 and P = 32; P = 32 decided for the rest of M2. S1b done: a joint is a
  spring with a load-dependent rest band, error = c * tau + side * (b + k |tau|) + o, tau from the URDF; fitted
  without payload, it predicts a 100 g payload run to 0.08-0.21 deg RMS. Use the fit on both runs
  (c = 1.62 / 1.79 / 1.81 deg/N m for lift / elbow / wrist).
- S1c done: feed-forward (spring + band + 80 ms lead) cuts tracking error on M1's circle to 0.6-0.7 mm median
  (from 2.0 / 3.8 mm at 2 / 5 cm/s); lead could be 65-70 ms. `so101-track plan` has a virtual-servo preview.
- S2 tried and stopped (README, DECISIONS): uncalibrated about 8 mm RMS (up to ~15 mm) in the touched region;
  no offsets applied. `so101-touch` stays for a later try (rigid pointer, teleop).
- S3 done (README): wrist intrinsics in results/s3/wrist_intrinsics.yaml (left corners weak); ZED over UVC
  with the factory calibration (`use: factory` in results/s3/zed_intrinsics.yaml; file in captures/zed/).
- **Next:** S4: `so101-handeye` written (zed-view to place the ZED; run: teach a limp pose, then ~24 automatic poses
  over the still iPad, collision-checked with it; fit: hand-eye with/without joint offsets, held-out comparison,
  ZED via the shared board). The user runs `zed-view` then `run`; Claude runs `fit`. 

- For M3 (noted 2026-11-07): the ZED is wide (~100 deg H as calibrated, 110 nominal); the workspace fills a small
  part of the image and the room (people, a cat) the rest. Plan: FHD per eye (factory parameters exist) and a fixed
  crop around the workspace (cx, cy shift; the hand-eye pose is unaffected). Decide the crop with M3's tasks.

## How we work
- Claude writes and runs most of the code. The user does physical work on the arm, makes the key
  decisions and learns the concepts. Present options with a recommendation and let the user decide
  design choices, then record them in DECISIONS.md (date, decided, why, options).
- The user must be able to defend every choice, so explain the why in plain terms, briefly.
- Talk to the user in Chinese. Code, comments, commits and repo docs are in English.
- Small steps; one session ≈ 1–2 hours of the user's time.
- Check claims against a baseline before writing them down; report results that go against the plan as they are.

## Rules
- The repo is public. Never commit personal information or device serial numbers. Camera images can show
  people and the room: keep raw captures in the git-ignored `captures/`, commit only reviewed, cropped images.
- Real-arm motion only through so101-core's `SafeArm` (limit margin, step cap, stall detection; `move_to`,
  `follow`), never raw bus writes. Tell the user before anything moves the arm; they must be present with power
  in reach.
- Dependencies are pinned git commits (so101-core, so101-kinematics). If a fix belongs there, make it there, push,
  then bump the pin here.
- Tooling: uv, Python 3.12, ruff, pytest. Run `uv run ruff check . && uv run pytest` before committing.
- Commit and push when a step is done and verified. After a session milestone, update the status line in
  README.md and the project row in the user's twin repo (`~/code/digital-twin/projects/README.md`).

## Practical notes from M1
- Device config: `SO101_CONFIG_DIR=~/code/mine/so101-core/config` (ports, rest pose, joint map).
- Powered runs: the user runs the script in their own terminal (it asks for `yes`, and for Enter before
  releasing torque; Claude's shell cannot answer prompts) and pastes the output; Claude analyzes the logs.
  Torque-off reads (`SafeArm(...).connect(torque=False)`) Claude can run directly; they make the arm limp.
- If the follower reports "Missing motor IDs", the servo power supply is off.
- Always `uv sync --extra hw`: a plain `uv sync` removes LeRobot and with it opencv-python-headless, which
  deletes files that opencv-python shares (`cv2.aruco` disappears). Fix: `uv sync --extra hw --reinstall-package opencv-python`.
  The same reinstall fixes "The function is not implemented" from `cv2.imshow`/`waitKey`: the headless build's
  files had overwritten opencv-python's (no GUI). opencv-python is pinned below 5 to stay on LeRobot's major.
- ZED 2 is `/dev/video6` as a UVC device: 2560x720 side by side, take the left half.
- Rendering Mermaid diagrams: `npx -y @mermaid-js/mermaid-cli@11.4.2 -p <puppeteer.json>` with
  `{"args":["--no-sandbox"]}`; Chromium's sandbox cannot start on this Ubuntu.
