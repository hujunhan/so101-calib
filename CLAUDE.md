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
- **Next:** S2: kinematic calibration (joint zero offsets; M1's 10 cm ruler read 8.66 cm by FK). Open for S3: ZED SDK vs UVC; the
  ChArUco board (the user has no print; an iPad is the likely board for intrinsics and eye-in-hand, not for
  eye-to-hand, which needs a light marker on the gripper).

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
- ZED 2 is `/dev/video6` as a UVC device: 2560x720 side by side, take the left half.
- Rendering Mermaid diagrams: `npx -y @mermaid-js/mermaid-cli@11.4.2 -p <puppeteer.json>` with
  `{"args":["--no-sandbox"]}`; Chromium's sandbox cannot start on this Ubuntu.
