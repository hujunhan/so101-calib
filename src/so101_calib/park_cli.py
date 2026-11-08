"""so101-park: why the shoulder shakes at rest, and a pose to wait in instead.

Folded at rest, the arm's centre of mass sits almost over the shoulder axis: gravity loads shoulder_lift with
~0.03 N m, so the gears sit in their backlash with nothing pressing them to one side, and at P = 32 the
servo can hunt there. Standing up (lift -90, elbow -30, wrist 10: upper arm back, forearm up, the tool
~17 cm out and ~41 cm up, off the table's workspace) loads the shoulder with ~0.6 N m, always the same way.

    so101-park plan     no hardware: the route rest -> park -> rest, and the loads
    so101-park test     POWERED: hold 6 s at rest, 6 s at the park pose, 6 s at rest again, reading the
                        joints as fast as the bus allows; prints each joint's peak-to-peak movement
"""

import argparse
import time

import numpy as np

from so101_calib import SERVO_P
from so101_calib import statics as st

PARK_DEG = {"shoulder_lift": -90.0, "elbow_flex": -30.0, "wrist_flex": 10.0}  # others stay as at rest
HOLD_S = 6.0
MOVE_SPEED = np.radians(12.0)


def park_pose(rest: np.ndarray) -> np.ndarray:
    from so101_core.model import JOINTS

    q = rest.copy()
    for j, v in PARK_DEG.items():
        q[JOINTS.index(j)] = np.radians(v)
    return q


def plan(safety, rest: np.ndarray) -> tuple[list[str], list[str]]:
    from so101_core.model import ARM_JOINTS

    q_rest = safety.clip(rest)[0]
    park = park_pose(q_rest)
    path = st.route([park, q_rest], q_rest, safety, MOVE_SPEED / safety.cfg.rate_hz)
    problems = st.route_problems(path, st.Collisions(clearance_m=0.01), rest)
    g = st.Gravity()
    lines = [
        f"Route rest -> park -> rest: {len(path)} commands at {np.degrees(MOVE_SPEED):.0f} deg/s.",
        "Gravity load (N m) " + ", ".join(ARM_JOINTS[1:4]) + ":",
        "  at rest (clipped to the limits): " + ", ".join(f"{t:+.2f}" for t in g.tau(q_rest[:5])[1:4]),
        "  at the park pose:              " + ", ".join(f"{t:+.2f}" for t in g.tau(park[:5])[1:4]),
    ]
    return lines, problems


def hold(arm, seconds: float) -> np.ndarray:
    out, t0 = [], time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        out.append(arm.read())
    return np.array(out)


def run() -> None:
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap
    from so101_core.model import ARM_JOINTS

    devices, jm = DeviceConfig.load(), JointMap.load()
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    lines, problems = plan(arm.safety, jm.rest)
    print("\n".join(lines))
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems[:5]))
    print("\nThe arm will stand up above its base (~45 cm high). Clear the space; power in reach.")
    if input("\n>>> The arm will be POWERED and MOVE. Type 'yes': ").strip().lower() != "yes":
        return
    q_rest = arm.safety.clip(jm.rest)[0]
    results = []
    with arm.connect(torque=True):
        for name, q in (("rest", q_rest), ("park", park_pose(q_rest)), ("rest again", q_rest)):
            arm.move_to(q, speed_rad_s=MOVE_SPEED)
            time.sleep(1.0)
            r = hold(arm, HOLD_S)
            ptp = np.degrees(r[:, :5].max(0) - r[:, :5].min(0))
            results.append((name, len(r), ptp))
            print(
                f"{name:10s}: {len(r)} reads in {HOLD_S:.0f} s; peak-to-peak (deg) "
                + ", ".join(f"{j} {v:.2f}" for j, v in zip(ARM_JOINTS, ptp, strict=True)),
                flush=True,
            )
        print("\nSupport the arm: torque goes off next.")
    print("Done.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["plan", "test"])
    a = ap.parse_args()
    if a.stage == "test":
        run()
        return
    from so101_core.config import JointMap
    from so101_core.model import urdf_limits
    from so101_core.safety import SafetyLayer

    jm = JointMap.load()
    lines, problems = plan(SafetyLayer(limits=jm.model_limits(*urdf_limits())), jm.rest)
    print("\n".join(lines))
    print("Preflight: " + ("ok" if not problems else "\n  " + "\n  ".join(problems[:5])))


if __name__ == "__main__":
    main()
