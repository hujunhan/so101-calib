"""so101-zero: drive the arm to the model's zero pose and check the elbow's zero offset with a ruler.

At zero the upper arm stands nearly upright and the forearm is level: the elbow axis (the centre of the elbow
servo's round horn) and the wrist_flex axis (the wrist servo's horn) are 135 mm apart at almost the same
height. Each degree of elbow error moves the wrist axis ~2.4 mm up or down, so a ruler tells the old zero
(7.3 deg off: the wrist axis ~17 mm lower) from the new one.

The run stops at two poses: the model's zero with the current joint map, then LeRobot's zero (every reading 0,
what the model took for zero before the offset), so both can be seen and measured.

    so101-zero plan     preview only (no hardware): the route and the predicted heights
    so101-zero run      POWERED: rest -> model zero -> LeRobot zero, holding at each, then back to rest
"""

import argparse

import numpy as np

from so101_calib import SERVO_P
from so101_calib import statics as st

MOVE_SPEED = np.radians(12.0)
HEIGHT_JOINTS = ("elbow_flex", "wrist_flex")


def axis_heights_mm(q: np.ndarray) -> dict[str, float]:
    """Heights (mm) above the table of the elbow and wrist_flex axes at model angles q (6,), with the table at
    the base's bottom face."""
    import mujoco
    from so101_core.model import JOINTS, MJCF_PATH

    from so101_calib.handeye_cli import base_bottom_z

    m = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    d = mujoco.MjData(m)
    for i, j in enumerate(JOINTS):
        d.qpos[m.joint(j).qposadr[0]] = q[i]
    mujoco.mj_forward(m, d)
    floor = base_bottom_z()
    return {j: float((d.xanchor[m.joint(j).id][2] - floor) * 1000) for j in HEIGHT_JOINTS}


def targets(rest: np.ndarray, offset: np.ndarray) -> list[tuple[str, np.ndarray]]:
    """The model's zero, and LeRobot's (every reading 0: q = the joint map's offsets); the gripper stays."""
    model = np.zeros_like(rest)
    lerobot = np.asarray(offset, float).copy()
    model[-1] = lerobot[-1] = rest[-1]
    return [
        ("model zero (joint map with the offset)", model),
        ("LeRobot zero (as before the offset)", lerobot),
    ]


def describe(q: np.ndarray) -> str:
    h = axis_heights_mm(q)
    return (
        "elbow axis {e:.0f} mm, wrist_flex axis {w:.0f} mm above the table: the wrist {d:+.0f} mm".format(
            e=h["elbow_flex"], w=h["wrist_flex"], d=h["wrist_flex"] - h["elbow_flex"]
        )
        + " relative to the elbow"
    )


def plan(safety, rest: np.ndarray, offset: np.ndarray) -> tuple[np.ndarray, list[str], list[str]]:
    poses = targets(rest, offset)
    path = st.route([q for _, q in poses], rest, safety, MOVE_SPEED / safety.cfg.rate_hz)
    problems = st.route_problems(path, st.Collisions(clearance_m=0.01), rest)
    lines = [f"Route: rest -> two poses, {len(path)} commands at {np.degrees(MOVE_SPEED):.0f} deg/s."]
    lines += [f"Predicted at the {name}: {describe(q)}." for name, q in poses]
    return path, lines, problems


def ask_yes(msg: str) -> bool:
    return input(f"\n>>> {msg} Type 'yes': ").strip().lower() == "yes"


def run() -> None:
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap

    devices, jm = DeviceConfig.load(), JointMap.load()
    if jm.rest is None:
        raise SystemExit("No rest pose in the joint map.")
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    path, lines, problems = plan(arm.safety, jm.rest, jm.offset)
    print("\n".join(lines))
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems[:5]))
    print(
        "\nThe arm will stretch forward ~40 cm at ~23 cm height. Clear that space (leader arm, cat); "
        "stay next to it, power in reach."
    )
    if not ask_yes("The arm will be POWERED (torque on) and MOVE to the two zero poses."):
        return
    import time

    with arm.connect(torque=True):
        poses = targets(jm.rest, jm.offset)
        for k, (name, q_cmd) in enumerate(poses, 1):
            arm.move_to(q_cmd, speed_rad_s=MOVE_SPEED)
            time.sleep(1.0)
            q = np.mean([arm.read() for _ in range(20)], axis=0)
            print(
                f"\n[{k}/{len(poses)}] At the {name}. Measured joint angles (deg): "
                + ", ".join(f"{a:+.1f}" for a in np.degrees(q[:5]))
                + f"\nFor these angles the model puts the {describe(q)}."
            )
            print(
                "Measure the height of the centre of the elbow servo's round horn and of the wrist servo's "
                "horn above the table (same side of the arm), and look at the forearm: level?"
            )
            nxt = "the next pose" if k < len(poses) else "back to rest"
            input(f"\n>>> Done measuring? Press Enter: the arm goes to {nxt}. ")
        arm.move_to(jm.rest, speed_rad_s=MOVE_SPEED)
    print("Done.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["plan", "run"])
    a = ap.parse_args()
    if a.stage == "run":
        run()
    else:
        from so101_core.config import JointMap
        from so101_core.model import urdf_limits
        from so101_core.safety import SafetyLayer

        jm = JointMap.load()
        _, lines, problems = plan(SafetyLayer(limits=jm.model_limits(*urdf_limits())), jm.rest, jm.offset)
        print("\n".join(lines))
        print("Preflight: " + ("ok" if not problems else "\n  " + "\n  ".join(problems[:5])))


if __name__ == "__main__":
    main()
