"""so101-zero: drive the arm to the model's zero pose and check the elbow's zero offset with a ruler.

At zero the upper arm stands nearly upright and the forearm is level: the elbow axis (the centre of the elbow
servo's round horn) and the wrist_flex axis (the wrist servo's horn) are 135 mm apart at almost the same
height. Each degree of elbow error moves the wrist axis ~2.4 mm up or down, so a ruler tells the old zero
(7.3 deg off: the wrist axis ~17 mm lower) from the new one.

    so101-zero plan     preview only (no hardware): the route and the predicted heights
    so101-zero run      POWERED: rest -> zero, hold for measuring, then back to rest
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


def target(rest: np.ndarray) -> np.ndarray:
    q = np.zeros_like(rest)
    q[-1] = rest[-1]  # the gripper stays as it is
    return q


def plan(safety, rest: np.ndarray) -> tuple[np.ndarray, list[str], list[str]]:
    q0 = target(rest)
    path = st.route([q0], rest, safety, MOVE_SPEED / safety.cfg.rate_hz)
    problems = st.route_problems(path, st.Collisions(clearance_m=0.01), rest)
    h = axis_heights_mm(q0)
    wrong = q0.copy()
    wrong[2] -= np.radians(-7.28)  # if the old zero had been right, the elbow would really be 7.3 deg further
    hw = axis_heights_mm(wrong)
    lines = [
        f"Route: rest -> zero, {len(path)} commands at {np.degrees(MOVE_SPEED):.0f} deg/s.",
        "Predicted at zero: elbow axis {e:.0f} mm, wrist_flex axis {w:.0f} mm above the table "
        "(wrist {d:+.0f} mm relative to the elbow).".format(
            e=h["elbow_flex"], w=h["wrist_flex"], d=h["wrist_flex"] - h["elbow_flex"]
        ),
        "If the old zero had been right instead: wrist {d:+.0f} mm relative to the elbow.".format(
            d=hw["wrist_flex"] - hw["elbow_flex"]
        ),
    ]
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
    path, lines, problems = plan(arm.safety, jm.rest)
    print("\n".join(lines))
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems[:5]))
    print(
        "\nThe arm will stretch forward ~40 cm at ~23 cm height. Clear that space (leader arm, cat); "
        "stay next to it, power in reach."
    )
    if not ask_yes("The arm will be POWERED (torque on) and MOVE to the zero pose."):
        return
    with arm.connect(torque=True):
        arm.move_to(target(jm.rest), speed_rad_s=MOVE_SPEED)
        import time

        time.sleep(1.0)
        q = np.mean([arm.read() for _ in range(20)], axis=0)
        h = axis_heights_mm(q)
        print(
            "\nAt zero. Measured joint angles (deg): "
            + ", ".join(f"{a:+.1f}" for a in np.degrees(q[:5]))
            + "\nFor these angles the model puts the elbow axis at {e:.0f} mm and the wrist_flex axis at "
            "{w:.0f} mm above the table: the wrist {d:+.0f} mm relative to the elbow.".format(
                e=h["elbow_flex"], w=h["wrist_flex"], d=h["wrist_flex"] - h["elbow_flex"]
            )
        )
        print(
            "Measure the height of the centre of the elbow servo's round horn and of the wrist servo's horn "
            "above the table (same side of the arm), and look at the forearm: level?"
        )
        input("\n>>> Done measuring? Press Enter: the arm goes back to rest. ")
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
        _, lines, problems = plan(SafetyLayer(limits=jm.model_limits(*urdf_limits())), jm.rest)
        print("\n".join(lines))
        print("Preflight: " + ("ok" if not problems else "\n  " + "\n  ".join(problems[:5])))


if __name__ == "__main__":
    main()
