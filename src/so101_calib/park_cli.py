"""so101-park: why the shoulder shakes at rest, and a pose to wait in instead.

Folded at rest, the arm's centre of mass sits almost over the shoulder axis: gravity loads shoulder_lift with
~0.03 N m, so the gears sit in their backlash with nothing pressing them to one side, and at P = 32 the
servo can hunt there. Standing up (lift -90, elbow -30, wrist 10: upper arm back, forearm up, the tool
~17 cm out and ~41 cm up, off the table's workspace) loads the shoulder with ~0.6 N m, always the same way.

    so101-park plan     no hardware: the route rest -> park -> rest, and the loads
    so101-park test     POWERED: hold 6 s at rest, 6 s at the park pose, 6 s at rest again, reading the
                        joints as fast as the bus allows; prints each joint's peak-to-peak movement
    so101-park approach POWERED: moves between a working pose (one of S5's views), rest and the park pose,
                        reading the joints densely; where along each move does a joint shake, and how much

Held still, no joint moved by one encoder step (0.09 deg) at rest or parked (2026-11-07): the shaking the user
sees comes on the way to rest, which `approach` records.
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


WORK_DEG = [13.5, -62.7, 51.9, 76.8, -129.0]  # an S5 wrist view of the iPad (p3), arm joints
SHAKE_WINDOW_S = 0.25


def shake(
    t: np.ndarray, q: np.ndarray, t_sent: np.ndarray, q_cmd: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Per arm joint: the largest peak-to-peak (deg) of the tracking error after removing its slow part (a
    60 ms running mean), over 0.25 s windows; and the time (s) of that window's centre."""
    from so101_calib import compensation as cp

    e = np.degrees(q[:, :5] - cp.desired_at(t, t_sent, q_cmd)[:, :5])
    dt = np.median(np.diff(t))
    k = max(int(0.06 / dt), 1)
    slow = np.column_stack([np.convolve(e[:, j], np.ones(k) / k, mode="same") for j in range(5)])
    fast = e - slow
    w = max(int(SHAKE_WINDOW_S / dt), 2)
    best, when = np.zeros(5), np.zeros(5)
    for i in range(k, len(t) - w - k, w // 2):
        ptp = fast[i : i + w].max(0) - fast[i : i + w].min(0)
        better = ptp > best
        best[better], when[better] = ptp[better], t[i + w // 2]
    return best, when


def approach() -> None:
    from datetime import datetime
    from pathlib import Path

    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap
    from so101_core.model import ARM_JOINTS

    from so101_calib.servo_id import ramp_path

    devices, jm = DeviceConfig.load(), JointMap.load()
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    rest = arm.safety.clip(jm.rest)[0]
    work = rest.copy()
    work[:5] = np.radians(WORK_DEG)
    park = park_pose(rest)
    moves = [
        ("rest -> work", rest, work),
        ("work -> rest", work, rest),
        ("rest -> work", rest, work),
        ("work -> park", work, park),
        ("park -> rest", park, rest),
    ]
    step = MOVE_SPEED / arm.safety.cfg.rate_hz
    path = np.vstack([ramp_path(a, b, step) for _, a, b in moves])
    problems = st.route_problems(path, st.Collisions(clearance_m=0.01), jm.rest)
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems[:5]))
    print(
        "Moves: " + ", ".join(m[0] for m in moves) + f", at {np.degrees(MOVE_SPEED):.0f} deg/s, read densely."
    )
    print(
        "The arm reaches forward over the table and stands up above its base. Clear the space; "
        "power in reach."
    )
    if input("\n>>> The arm will be POWERED and MOVE. Type 'yes': ").strip().lower() != "yes":
        return
    g = st.Gravity()
    out = Path("captures/park") / f"approach_{datetime.now():%Y%m%d_%H%M%S}.npz"
    out.parent.mkdir(parents=True, exist_ok=True)
    data = {}
    with arm.connect(torque=True):
        arm.move_to(rest, speed_rad_s=MOVE_SPEED)
        for i, (name, a, b) in enumerate(moves):
            res = arm.follow(ramp_path(a, b, step), dense=True)
            t_end = res.t_sent[-1]
            # and 2 s held at the end
            hold_t, hold_q, t0 = [], [], time.perf_counter()
            while time.perf_counter() - t0 < 2.0:
                hold_q.append(arm.read())
                hold_t.append(t_end + time.perf_counter() - t0)
            t = np.concatenate([res.t_dense, hold_t])
            q = np.vstack([res.q_dense, hold_q])
            data |= {f"{i}/t": t, f"{i}/q": q, f"{i}/t_sent": res.t_sent, f"{i}/q_cmd": res.q_cmd}
            best, when = shake(t, q, res.t_sent, res.q_cmd)
            j = int(np.argmax(best))
            qa = q[np.searchsorted(t, when[j]).clip(0, len(q) - 1)]
            print(
                f"{name:13s} ({t_end:.1f} s): shake peak-to-peak (deg) "
                + ", ".join(f"{n} {v:.2f}" for n, v in zip(ARM_JOINTS, best, strict=True))
                + f"\n               worst: {ARM_JOINTS[j]} at {when[j] - t_end:+.1f} s from the end, lift "
                f"{np.degrees(qa[1]):.0f} deg, gravity on lift {g.tau(qa[:5])[1]:+.2f} N m",
                flush=True,
            )
        np.savez_compressed(out, names=np.array([m[0] for m in moves]), **data)
        print(f"Saved {out}. Support the arm: torque goes off next.")
    print("Done.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["plan", "test", "approach"])
    a = ap.parse_args()
    if a.stage == "test":
        run()
        return
    if a.stage == "approach":
        approach()
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
