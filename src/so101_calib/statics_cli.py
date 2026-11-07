"""so101-statics: S1b, the servos' static error over the workspace. Motion only through so101-core's SafeArm.

    so101-statics plan                      no hardware: the poses, the route, the collision and height checks
    so101-statics run [--payload-g G --payload-box L W T]
                                            torque ON (P = SERVO_P): at each pose, approach from 3 deg above
                                            and below on every arm joint, settle, read;
                                            -> results/s1/statics_<time>.npz
    so101-statics preview [--viewer]        no hardware: render the poses (results/s1/statics_poses.png); with
                                            --viewer, play the whole route in a MuJoCo window at real speed
    so101-statics analyze FILE.npz          fit error = c * tau + b * side + o per joint -> FILE.md, FILE.png

`--payload-g` only labels the run and adds that mass to the torque model: the payload itself must already
be fixed to the gripper. With `--payload-box L W T` (cm) the payload is a box taped flat against the outside
of the fixed jaw, long side along the gripper, end flush with the jaw tip: its centre is the centre of mass in
the model and the box joins the collision check. Without it the mass sits at the tool point.
`run` asks for `yes` before the arm is powered and again before it moves.
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from so101_core.arm import SafeArm
from so101_core.config import DeviceConfig, JointMap
from so101_core.model import ARM_JOINTS, JOINTS
from so101_core.safety import SafetyLayer

from so101_calib import SERVO_P
from so101_calib import statics as st

RESULTS = Path("results/s1")
MOVE_SPEED = np.radians(20.0)


def ask_yes(message: str) -> bool:
    return input(f"\n>>> {message} Type 'yes' to continue: ").strip().lower() == "yes"


def outbound(rest: np.ndarray, first: np.ndarray, safety: SafetyLayer) -> np.ndarray:
    """The commands `move_to(first)` sends from the rest pose; reversed, the way back."""
    return st.ramp_path(safety.clip(rest)[0], first, MOVE_SPEED / safety.cfg.rate_hz)


def plan(
    safety: SafetyLayer, rest: np.ndarray, grav: st.Gravity, box_m: tuple[float, float, float] | None = None
) -> tuple[list[np.ndarray], list[str], list[str]]:
    """Poses (6,), a description, and problems (empty if the plan is safe to run)."""
    cands = st.candidate_poses(safety, st.Gravity())  # the same poses with and without a payload
    chosen = st.spread_poses(cands, np.radians([0, -34, 34, 34, 0]), st.N_POSES)
    grip = safety.clip(rest)[0][5]
    poses = [np.append(c.q, grip) for c in chosen]
    step = MOVE_SPEED / safety.cfg.rate_hz
    # Out: rest -> first pose (move_to). Back: the same path reversed (follow), so the arm folds the way it
    # unfolded; a plain move_to back finishes the short joints first and passes poses the model cannot clear
    # (it overlaps near rest). A payload goes on and comes off at the first pose: it would hit the shoulder
    # when the arm folds.
    out = outbound(rest, poses[0], safety)
    path = np.vstack([out, st.route(poses[1:] + [poses[0]], poses[0], safety, step)])
    coll = st.Collisions(payload_box_m=box_m)
    problems = st.route_problems(out, st.Collisions(), rest)
    problems += st.route_problems(path[len(out) :], coll, rest)
    approaches = [a for p in poses for _, a in st.approach_targets(p, safety)]
    for a in approaches:
        if coll.hits(a):
            problems.append(f"approach start in contact: {np.round(np.degrees(a[:5])).tolist()}")
    travel_s = (len(path) + len(out)) / safety.cfg.rate_hz
    per_approach = 2 * np.radians(st.APPROACH_DEG) / st.APPROACH_SPEED + 0.5 + st.HOLD_S
    total = travel_s + len(approaches) * per_approach
    tools = np.array([grav.arm.fk(p[:5])[:3, 3] for p in poses]) * 100
    lines = [
        f"{len(poses)} poses (pan = roll = 0) chosen from {len(cands)} candidates "
        "to spread the gravity torques; "
        f"position gain P = {SERVO_P}; payload {grav.payload_kg * 1000:.0f} g"
        + ("" if box_m is None else f", box {np.round(np.array(box_m) * 100, 1).tolist()} cm")
        + f" at {np.round(grav.payload_pos * 100, 1).tolist()} cm (gripper frame)",
        f"Tool positions: x {tools[:, 0].min():.0f} to {tools[:, 0].max():.0f} cm forward, "
        f"z {tools[:, 2].min():.0f} to {tools[:, 2].max():.0f} cm up",
        f"At each pose: {2 * st.REPEATS} approaches from +-{st.APPROACH_DEG:.0f} deg on every arm joint at "
        f"{np.degrees(st.APPROACH_SPEED):.0f} deg/s, settle, read {st.HOLD_S:.0f} s",
        f"Moves between poses at {np.degrees(MOVE_SPEED):.0f} deg/s; about {total / 60:.1f} min in all",
        "Collision check (MuJoCo model, 5 mm clearance, table = base bottom face): "
        + ("ok" if not problems else f"{len(problems)} problems"),
    ]
    for i, (p, c) in enumerate(zip(poses, chosen, strict=True)):
        lines.append(
            f"  {i:2d}: {np.round(np.degrees(p[:5])).astype(int).tolist()} deg, tool "
            f"{np.round(tools[i]).astype(int).tolist()} cm, tau lift/elbow/wrist "
            f"{np.round(c.tau[1:4], 2).tolist()} N m"
        )
    return poses, lines, problems


def run(payload_g: float, box_cm: list[float] | None) -> None:
    devices = DeviceConfig.load()
    jm = JointMap.load()
    if jm.rest is None:
        raise SystemExit("No rest pose in the joint map; record one with so101-core first.")
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    box_m = None if box_cm is None else tuple(np.array(box_cm) / 100)
    grav = st.Gravity(payload_g / 1000, None if box_m is None else st.payload_center(box_m))
    poses, lines, problems = plan(arm.safety, jm.rest, grav, box_m)
    print("\n".join(lines))
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems[:10]))
    print(
        "Clear a half-circle of about 50 cm in front of the arm, up to 55 cm high. Stay next to it with the "
        "power switch in reach."
    )
    if not ask_yes("The arm will be POWERED (torque on)."):
        return
    RESULTS.mkdir(parents=True, exist_ok=True)
    tag = f"_{payload_g:.0f}g" if payload_g else ""
    out = RESULTS / f"statics_{datetime.now():%Y%m%d_%H%M%S}{tag}.npz"
    meta = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "position_p": SERVO_P,
        "payload_g": payload_g,
        "payload_box_cm": box_cm,
        "payload_pos_m": grav.payload_pos.tolist(),
        "approach_deg": st.APPROACH_DEG,
        "hold_s": st.HOLD_S,
    }
    rec: dict[str, list] = {k: [] for k in ("pose", "side", "q_cmd", "q_meas", "q_std", "load", "temp", "t")}
    t0 = time.perf_counter()
    with arm.connect(torque=True):
        bus = arm.robot.bus
        meta["regs"] = {
            reg: {k: int(v) for k, v in bus.sync_read(reg, normalize=False).items()}
            for reg in ("P_Coefficient", "Present_Voltage", "Present_Temperature")
        }
        print(f"Servo settings: {meta['regs']}")
        if not ask_yes("The arm will MOVE through the poses."):
            return
        if box_m is not None:
            arm.move_to(poses[0], speed_rad_s=MOVE_SPEED)
            print(
                "The arm holds the first pose. Tape the payload flat against the OUTSIDE of the fixed jaw, "
                "long "
                "side along the gripper, end flush with the jaw tip. Press gently; the servos hold."
            )
            if not ask_yes("Payload attached and tight?"):
                return
        for i, pose in enumerate(poses):
            print(
                f"pose {i + 1}/{len(poses)}: {np.round(np.degrees(pose[:5])).astype(int).tolist()} deg",
                flush=True,
            )
            arm.move_to(pose, speed_rad_s=MOVE_SPEED)
            errs = []
            for side, start in st.approach_targets(pose, arm.safety):
                arm.move_to(start, speed_rad_s=st.APPROACH_SPEED)
                arm.move_to(pose, speed_rad_s=st.APPROACH_SPEED)
                ts, qs = [], []
                t_hold = time.perf_counter()
                while time.perf_counter() - t_hold < st.HOLD_S:
                    qs.append(arm.read())
                    ts.append(time.perf_counter())
                qs = np.array(qs)
                last = np.array(ts) > ts[0] + st.HOLD_S / 2
                rec["pose"].append(i)
                rec["side"].append(side)
                rec["q_cmd"].append(arm.q_cmd.copy())
                rec["q_meas"].append(qs[last].mean(axis=0))
                rec["q_std"].append(qs[last].std(axis=0))
                rec["load"].append([int(v) for v in bus.sync_read("Present_Load", normalize=False).values()])
                rec["temp"].append(
                    [int(v) for v in bus.sync_read("Present_Temperature", normalize=False).values()]
                )
                rec["t"].append(time.perf_counter() - t0)
                errs.append(np.degrees(arm.q_cmd - rec["q_meas"][-1])[:5])
            e = np.array(errs)
            print(
                "  error (deg, command - measured), from above / below: "
                + ", ".join(
                    f"{ARM_JOINTS[k]} {e[0::2, k].mean():+.2f}/{e[1::2, k].mean():+.2f}" for k in range(5)
                )
                + f"; max temp {max(rec['temp'][-1])} C"
            )
            np.savez_compressed(out, meta=json.dumps(meta), **{k: np.array(v) for k, v in rec.items()})
        arm.move_to(poses[0], speed_rad_s=MOVE_SPEED)
        if box_m is not None:
            input("\n>>> The arm holds the first pose. Remove the payload, then press Enter. ")
        print(f"Saved {out}. Moving back to the rest pose the way it came.")
        arm.follow(outbound(jm.rest, poses[0], arm.safety)[::-1])
    print(f"\nDone. Analyze with: uv run so101-statics analyze {out}")


# -- preview ------------------------------------------------------------------------------------------------
def preview(viewer: bool) -> None:
    import os

    os.environ.setdefault("MUJOCO_GL", "egl")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import mujoco
    from so101_core.twin import Twin

    jm = JointMap.load()
    safety = SafetyLayer()
    rest = jm.rest if jm.rest is not None else np.zeros(len(JOINTS))
    grav = st.Gravity()
    poses, lines, _ = plan(safety, rest, grav)
    twin = Twin()
    renderer = mujoco.Renderer(twin.model, 360, 300)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.18, 0.0, 0.22]
    cam.distance, cam.azimuth, cam.elevation = 1.05, 90.0, -8.0  # from the arm's left side: forward = right
    tiles = [("rest", rest)] + [(f"{i}", p) for i, p in enumerate(poses)]
    cols = 6
    fig, axes = plt.subplots(-(-len(tiles) // cols), cols, figsize=(2.6 * cols, 3.0 * -(-len(tiles) // cols)))
    for ax in axes.flat:
        ax.axis("off")
    for ax, (label, q) in zip(axes.flat, tiles, strict=False):
        twin.set(q)
        renderer.update_scene(twin.data, cam)
        ax.imshow(renderer.render())
        tool = grav.arm.fk(q[:5])[:3, 3] * 100
        ax.set_title(
            f"{label}: {np.round(np.degrees(q[1:4])).astype(int).tolist()} deg\n"
            f"tool x {tool[0]:.0f}, z {tool[2]:.0f} cm",
            fontsize=8,
        )
    fig.suptitle(
        "S1b poses, seen from the arm's left (forward = right); joints shown: lift, elbow, wrist", fontsize=10
    )
    fig.tight_layout()
    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / "statics_poses.png"
    fig.savefig(out, dpi=100)
    print("\n".join(lines))
    print(f"Wrote {out}")
    if viewer:
        hz = safety.cfg.rate_hz
        out = outbound(rest, poses[0], safety)
        path = np.vstack(
            [out, st.route(poses[1:] + [poses[0]], poses[0], safety, MOVE_SPEED / hz), out[::-1]]
        )
        twin.open_viewer()
        rate_dt = 1 / hz
        for q in path:
            if not twin.show(q):
                break
            time.sleep(rate_dt)
        input("Route done. Press Enter to close the viewer. ")
        twin.close()


# -- analysis ------------------------------------------------------------------------------------------------
def analyze(path: Path) -> None:
    z = np.load(path)
    err, tau, side, meta = st.load_run(path)
    load = z["load"][:, :5].astype(float)
    rows, fits = [], {}
    for k, name in enumerate(ARM_JOINTS):
        f = st.fit_joint(err[:, k], side, tau[:, k] if k in st.GRAVITY_JOINTS else None)
        fits[name] = f
        r = np.corrcoef(tau[:, k], load[:, k])[0, 1] if k in st.GRAVITY_JOINTS else np.nan
        c = "-" if f["c"] is None else f"{f['c']:.2f}"
        kk = "-" if f["k"] is None else f"{f['k']:+.2f}"
        const = f.get("rms_resid_const_band_deg")
        rows.append(
            f"| {name} | {c} | {f['b']:+.2f} | {kk} | {f['o']:+.2f} | {f['rms_before_deg']:.2f} | "
            f"{'-' if const is None else f'{const:.2f}'} | {f['rms_resid_deg']:.2f} | "
            f"{f['max_resid_deg']:.2f} | {'-' if np.isnan(r) else f'{r:.2f}'} |"
        )
    noise = np.degrees(z["q_std"][:, :5]).max()
    md = [
        f"# S1b static error over the workspace: {path.name}",
        "",
        f"Date {meta['date']}; position gain P = {meta['position_p']}; payload {meta['payload_g']:.0f} g; "
        f"{len(set(z['pose'].tolist()))} poses, {len(side)} approaches "
        f"(from +-{meta['approach_deg']:.0f} deg on "
        f"every arm joint). Largest std of a joint while held: {noise:.3f} deg.",
        "",
        "Fit per joint: error (command - measured, deg) = c * tau + side * (b + k * |tau|) + o. "
        "tau = holding torque from the "
        "URDF masses (N m); side = +1 approached from above, -1 from below; b + k |tau| is the (signed) "
        "half-width of the rest band, which grows with the load. Pan and roll carry no gravity "
        "torque: error = b * side + o. r(load) = correlation of tau with the servo's Present_Load register.",
        "",
        "| Joint | c (deg/N m) | b (deg) | k (deg/N m) | o (deg) | RMS error (deg) | RMS after fit, k = 0 "
        "| RMS after fit | Max after fit | r(load) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        *rows,
        "",
    ]
    path.with_suffix(".md").write_text("\n".join(md))
    _plot(err, tau, side, fits, path.with_suffix(".png"))
    print("\n".join(md))
    print(f"Wrote {path.with_suffix('.md')} and {path.with_suffix('.png')}")


def _plot(err, tau, side, fits, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 5, figsize=(17, 3.4))
    for k, (ax, name) in enumerate(zip(axes, ARM_JOINTS, strict=True)):
        f = fits[name]
        x = tau[:, k] if f["c"] is not None else np.zeros_like(side)
        for s, color, label in ((1, "tab:red", "from above"), (-1, "tab:blue", "from below")):
            m = side == s
            ax.scatter(x[m], err[m, k], s=12, color=color, label=label)
            if f["c"] is not None:
                xs = np.linspace(x.min(), x.max(), 20)
                ax.plot(xs, st.predict(f, np.full_like(xs, s), xs), color=color, lw=0.8)
        ax.axhline(0, color="k", lw=0.5)
        ax.set(title=name, xlabel="gravity torque (N m)" if f["c"] is not None else "(no gravity load)")
        if k == 0:
            ax.set_ylabel("command - measured (deg)")
            ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("plan", "run"):
        sp = sub.add_parser(name)
        sp.add_argument("--payload-g", type=float, default=0.0)
        sp.add_argument("--payload-box", type=float, nargs=3, metavar=("L", "W", "T"), help="cm")
    sub.add_parser("preview").add_argument("--viewer", action="store_true")
    sub.add_parser("analyze").add_argument("file", type=Path)
    args = p.parse_args()
    if args.cmd == "plan":
        jm = JointMap.load()
        rest = jm.rest if jm.rest is not None else np.zeros(len(JOINTS))
        box_m = None if args.payload_box is None else tuple(np.array(args.payload_box) / 100)
        grav = st.Gravity(args.payload_g / 1000, None if box_m is None else st.payload_center(box_m))
        _, lines, problems = plan(SafetyLayer(), rest, grav, box_m)
        print("\n".join(lines))
        print("Preflight: " + ("ok" if not problems else "\n  " + "\n  ".join(problems[:10])))
    elif args.cmd == "preview":
        preview(args.viewer)
    elif args.cmd == "run":
        run(args.payload_g, args.payload_box)
    else:
        analyze(args.file)


if __name__ == "__main__":
    main()
