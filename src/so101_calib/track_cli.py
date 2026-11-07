"""so101-track: S1c, M1's circle with and without feed-forward compensation. Motion only through SafeArm.

    so101-track model [RUN.npz ...]   no hardware: fit the servo model on S1b runs (default: all of
                                      results/s1/statics_*.npz) -> results/s1/servo_model.json
    so101-track plan                  no hardware: the commands of every variant, safety preflight, collisions
    so101-track run                   torque ON (P = SERVO_P): draw the circle with each variant (none,
                                      spring, band, full) at each speed, interleaved, twice; read densely
                                      -> results/s1/track_<time>.npz
    so101-track analyze FILE.npz      tracking error per variant -> FILE.md, FILE.png

The circle is M1's (so101-kinematics drawing.py): radius 3.6 cm on the plane 20 cm in front of the base, the
tool horizontal. Errors are of the measured joints (FK) against the target: how well the servos follow, not
the absolute accuracy of the model (that is S2). `run` asks for `yes` before the arm is powered and before it
moves.
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
from so101_kinematics.drawing import preflight, shape_ticks

from so101_calib import SERVO_P
from so101_calib import compensation as cp
from so101_calib import statics as st
from so101_calib.statics_cli import MOVE_SPEED, ask_yes, outbound

RESULTS = Path("results/s1")
SPEEDS_CM_S = (2.0, 5.0)
REPEATS = 2
SETTLE_S = 0.5  # hold the first command this long before drawing
HOLD_S = 1.0  # read this long after the last command: the static error where the circle ends


def programs(safety: SafetyLayer, rest: np.ndarray, model: cp.ServoModel, lead_s: float):
    """[(variant, speed, repeat, target, commands)] in run order, and the uncompensated start pose."""
    from so101_kinematics.models import PinArm

    kin, grav, hz = PinArm(), st.Gravity(), safety.cfg.rate_hz
    grip = safety.clip(rest)[0][5]
    targets = {v: shape_ticks(kin, "circle", v / 100, hz, grip) for v in SPEEDS_CM_S}
    cmds = {
        (name, v): cp.compensate(targets[v], hz, model, grav, parts, lead_s)
        for v in SPEEDS_CM_S
        for name, parts in cp.VARIANTS.items()
    }
    out = [
        (name, v, r, targets[v], cmds[name, v])
        for r in range(REPEATS)
        for v in SPEEDS_CM_S
        for name in cp.VARIANTS
    ]
    return out, targets[SPEEDS_CM_S[0]][0]


def plan(safety: SafetyLayer, rest: np.ndarray, model: cp.ServoModel, lead_s: float):
    from so101_kinematics.models import PinArm

    progs, home = programs(safety, rest, model, lead_s)
    hz, fk, grav = safety.cfg.rate_hz, PinArm().fk, st.Gravity()
    problems, lines = [], []
    coll = st.Collisions()
    problems += st.route_problems(outbound(rest, home, safety), coll, rest)
    for name, v, r, target, cmd in progs:
        if r:
            continue
        clipped, capped = preflight(cmd, safety)
        if clipped or capped:
            problems.append(f"{name} at {v:g} cm/s: {clipped} ticks clipped, {capped} step-capped")
        problems += [f"{name} {v:g} cm/s: {p}" for p in st.route_problems(cmd, coll, rest)]
        add = np.degrees(cmd - target)[:, :5]
        step = np.degrees(np.abs(np.diff(cmd, axis=0))).max()
        lines.append(
            f"  {name:6s} {v:g} cm/s: {len(cmd)} ticks ({len(cmd) / hz:.1f} s); added to the target (deg) "
            + ", ".join(
                f"{j} {add[:, k].min():+.2f}..{add[:, k].max():+.2f}" for k, j in enumerate(ARM_JOINTS)
            )
            + f"; largest step {step:.2f} deg"
        )
        t_sent, t_dense, q_dense = cp.simulate(cmd, hz, model, grav)
        sim = cp.Run.make(fk, t_sent, target, t_dense, q_dense).tracking()
        lines.append(
            f"         virtual servo (S1 model, {cp.SIM_DELAY_S * 1000:.0f} ms delay): tool error median "
            f"{sim['tool_median_mm']:.1f} mm, max {sim['tool_max_mm']:.1f} mm"
        )
    moves = len(progs) * (SETTLE_S + HOLD_S + 2.0)
    total = sum(len(p[4]) for p in progs) / hz + moves + 2 * len(outbound(rest, home, safety)) / hz
    head = [
        f"M1 circle, P = {SERVO_P}, servo model from {model.runs}; lead {lead_s * 1000:.0f} ms",
        f"Variants {list(cp.VARIANTS)} x speeds {list(SPEEDS_CM_S)} cm/s, interleaved, {REPEATS} times: "
        f"{len(progs)} circles, about {total / 60:.1f} min",
        "Safety preflight and collision check (MuJoCo, 5 mm): " + ("ok" if not problems else "PROBLEMS"),
    ]
    return progs, home, head + lines, problems


def run(lead_s: float) -> None:
    devices, jm = DeviceConfig.load(), JointMap.load()
    if jm.rest is None:
        raise SystemExit("No rest pose in the joint map; record one with so101-core first.")
    model = cp.ServoModel.load()
    if model.position_p != SERVO_P:
        raise SystemExit(f"servo model fitted at P = {model.position_p}, running at {SERVO_P}")
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    progs, home, lines, problems = plan(arm.safety, jm.rest, model, lead_s)
    print("\n".join(lines))
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems[:10]))
    print(
        "Clear the space in front of the arm (about 35 cm out, 20-40 cm up). Stay next to it, power in reach."
    )
    if not ask_yes("The arm will be POWERED (torque on)."):
        return
    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / f"track_{datetime.now():%Y%m%d_%H%M%S}.npz"
    meta = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "position_p": SERVO_P,
        "lead_s": lead_s,
        "model": cp.ServoModel.load().__dict__,
        "runs": [],
    }
    data: dict[str, np.ndarray] = {}
    with arm.connect(torque=True):
        bus = arm.robot.bus
        if not ask_yes("The arm will MOVE to the circle and draw it 16 times (no pen, in the air)."):
            return
        arm.move_to(home, speed_rad_s=MOVE_SPEED)
        for i, (name, v, r, target, cmd) in enumerate(progs):
            arm.move_to(cmd[0], speed_rad_s=np.radians(10.0))
            time.sleep(SETTLE_S)
            t0 = time.perf_counter()
            status = "complete"
            try:
                res = arm.follow(cmd, dense=True)
            except BaseException as e:
                status = f"stopped: {type(e).__name__}: {e}"
                meta["runs"].append({"variant": name, "speed_cm_s": v, "repeat": r, "status": status})
                np.savez_compressed(out, meta=json.dumps(meta), **data)
                raise
            hold = []
            th = time.perf_counter()
            while time.perf_counter() - th < HOLD_S:
                hold.append((time.perf_counter() - t0, arm.read()))
            temp = [int(x) for x in bus.sync_read("Present_Temperature", normalize=False).values()]
            meta["runs"].append(
                {"variant": name, "speed_cm_s": v, "repeat": r, "status": status, "temp": temp}
            )
            data |= {
                f"{i}/target": target,
                f"{i}/cmd": res.q_cmd,
                f"{i}/t_sent": res.t_sent,
                f"{i}/t_dense": np.concatenate([res.t_dense, [h[0] for h in hold]]),
                f"{i}/q_dense": np.vstack([res.q_dense, [h[1] for h in hold]]),
                f"{i}/t_hold_start": np.array(th - t0),
            }
            end = np.degrees(target[-1] - np.mean([h[1] for h in hold[len(hold) // 2 :]], axis=0))[:5]
            print(
                f"{i + 1:2d}/{len(progs)} {name:6s} {v:g} cm/s: {len(res.t_dense)} reads; "
                f"{res.clipped} clipped, {res.step_limited} capped; error at the end (deg) "
                f"{np.round(end, 2).tolist()}; max temp {max(temp)} C",
                flush=True,
            )
            np.savez_compressed(out, meta=json.dumps(meta), **data)
        arm.move_to(home, speed_rad_s=MOVE_SPEED)
        print(f"Saved {out}. Moving back to the rest pose the way it came.")
        arm.follow(outbound(jm.rest, home, arm.safety)[::-1])
    print(f"\nDone. Analyze with: uv run so101-track analyze {out}")


# -- analysis ------------------------------------------------------------------------------------------------
def load_runs(path: Path):
    from so101_kinematics.models import PinArm

    z = np.load(path)
    meta = json.loads(str(z["meta"]))
    fk = PinArm().fk
    out = []
    for i, r in enumerate(meta["runs"]):
        if f"{i}/t_sent" not in z:
            continue
        t_sent = z[f"{i}/t_sent"]
        ths = float(z[f"{i}/t_hold_start"])
        td, qd = z[f"{i}/t_dense"], z[f"{i}/q_dense"]
        run = cp.Run.make(fk, t_sent, z[f"{i}/target"], td, qd)
        hold = qd[td >= ths + HOLD_S / 2]
        r = r | {"end_err_deg": np.degrees(z[f"{i}/target"][-1] - hold.mean(0))[:5]}
        out.append((r, run))
    return meta, out


def analyze(path: Path) -> None:
    meta, runs = load_runs(path)
    rows, stats = [], {}
    for r, run in runs:
        m = run.tracking()
        m["delay_ms"] = run.residual_delay_s() * 1000
        m["end_err_deg"] = r["end_err_deg"]
        stats.setdefault((r["variant"], r["speed_cm_s"]), []).append((m, run))
    for (name, v), ms in stats.items():

        def avg(key, ms=ms):
            return np.mean([m[key] for m, _ in ms], axis=0)

        mean = avg("tool_mean_mm")
        rows.append(
            f"| {name} | {v:g} | {avg('tool_median_mm'):.1f} | {avg('tool_rms_mm'):.1f} | "
            f"{max(m['tool_max_mm'] for m, _ in ms):.1f} | "
            f"{np.linalg.norm(mean):.1f} ({mean[0]:+.1f}, {mean[1]:+.1f}, {mean[2]:+.1f}) | "
            f"{avg('delay_ms'):.0f} | "
            + " | ".join(f"{x:.2f}" for x in avg("joint_rms_deg"))
            + f" | {np.abs(avg('end_err_deg')).max():.2f} |"
        )
    md = [
        f"# S1c tracking with compensation: {path.name}",
        "",
        f"Date {meta['date']}; P = {meta['position_p']}; lead {meta['lead_s'] * 1000:.0f} ms; servo model "
        f"{meta['model']['runs']}. M1's circle (radius 3.6 cm, plane x = 20 cm), {len(runs)} circles; values "
        f"are means over the repeats (max: the largest). Errors from {cp.SKIP_S:g} s after the start to the "
        "last command, against the target at the time of each read (reads ~1 ms apart); tool positions by "
        "FK of the measured and the target joints, so this is how well the servos follow, not absolute "
        "accuracy.",
        "",
        "Variants: none = target (M1); spring = + c tau + o; band = + side (b + k |tau|); full = band "
        "on the target `lead` ahead. Residual delay: the extra lag that best lines the measured path up with "
        "the target (an uncompensated band counts in it too). End: largest joint error 0.5-1 s after the "
        "last command.",
        "",
        "| Variant | cm/s | Tool median (mm) | RMS | Max | Mean offset (x, y, z) | Residual delay (ms) | "
        + " | ".join(f"{j} RMS (deg)" for j in ARM_JOINTS)
        + " | End (deg) |",
        "|---|---:|---:|---:|---:|---|---:|" + "---:|" * len(ARM_JOINTS) + "---:|",
        *rows,
        "",
    ]
    path.with_suffix(".md").write_text("\n".join(md))
    _plot(stats, path.with_suffix(".png"))
    print("\n".join(md))
    print(f"Wrote {path.with_suffix('.md')} and {path.with_suffix('.png')}")


def _plot(stats, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    speeds = sorted({v for _, v in stats})
    fig, axes = plt.subplots(2, len(speeds), figsize=(6 * len(speeds), 9), squeeze=False)
    for col, v in enumerate(speeds):
        ax_path, ax_err = axes[0, col], axes[1, col]
        first = True
        for name in cp.VARIANTS:
            if (name, v) not in stats:
                continue
            m, run = stats[name, v][0]
            if first:
                tt = run.tool_target * 100
                ax_path.plot(tt[:, 1], tt[:, 2], "k--", lw=1, label="target")
                first = False
            sel = (run.t_dense >= run.t_sent[0]) & (run.t_dense <= run.t_sent[-1])
            ax_path.plot(run.tool_dense[sel, 1] * 100, run.tool_dense[sel, 2] * 100, lw=1, label=name)
            ax_err.plot(m["t"], np.linalg.norm(m["tool_err_mm"], axis=1), lw=1, label=name)
        ax_path.set(title=f"{v:g} cm/s: tool path (first repeat)", xlabel="y (cm)", ylabel="z (cm)")
        ax_path.set_aspect("equal")
        ax_path.legend(fontsize=8)
        ax_err.set(title=f"{v:g} cm/s: tool error", xlabel="time (s)", ylabel="mm")
        ax_err.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=100)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["model", "plan", "run", "analyze"])
    ap.add_argument("files", nargs="*", type=Path)
    ap.add_argument("--lead-ms", type=float, default=cp.LEAD_S * 1000)
    args = ap.parse_args()
    lead = args.lead_ms / 1000
    if args.stage == "model":
        files = args.files or sorted(RESULTS.glob("statics_*.npz"))
        model = cp.ServoModel.fit(files)
        model.save()
        print(json.dumps(model.__dict__, indent=1))
        print(f"Wrote {cp.MODEL_PATH}")
    elif args.stage == "plan":
        jm = JointMap.load()
        rest = jm.rest if jm.rest is not None else np.zeros(len(JOINTS))
        _, _, lines, problems = plan(SafetyLayer(), rest, cp.ServoModel.load(), lead)
        print("\n".join(lines + problems))
        print("Preflight: " + ("ok" if not problems else "FAILED"))
    elif args.stage == "run":
        run(lead)
    else:
        for f in args.files:
            analyze(f)


if __name__ == "__main__":
    main()
