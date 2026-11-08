"""so101-servo: S1a servo identification on the real arm. Every motion goes through so101-core's SafeArm.

    so101-servo plan                      no hardware: the test program, its duration, the preflight checks
    so101-servo run [--joints a b ...] [--p 32]
                                          torque ON: move to the test pose, run the program on each joint,
                                          save every read to results/s1/servo_<time>.npz
    so101-servo analyze FILE.npz          step and sine metrics -> FILE.md and FILE.png

`run` prints the plan and asks for `yes` before the arm is powered and again before it moves. Run with
SO101_CONFIG_DIR pointing at so101-core's config (device ports).
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from so101_core.arm import SafeArm
from so101_core.config import DeviceConfig, JointMap
from so101_core.model import ARM_JOINTS, JOINTS, urdf_limits
from so101_core.safety import SafetyLayer
from so101_kinematics.models import PinArm

from so101_calib import servo_id as sid

RESULTS = Path("results/s1")
MOVE_SPEED = np.radians(20.0)  # move_to between poses (its default)
STALL_RAD = np.radians(5.0)  # follow's default


def ask_yes(message: str) -> bool:
    return input(f"\n>>> {message} Type 'yes' to continue: ").strip().lower() == "yes"


def test_pose(safety: SafetyLayer, grip: float) -> np.ndarray:
    q0 = np.append(np.radians(sid.TEST_POSE_DEG), grip)
    return safety.clip(q0)[0]


def check_program(safety: SafetyLayer, q0: np.ndarray, joints: list[int]) -> list[str]:
    """Problems that would make the program unsafe or the data meaningless; empty if none."""
    hz = safety.cfg.rate_hz
    problems = []
    for j in joints:
        ticks, _ = sid.program(q0, j, hz)
        q = ticks[0]
        for target in ticks[1:]:
            res = safety.filter(target, q)
            if res.clipped.any() or res.step_limited.any():
                problems.append(f"{JOINTS[j]}: the safety layer would change the program (limit or step cap)")
                break
            q = res.q
        worst = sid.expected_following_error(ticks, hz)[j]
        if worst > 0.8 * STALL_RAD:
            problems.append(
                f"{JOINTS[j]}: expected following error {np.degrees(worst):.1f} deg is close to the "
                f"stall threshold {np.degrees(STALL_RAD):.0f} deg"
            )
    return problems


def describe_plan(safety: SafetyLayer, q_now: np.ndarray, joints: list[int]) -> tuple[np.ndarray, list[str]]:
    hz = safety.cfg.rate_hz
    q0 = test_pose(safety, q_now[5])
    kin = PinArm()
    ticks, segments = sid.program(q0, joints[0], hz)
    lines = [
        f"Test pose (deg): {np.round(np.degrees(q0), 1)}; "
        f"tool at {np.round(kin.fk(q0[:5])[:3, 3] * 100, 1)} cm "
        "(x forward, z up, base frame)",
        f"Per joint: steps of {sid.STEP_SIZES_DEG} deg up and down ({sid.STEP_HOLD_S:.0f} s each), "
        "then sines of "
        f"{sid.SINE_AMP_DEG:.0f} deg at {sid.SINE_FREQS_HZ} Hz: {len(ticks) / hz:.0f} s per joint, "
        f"{len(ticks) / hz * len(joints) / 60:.1f} min for {len(joints)} joints",
        f"Largest joint motion: +-{max(sid.SINE_AMP_DEG, max(sid.STEP_SIZES_DEG)):.0f} deg "
        "around the test pose; "
        "steps happen in one tick; sine peak speed "
        f"{2 * np.pi * max(sid.SINE_FREQS_HZ) * sid.SINE_AMP_DEG:.0f} deg/s",
        "Expected following error (M1 model: 100 ms delay + 1 deg sag), deg: "
        + ", ".join(
            f"{JOINTS[j]} {np.degrees(sid.expected_following_error(sid.program(q0, j, hz)[0], hz)[j]):.1f}"
            for j in joints
        )
        + f" (stall at {np.degrees(STALL_RAD):.0f})",
    ]
    path = sid.ramp_path(safety.clip(q_now)[0], q0, MOVE_SPEED / hz)
    z = np.array([kin.fk(q[:5])[2, 3] for q in path]) * 100
    lines.append(
        f"Move from the current pose to the test pose: {len(path) / hz:.0f} s; tool height along the way "
        f"{z.min():.1f} to {z.max():.1f} cm"
    )
    return q0, lines


def run(joints: list[int], position_p: int | None = None) -> None:
    devices = DeviceConfig.load()
    jm = JointMap.load()
    arm = SafeArm(devices, joint_map=jm, position_p=position_p)
    safety = arm.safety
    hz = safety.cfg.rate_hz
    rest = jm.rest
    if rest is None:
        raise SystemExit("No rest pose in the joint map; record one with so101-core first.")
    q0, lines = describe_plan(safety, rest, joints)
    print("\n".join(lines))
    problems = check_program(safety, q0, joints)
    if problems:
        raise SystemExit("Preflight failed:\n  " + "\n  ".join(problems))
    print("Clear about 40 cm in front of the arm. Stay next to it with the power switch in reach.")
    if not ask_yes("The arm will be POWERED (torque on)."):
        return
    RESULTS.mkdir(parents=True, exist_ok=True)
    tag = "" if position_p is None else f"_p{position_p}"
    out = RESULTS / f"servo_{datetime.now():%Y%m%d_%H%M%S}{tag}.npz"
    data: dict[str, np.ndarray] = {}
    meta = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "rate_hz": hz,
        "test_pose_rad": q0.tolist(),
        "position_p": position_p,
        "joints": [],
        "regs": {},
    }
    with arm.connect(torque=True):
        bus = arm.robot.bus
        for reg in ("P_Coefficient", "D_Coefficient", "Present_Voltage", "Present_Temperature"):
            meta["regs"][reg] = {k: int(v) for k, v in bus.sync_read(reg, normalize=False).items()}
        print(f"Servo settings: {meta['regs']}")
        _, lines = describe_plan(safety, arm.read(), joints)
        print(lines[-1])
        if not ask_yes("The arm will MOVE to the test pose and then run the program."):
            return
        arm.move_to(q0, speed_rad_s=MOVE_SPEED)
        for j in joints:
            name = JOINTS[j]
            ticks, segments = sid.program(arm.q_cmd, j, hz)
            print(f"{name}: {len(ticks) / hz:.0f} s ...", flush=True)
            t_start = time.perf_counter()
            res = arm.follow(ticks, dense=True)
            load = {k: int(v) for k, v in bus.sync_read("Present_Load", normalize=False).items()}
            temp = {k: int(v) for k, v in bus.sync_read("Present_Temperature", normalize=False).items()}
            data |= {
                f"{name}/t": res.t,
                f"{name}/t_sent": res.t_sent,
                f"{name}/q_cmd": res.q_cmd,
                f"{name}/q_meas": res.q_meas,
                f"{name}/t_dense": res.t_dense,
                f"{name}/q_dense": res.q_dense,
                f"{name}/segments": sid.segments_to_array(segments),
            }
            meta["joints"].append(name)
            meta[f"{name}/load_after"] = load
            meta[f"{name}/temp_after"] = temp
            meta[f"{name}/filter"] = {"clipped": res.clipped, "step_limited": res.step_limited}
            np.savez_compressed(
                out, meta=json.dumps(meta), **data
            )  # after every joint: a stall keeps the rest
            dt = np.diff(res.t_dense)
            print(
                f"  done in {time.perf_counter() - t_start:.1f} s; {len(res.t_dense)} reads, median "
                f"{1000 * np.median(dt):.2f} ms apart; "
                f"filter clipped {res.clipped}, capped {res.step_limited}; "
                f"temperatures {list(temp.values())} C"
            )
        print(f"Saved {out}. Moving back towards the rest pose.")
        arm.move_to(rest, speed_rad_s=MOVE_SPEED)
    print(f"\nDone. Analyze with: uv run so101-servo analyze {out}")


# -- analysis ------------------------------------------------------------------------------------------------
def _fmt(v, nd=1):
    return "-" if v is None else f"{v:.{nd}f}"


def analyze(path: Path) -> None:
    z = np.load(path)
    meta = json.loads(str(z["meta"]))
    hz = meta["rate_hz"]
    rows_step, rows_sine, rows_summary = [], [], []
    plots = []
    for name in meta["joints"]:
        j = JOINTS.index(name)
        t_sent, q_cmd = z[f"{name}/t_sent"], z[f"{name}/q_cmd"]
        t_dense, q_dense = z[f"{name}/t_dense"], z[f"{name}/q_dense"]
        segs = sid.segments_from_array(z[f"{name}/segments"])
        steps = [
            sid.step_metrics(t_sent, q_cmd, t_dense, q_dense, s, j, hz) for s in segs if s.kind == "step"
        ]
        sines = [sid.sine_metrics(t_sent, q_cmd, t_dense, q_dense, s, j) for s in segs if s.kind == "sine"]
        for m in steps:
            rows_step.append(
                f"| {name} | {m['step_deg']:+.1f} | {m['moved_deg']:+.2f} | {m['shortfall_deg']:+.2f} | "
                f"{m['error_deg']:+.2f} | {_fmt(m['t_first_ms'], 0)} | {_fmt(m['t10_ms'], 0)} | "
                f"{_fmt(m['t50_ms'], 0)} | {_fmt(m['t90_ms'], 0)} |"
            )
        peaks = [
            sid.sine_peak_errors(t_sent, q_cmd, t_dense, q_dense, s, j) for s in segs if s.kind == "sine"
        ]
        for m, (top, bottom) in zip(sines, peaks, strict=True):
            m["top"], m["bottom"] = top, bottom
        for m in sines:
            rows_sine.append(
                f"| {name} | {m['freq_hz']:.2f} | {m['gain']:.2f} | "
                f"{m['phase_deg']:.0f} | {m['delay_ms']:.0f} | {m['top']:+.2f} | {m['bottom']:+.2f} |"
            )
        big = [m for m in steps if abs(m["step_deg"]) >= 1.0 and m["t50_ms"] is not None]
        coup = sid.coupling_deg(q_cmd, q_dense, t_dense, t_sent, j)
        worst = max(coup, key=coup.get)
        firsts = [m["t_first_ms"] for m in steps if m["t_first_ms"] is not None]
        jitter = max(sid.hold_jitter_deg(t_sent, t_dense, q_dense, s, j) for s in segs if s.kind == "hold")
        temp = meta.get(f"{name}/temp_after", {}).get(name)
        top, bottom = np.mean(peaks, axis=0)
        rows_summary.append(
            f"| {name} | {np.median(firsts):.0f} | "
            f"{_fmt(np.median([m['t50_ms'] for m in big]) if big else None, 0)} | "
            f"{top:+.2f} | {bottom:+.2f} | {(top + bottom) / 2:+.2f} | {(top - bottom) / 2:.2f} | "
            f"{jitter:.2f} | {_fmt(temp, 0)} | "
            f"{np.median([m['delay_ms'] for m in sines]):.0f} | "
            f"{np.mean([m['error_deg'] for m in steps]):+.2f} | "
            f"{np.mean([abs(m['shortfall_deg']) for m in steps]):.2f} | {worst} {coup[worst]:.2f} |"
        )
        plots.append((name, j, t_sent, q_cmd, t_dense, q_dense, segs))
        dt = np.diff(t_dense)
    md = [
        f"# S1a servo identification: {path.name}",
        "",
        f"Date {meta['date']}; position gain P = {meta.get('position_p') or 16}; "
        f"command rate {hz:.0f} Hz; reads {1000 * np.median(dt):.2f} ms apart (median, "
        f"last joint). Test pose (deg) {np.round(np.degrees(meta['test_pose_rad']), 1).tolist()}. "
        f"Servo registers: {meta['regs']}.",
        "",
        "Times are from the moment the command was written. A command is held for one tick (33 ms), so the "
        "servo sees a staircase: against the smooth commanded sine that alone adds half a tick (17 ms).",
        "",
        "## Summary per joint",
        "",
        "t_first: from the step command to the first change of 1.5 encoder counts. Center = mean of the "
        "errors at the two turning points (the static offset, e.g. gravity sag); half-width = half their "
        "difference (the band the servo can rest anywhere in: deadband and friction). "
        "Jitter: largest peak to peak of the joint over the last 0.5 s of a hold "
        "(oscillation would show here; one count is "
        f"{sid.COUNT_DEG:.2f} deg).",
        "",
        "| Joint | Step t_first (ms, median) | Step t50 (ms, steps >= 1 deg) | Error at sine top (deg) | "
        "Error at sine bottom | Center | Half-width | Jitter at rest (deg p-p) | Temp after (C) | "
        "Sine delay (ms, median) | "
        "Mean error after settling (deg) | Mean shortfall (deg) | Other joint moved most (deg p-p) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
        *rows_summary,
        "",
        "## Steps",
        "",
        "Error = commanded - measured after settling (sag + direction-dependent part). Shortfall = step - "
        "measured change (sag cancels). t_first = first change of >= 1.5 encoder counts "
        f"({1.5 * sid.COUNT_DEG:.2f} deg).",
        "",
        "| Joint | Step (deg) | Moved | Shortfall | Error | t_first (ms) | t10 | t50 | t90 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        *rows_step,
        "",
        "## Sines",
        "",
        f"Amplitude {sid.SINE_AMP_DEG} deg; first cycle skipped. Delay = phase lag / (2 pi f). "
        "Error at top / "
        "bottom = commanded - measured at the turning points.",
        "",
        "| Joint | f (Hz) | Gain | Phase lag (deg) | Delay (ms) | Error at top (deg) | Error at bottom |",
        "|---|---:|---:|---:|---:|---:|---:|",
        *rows_sine,
        "",
    ]
    path.with_suffix(".md").write_text("\n".join(md))
    _plot(plots, path.with_suffix(".png"))
    print("\n".join(md))
    print(f"Wrote {path.with_suffix('.md')} and {path.with_suffix('.png')}")


def _plot(plots, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(len(plots), 2, figsize=(13, 2.6 * len(plots)), squeeze=False)
    for row, (name, j, t_sent, q_cmd, t_dense, q_dense, segs) in zip(axes, plots, strict=True):
        ax = row[0]
        for s in segs:
            if s.kind != "step" or abs(s.value) < 1.0:
                continue
            t0 = t_sent[s.start]
            m = (t_dense >= t0 - 0.05) & (t_dense < t0 + 0.4)
            base = np.mean(np.degrees(q_dense[(t_dense > t0 - 0.2) & (t_dense < t0), j]))
            ax.plot(1000 * (t_dense[m] - t0), (np.degrees(q_dense[m, j]) - base) / s.value, lw=0.8)
        ax.axhline(1, color="k", lw=0.5)
        ax.axvline(0, color="k", lw=0.5)
        ax.set(title=f"{name}: steps >= 1 deg, normalized", xlabel="ms after the command", ylabel="fraction")
        ax = row[1]
        sines = [s for s in segs if s.kind == "sine"]
        t_a, t_b = t_sent[sines[0].start], t_sent[sines[-1].end - 1]
        c = (t_sent >= t_a) & (t_sent <= t_b)
        m = (t_dense >= t_a) & (t_dense <= t_b)
        ax.step(t_sent[c] - t_a, np.degrees(q_cmd[c, j]), where="post", lw=0.8, label="command")
        ax.plot(t_dense[m] - t_a, np.degrees(q_dense[m, j]), lw=0.8, label="measured")
        ax.set(title=f"{name}: sines", xlabel="s", ylabel="deg")
        ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("plan")
    r = sub.add_parser("run")
    r.add_argument("--joints", nargs="+", default=list(ARM_JOINTS), choices=ARM_JOINTS)
    r.add_argument("--p", type=int, default=None, help="servo position gain (default: LeRobot's 16)")
    a = sub.add_parser("analyze")
    a.add_argument("file", type=Path)
    args = p.parse_args()
    if args.cmd == "plan":
        jm = JointMap.load()
        safety = SafetyLayer(limits=jm.model_limits(*urdf_limits()))  # as SafeArm builds it
        joints = list(range(len(ARM_JOINTS)))
        q_from = jm.rest if jm.rest is not None else np.zeros(len(JOINTS))
        q0, lines = describe_plan(safety, q_from, joints)
        print("\n".join(lines))
        problems = check_program(safety, q0, joints)
        print("Preflight: " + ("ok" if not problems else "\n  " + "\n  ".join(problems)))
    elif args.cmd == "run":
        run([JOINTS.index(n) for n in args.joints], args.p)
    else:
        analyze(args.file)


if __name__ == "__main__":
    main()
