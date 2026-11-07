"""so101-touch: S2, joint zero offsets from pointer touches on an iPad dot grid. The arm stays limp.

    so101-touch plan                    no hardware: where to put the iPad, the touches, and how precisely a
                                        simulated arm's offsets come back from them
    so101-touch record fit|check        torque OFF: guide the pointer to each dot, press Enter; the joints
                                        are read -> results/s2/touch_<fit|check>_<time>.npz
    so101-touch push [FILE.npz]         torque OFF: tip pinned in a hole, push the arm around; how far the
                                        tip computed from the joints moves (with FILE: report only)
    so101-touch fit FIT.npz [CHECK.npz] fit offsets, pointer and placement on FIT; test them on CHECK (only
                                        its placement refitted) -> results/s2/<FIT>.md, offsets in .json

`fit` set: the iPad flat on the table (27 touches). `check` set: the iPad flat on a box 10-15 cm high
(12 touches), never used for fitting. The grid: tools/ipad_grid.html, full screen on the iPad.
"""

import argparse
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
from so101_core.model import ARM_JOINTS

from so101_calib import touch as tc

RESULTS = Path("results/s2")
# Grid centre 24 cm ahead of the base-frame origin (20 cm ahead of the shoulder_pan axis). Every touch is
# reachable for 22-26 cm, boxes 10-15 cm high, the iPad turned +-8 deg; at 21 cm on a 12 cm box the near row
# needs shoulder_lift and wrist_flex past their limits.
NOMINAL = tc.Placement(np.radians(-90.0), np.array([0.24, 0.0, 0.007]))
BOX = tc.Placement(np.radians(-90.0), np.array([0.24, 0.0, 0.12]))
WINDOW_S = 0.4  # a touch is the mean of the reads in the last 0.4 s before Enter
STILL_DEG = 0.15  # ... and every arm joint must have been this still (std)


def plan() -> None:
    ptr, rng = tc.Pointer(), np.random.default_rng(0)
    true_off = np.array([0.0, 2.0, -3.0, 2.5, 0.0])
    true_p = np.array([-0.022, 0.003, -0.127])
    print(
        "Place the iPad landscape, flat on the table, long side left-right, row 1 nearest the arm; the grid "
        "centre (between B2 and C2) about 20 cm (18-22) in front of the shoulder_pan axis (turntable centre)."
    )
    print("Then on a box 10-15 cm high, the same way. Distances need not be exact: the placement is fitted.")
    for kind in ("fit", "check"):
        print(f"\n'{kind}' touches:")
        for i, t in enumerate(tc.touch_list(kind)):
            print(
                f"  {i + 1:2d}. dot {t.dot}: pointer {_pitch_word(t.pitch_deg)}, "
                f"gripper {_roll_word(t.roll_deg)}"
            )
    ests, checks = [], []
    for _ in range(10):
        qs, dots, kept = tc.simulate(ptr, tc.touch_list("fit"), NOMINAL, true_off, true_p, rng)
        f = tc.fit(ptr, qs, dots)
        qb, db, _ = tc.simulate(ptr, tc.touch_list("check"), BOX, true_off, true_p, rng)
        ests.append(f.offsets_deg - true_off)
        checks.append(tc.check(ptr, qb, db, f.offsets_deg, f.pointer).rms_mm)
    e = np.array(ests)[:, list(tc.FITTED)]
    print(
        f"\nSimulated (offsets {true_off[list(tc.FITTED)].tolist()} deg, touches 0.5 mm, encoders 0.1 deg; "
        f"{len(kept)}/{len(tc.touch_list('fit'))} fit touches reachable): offsets come back within "
        + ", ".join(f"{ARM_JOINTS[j]} {s:.2f}" for j, s in zip(tc.FITTED, e.std(0), strict=True))
        + f" deg (std); check set RMS {np.mean(checks):.2f} mm"
    )


def _pitch_word(p: float) -> str:
    return "steep (~70 deg down)" if p > 55 else "shallow (~40 deg down)"


def _roll_word(r: float) -> str:
    return "level (roll ~0)" if abs(r) < 1 else f"turned to roll ~{r:+.0f} deg (watch the 'now' line)"


class Reader:
    """Reads the limp arm in the background; shows the gripper pitch and roll on one line; keeps the reads of
    the last seconds so a touch can be taken from just before Enter."""

    def __init__(self, arm, ptr: tc.Pointer):
        self.arm, self.ptr = arm, ptr
        self.buf: list[tuple[float, np.ndarray]] = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.show = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def _loop(self) -> None:
        last_print = 0.0
        while not self.stop.is_set():
            q = self.arm.read()
            now = time.perf_counter()
            with self.lock:
                self.buf.append((now, q))
                self.buf = [b for b in self.buf if b[0] > now - 2.0]
            if self.show.is_set() and now - last_print > 0.2:
                last_print = now
                sys.stdout.write(
                    f"\r    now: pointer {self.ptr.pitch_deg(q):5.1f} deg down, "
                    f"roll {np.degrees(q[4]):+6.1f} deg   "
                )
                sys.stdout.flush()
            time.sleep(0.01)

    def window(self, t_end: float) -> np.ndarray:
        with self.lock:
            return np.array([q for t, q in self.buf if t_end - WINDOW_S <= t <= t_end])


def record(kind: str) -> None:
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap

    touches = tc.touch_list(kind)
    jm = JointMap.load()
    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / f"touch_{kind}_{datetime.now():%Y%m%d_%H%M%S}.npz"
    meta = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "kind": kind,
        "grid_pitch_m": tc.PITCH_M,
        "joint_map_offset_rad": jm.offset.tolist(),
    }
    rec: dict[str, list] = {k: [] for k in ("dot", "xyz", "q", "q_std", "ask_pitch", "ask_roll")}
    where = "flat on the table" if kind == "fit" else "flat on the box (10-15 cm high)"
    print(
        f"{len(touches)} touches. The iPad {where}, landscape, row 1 nearest the arm, grid full screen.\n"
        "The arm goes LIMP: hold it. For each touch: put the pointer tip on the dot's centre, hold still, "
        "press Enter with the other hand.\n"
        "  Enter = record   s + Enter = skip   r + Enter = redo the last one   q + Enter = save and stop"
    )
    if input("\n>>> Ready? The arm will go limp (torque off). Type 'yes': ").strip().lower() != "yes":
        return
    ptr = tc.Pointer()
    with SafeArm(DeviceConfig.load(), joint_map=jm).connect(torque=False) as arm:
        reader = Reader(arm, ptr)
        reader.thread.start()
        i = 0
        while i < len(touches):
            t = touches[i]
            print(
                f"\n[{i + 1}/{len(touches)}] dot {t.dot}: pointer {_pitch_word(t.pitch_deg)}, "
                f"gripper {_roll_word(t.roll_deg)}"
            )
            reader.show.set()
            cmd = input().strip().lower()
            t_enter = time.perf_counter()
            reader.show.clear()
            if cmd == "q":
                break
            if cmd == "s":
                i += 1
                continue
            if cmd == "r":
                if rec["dot"]:
                    for v in rec.values():
                        v.pop()
                    i -= 1
                    print("    removed the last touch")
                continue
            qs = reader.window(t_enter)
            if len(qs) < 5:
                print("    too few reads; try again")
                continue
            std = np.degrees(qs[:, :5].std(0))
            if std.max() > STILL_DEG:
                print(f"    the arm moved (std up to {std.max():.2f} deg); hold still and press Enter again")
                continue
            q = qs.mean(0)
            rec["dot"].append(t.dot)
            rec["xyz"].append(tc.DOTS[t.dot])
            rec["q"].append(q)
            rec["q_std"].append(qs.std(0))
            rec["ask_pitch"].append(t.pitch_deg)
            rec["ask_roll"].append(t.roll_deg)
            np.savez_compressed(out, meta=json.dumps(meta), **{k: np.array(v) for k, v in rec.items()})
            print(
                f"    recorded: pointer {ptr.pitch_deg(q):.0f} deg down, roll {np.degrees(q[4]):+.0f} deg "
                f"({len(qs)} reads)"
            )
            i += 1
        reader.stop.set()
        reader.thread.join(timeout=1.0)
        input("\n>>> Done. Rest the arm in its rest pose (or hold it), then press Enter. ")
    print(f"Saved {len(rec['dot'])} touches to {out}")


PUSH_PHASES = (
    ("still", "hold the arm lightly, no push"),
    ("left", "push the arm to ITS left (sideways), as hard as for the last mm of a touch"),
    ("right", "push to its right"),
    ("away", "push away from the base"),
    ("toward", "push toward the base"),
    ("down", "push down (into the hole)"),
    ("still2", "hold lightly again, no push"),
)
PUSH_S = 3.0


def push() -> None:
    """Pointer tip pinned in a small hole, the arm limp: push it in several directions and read the
    joints. The tip cannot move, so any change in the tip computed from the joints is motion the encoders
    read while the structure gives (printed parts, horns, the base on the table): the error of a pushed
    touch."""
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap

    print(
        "Make a small hole the pointer tip cannot slide out of (poke it into cardboard or an eraser\n"
        "taped to the table, about 20 cm in front of the base). Put the tip in it, pointer steep, gripper\n"
        f"level. Then {len(PUSH_PHASES)} phases of {PUSH_S:.0f} s: press Enter to start each, keep the tip\n"
        "in the hole, do not tilt or turn the gripper on purpose."
    )
    if input("\n>>> Ready? The arm will go limp (torque off). Type 'yes': ").strip().lower() != "yes":
        return
    jm = JointMap.load()
    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / f"push_{datetime.now():%Y%m%d_%H%M%S}.npz"
    rows: dict[str, list] = {"phase": [], "t": [], "q": []}
    with SafeArm(DeviceConfig.load(), joint_map=jm).connect(torque=False) as arm:
        for name, what in PUSH_PHASES:
            input(f"\n[{name}] {what}. Press Enter, then keep it up for {PUSH_S:.0f} s. ")
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < PUSH_S:
                rows["phase"].append(name)
                rows["t"].append(time.perf_counter() - t0)
                rows["q"].append(arm.read() - jm.offset)
            print("    done")
        np.savez_compressed(out, **{k: np.array(v) for k, v in rows.items()})
        input("\n>>> Done. Rest the arm in its rest pose (or hold it), then press Enter. ")
    print(f"Saved {out}\n")
    push_report(out)


def push_report(path: Path, fit_json: Path | None = None) -> None:
    z = np.load(path)
    ptr = tc.Pointer()
    off, p = np.zeros(5), tc.POINTER_GUESS
    fits = sorted(RESULTS.glob("touch_fit_*_fit.json")) if fit_json is None else [fit_json]
    if fits:
        js = json.loads(fits[-1].read_text())
        off = np.radians([js["offsets_deg"][j] for j in ARM_JOINTS])
        p = np.array(js["pointer_m"])
    phase, q = z["phase"], z["q"][:, :5]
    tips = np.array([ptr.tip(x + off, p) for x in q]) * 1000
    ref_tip = tips[phase == "still"].mean(0)
    ref_q = q[phase == "still"].mean(0)
    print(f"pointer {np.round(p * 1000, 1).tolist()} mm, offsets {np.round(np.degrees(off), 2).tolist()} deg")
    print("phase    | tip moved (computed), mean / max mm | joint change from still (deg)")
    for name, _ in PUSH_PHASES:
        m = phase == name
        if not m.any():
            continue
        d = np.linalg.norm(tips[m] - ref_tip, axis=1)
        dq = np.degrees(q[m].mean(0) - ref_q)
        print(f"{name:8s} | {d.mean():5.2f} / {d.max():5.2f} | " + " ".join(f"{x:+.2f}" for x in dq))


def load(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    """Joint readings without any joint-map offset (as recorded, minus the offsets in use then), dots."""
    z = np.load(path)
    meta = json.loads(str(z["meta"]))
    q = z["q"] - np.array(meta["joint_map_offset_rad"])
    return q, z["xyz"], meta


def fit_cmd(fit_path: Path, check_path: Path | None) -> None:
    ptr = tc.Pointer()
    qs, dots, _ = load(fit_path)
    after = tc.fit(ptr, qs, dots)
    before = tc.fit(ptr, qs, dots, fit_offsets=False)
    rows = [
        f"| fit set: {fit_path.name} ({len(qs)} touches) | {before.rms_mm:.2f} / {before.max_mm:.2f} | "
        f"{after.rms_mm:.2f} / {after.max_mm:.2f} |"
    ]
    dist = None
    if check_path is not None:
        qb, db, _ = load(check_path)
        cb = tc.check(ptr, qb, db, before.offsets_deg, before.pointer)
        ca = tc.check(ptr, qb, db, after.offsets_deg, after.pointer)
        rows.append(
            f"| check set: {check_path.name} ({len(qb)} touches, not fitted) | {cb.rms_mm:.2f} / "
            f"{cb.max_mm:.2f} | {ca.rms_mm:.2f} / {ca.max_mm:.2f} |"
        )
        d0 = tc.distance_errors_mm(ptr, qb, db, before.offsets_deg, before.pointer)
        d1 = tc.distance_errors_mm(ptr, qb, db, after.offsets_deg, after.pointer)
        dist = (d0, d1)
    off = ", ".join(
        f"{ARM_JOINTS[j]} {after.offsets_deg[j]:+.2f} +- {after.offsets_std_deg[j]:.2f}" for j in tc.FITTED
    )
    md = [
        f"# S2 joint zero offsets: {fit_path.name}",
        "",
        "Model: FK(q + offset, pointer) = dot on a flat grid (yaw and position fitted). Before: offsets 0, "
        "pointer and placement fitted. Check: offsets and pointer from the fit set, only the placement "
        "refitted.",
        "",
        f"Offsets (deg, add to the reading): {off}.",
        f"Pointer tip (mm, gripper frame): {np.round(after.pointer * 1000, 1).tolist()}.",
        f"Grid on the table: centre {np.round(after.placement.t * 100, 1).tolist()} cm, "
        f"yaw {np.degrees(after.placement.yaw):.1f} deg.",
        "",
        "| Touches | Before: RMS / max (mm) | After |",
        "|---|---:|---:|",
        *rows,
        "",
    ]
    if dist is not None:
        d0, d1 = dist
        md += [
            "Distances between check-set touches (FK minus true, every pair; the ruler check without a "
            f"placement): before mean {d0.mean():+.2f} mm, RMS {np.sqrt((d0**2).mean()):.2f}; after mean "
            f"{d1.mean():+.2f} mm, RMS {np.sqrt((d1**2).mean()):.2f}.",
            "",
        ]
    per = [
        f"| {d} | {np.linalg.norm(r):.2f} |"
        for d, r in zip(np.load(fit_path)["dot"], after.resid_mm, strict=True)
    ]
    md += ["Residual per fit touch (mm):", "", "| Dot | Residual |", "|---|---:|", *per, ""]
    out = RESULTS / f"{fit_path.stem}_fit.md"
    out.write_text("\n".join(md))
    js = {
        "fit": fit_path.name,
        "check": None if check_path is None else check_path.name,
        "offsets_deg": dict(zip(ARM_JOINTS, np.round(after.offsets_deg, 3).tolist(), strict=True)),
        "offsets_std_deg": dict(zip(ARM_JOINTS, np.round(after.offsets_std_deg, 3).tolist(), strict=True)),
        "pointer_m": np.round(after.pointer, 5).tolist(),
    }
    out.with_suffix(".json").write_text(json.dumps(js, indent=1) + "\n")
    print("\n".join(md))
    print(f"Wrote {out} and {out.with_suffix('.json')}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["plan", "record", "fit", "push"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    if a.stage == "plan":
        plan()
    elif a.stage == "push":
        if a.args:
            push_report(Path(a.args[0]))
        else:
            push()
    elif a.stage == "record":
        if a.args not in (["fit"], ["check"]):
            raise SystemExit("so101-touch record fit|check")
        record(a.args[0])
    else:
        if not a.args:
            raise SystemExit("so101-touch fit FIT.npz [CHECK.npz]")
        fit_cmd(Path(a.args[0]), Path(a.args[1]) if len(a.args) > 1 else None)


if __name__ == "__main__":
    main()
