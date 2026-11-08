"""so101-budget: S5, the error budget over the workspace and the click-to-point demo.

so101-budget plan               no hardware: can the arm look at each suggested placement? (preview)
so101-budget run                POWERED. Put the iPad down where the ZED view shows the outline, press
                                space; the ZED sees it, the arm looks at it from 3-5 poses and returns to
                                rest. 5 placements -> captures/s5/<time>/p1..p5
so101-budget fit DIR [S4 DIRs]  the ZED refitted to all placements; leave-one-placement-out errors of the
                                ZED's ray-plane points against the wrist chain; the budget; a heat map
                                -> results/s5/
so101-budget point [--table]    POWERED demo: click in the ZED image; the tool point (between the jaw tips)
                                goes 1 cm above the clicked point on the iPad (or the table)
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

from so101_calib import SERVO_P
from so101_calib import budget as bu
from so101_calib import charuco as ch
from so101_calib import compensation as cp
from so101_calib import handeye as he
from so101_calib import statics as st
from so101_calib.handeye_cli import (
    MOVE_SPEED,
    Grabber,
    capture_at,
    load_samples,
    show,
    wrist_intrinsics,
    zed_board,
    zed_left,
    zed_views,
)
from so101_calib.touch import Pointer

CAPTURES = Path("captures/s5")
RESULTS = Path("results/s5")
HANDEYE = Path("results/s4/handeye.yaml")
BUDGET = RESULTS / "budget.yaml"
# Board centres (m, base frame) to put the iPad at, spread over the reachable part of the ZED's view.
SUGGEST = [(0.28, -0.08), (0.18, -0.14), (0.30, 0.10), (0.16, 0.12), (0.34, 0.0)]
BOARD_W, BOARD_H = ch.SQUARES_X * ch.SQUARE_M, ch.SQUARES_Y * ch.SQUARE_M
POINT_SPEED = np.radians(8.0)
HOVER_M, ABOVE_M = 0.06, 0.01


# -- calibration in hand -------------------------------------------------------------------------------------
class Calib:
    """S4's wrist camera and offsets (relative to LeRobot readings), and the ZED (S5's if fitted, else
    S4's)."""

    def __init__(self):
        h = yaml.safe_load(HANDEYE.read_text())
        self.X = np.array(h["T_gripper_wristcam"])
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
        self.off = np.radians([h["joint_offsets_deg"][j] for j in names])
        self.zed = np.array(h["T_base_zed_left"])
        self.table_z = h.get("table_z_m")
        self.ipad_z = float(np.mean([np.array(Y)[2, 3] for Y in h["T_base_board"].values()]))
        self.zed_source = "S4"
        if BUDGET.exists():
            b = yaml.safe_load(BUDGET.read_text())
            self.zed, self.ipad_z, self.zed_source = np.array(b["T_base_zed_left"]), b["ipad_z_m"], "S5"
            self.table_z = b.get("table_z_m", self.table_z)

    def extra(self, jm) -> np.ndarray:
        """Offsets not yet in the joint map: true angle = SafeArm angle + this."""
        return self.off - jm.offset[:5]


def board_at(x: float, y: float, yaw_deg: float = 0.0, z: float = 0.0086) -> np.ndarray:
    """A flat board pose with its centre at (x, y)."""
    Y = np.eye(4)
    Y[:3, :3] = he.rot_z(np.radians(yaw_deg)) @ he.FLAT_R
    Y[:3, 3] = np.array([x, y, z]) - Y[:3, :3] @ np.array([BOARD_W / 2, BOARD_H / 2, 0.0])
    return Y


def outline_px(Y: np.ndarray, T_base_zed: np.ndarray, intr) -> np.ndarray:
    pts = np.array([[0, 0, 0], [BOARD_W, 0, 0], [BOARD_W, BOARD_H, 0], [0, BOARD_H, 0]], float)
    T = he.inv(T_base_zed) @ Y
    p, _ = cv2.projectPoints(pts, *he.rvec_t(T), intr.K, intr.D)
    return p.reshape(-1, 2).astype(np.int32)


def ask_yes(msg: str) -> bool:
    return input(f"\n>>> {msg} Type 'yes': ").strip().lower() == "yes"


# -- plan (preview) ------------------------------------------------------------------------------------------
def plan_cmd() -> None:
    from so101_core.config import JointMap
    from so101_core.model import urdf_limits
    from so101_core.safety import SafetyLayer

    jm, cal = JointMap.load(), Calib()
    safety = SafetyLayer(limits=jm.model_limits(*urdf_limits()))
    rest = safety.clip(jm.rest)[0]
    rng = np.random.default_rng(0)
    for i, (x, y) in enumerate(SUGGEST, 1):
        vp = bu.plan_views(
            board_at(x, y), cal.X, wrist_intrinsics(), Pointer(), safety, rest, cal.extra(jm), rng
        )
        print(
            f"p{i} at ({x * 100:.0f}, {y * 100:+.0f}) cm: {len(vp.poses)} views "
            f"({vp.n_candidates} candidates) {'; '.join(vp.problems)}"
        )


# -- run -----------------------------------------------------------------------------------------------------
def place_board(zed: Grabber, intr, T_base_zed, target: np.ndarray, k: int, n: int) -> list | None:
    """Live ZED view with the suggested outline until the board is seen and SPACE is pressed (q: stop)."""
    det = ch.detector()
    print(
        f"\nPlacement {k}/{n}: put the iPad (flat, any turn) on the green outline in the ZED "
        "window, then SPACE."
    )
    while True:
        f = zed.latest()
        left = cam_left(f)
        v = ch.detect(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), det)
        disp = left.copy()
        cv2.polylines(disp, [outline_px(target, T_base_zed, intr)], True, (0, 220, 0), 2)
        text = f"placement {k}/{n}: no board"
        if v is not None:
            c = bu.board_centre(T_base_zed @ he.pnp(v, intr)) * 100
            text = f"placement {k}/{n}: {len(v.ids)} corners at ({c[0]:.0f}, {c[1]:+.0f}) cm, SPACE = here"
        key = show("so101-budget: ZED left", disp, v, text)
        if key == ord("q"):
            return None
        if key == ord(" ") and v is not None and len(v.ids) >= 20:
            return [zed.latest(after=time.perf_counter()) for _ in range(10)]


def cam_left(frame: np.ndarray) -> np.ndarray:
    return frame[:, : frame.shape[1] // 2]


def run() -> None:
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap

    devices, jm, cal = DeviceConfig.load(), JointMap.load(), Calib()
    extra = cal.extra(jm)
    intr, zi, ptr = wrist_intrinsics(), zed_left(), Pointer()
    out = CAPTURES / datetime.now().strftime("%Y%m%d_%H%M%S")
    print(
        f"S5: {len(SUGGEST)} placements of the iPad (board full screen, brightness ~50 %, in its case).\n"
        f"The ZED must not move (outlines from the {cal.zed_source} pose). The arm waits at rest, POWERED,\n"
        "while you place the iPad: do not touch the arm. After SPACE it looks at the iPad from 3-5 poses\n"
        "and comes back."
    )
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    rest = arm.safety.clip(jm.rest)[0]
    wrist, zed = Grabber("wrist"), Grabber("zed")
    det, rng = ch.detector(), np.random.default_rng(int(time.time()))
    done = 0
    try:
        if not ask_yes("The arm will be POWERED (torque on) for the whole session."):
            return
        with arm.connect(torque=True):
            arm.move_to(rest, speed_rad_s=MOVE_SPEED)
            while done < len(SUGGEST):
                x, y = SUGGEST[done]
                frames = place_board(zed, zi, cal.zed, board_at(x, y), done + 1, len(SUGGEST))
                if frames is None:
                    break
                Tz, nz = zed_board(frames)
                if Tz is None:
                    print("The ZED lost the board; try again.")
                    continue
                Yr = bu.flatten(cal.zed @ Tz)
                c = bu.board_centre(Yr) * 100
                print(f"Board at ({c[0]:.0f}, {c[1]:+.0f}) cm by the ZED. Planning views ...", flush=True)
                vp = bu.plan_views(Yr, cal.X, intr, ptr, arm.safety, rest, extra, rng)
                if not vp.poses:
                    print(
                        "Cannot look at it there ("
                        + "; ".join(vp.problems)
                        + "): move it nearer the outline."
                    )
                    continue
                pdir = out / f"p{done + 1}"
                pdir.mkdir(parents=True, exist_ok=True)
                for i, f in enumerate(frames[:3]):
                    cv2.imwrite(str(pdir / f"zed_before_{i}.png"), f)
                print(f"{len(vp.poses)} views; the arm moves now.")
                samples = []
                for k, q in enumerate(vp.poses):
                    s = capture_at(arm, q, wrist, pdir, k, det)
                    if s:
                        samples.append(s)
                arm.move_to(rest, speed_rad_s=MOVE_SPEED)
                after = [zed.latest(after=time.perf_counter()) for _ in range(10)]
                for i, f in enumerate(after[:3]):
                    cv2.imwrite(str(pdir / f"zed_after_{i}.png"), f)
                Ta, _ = zed_board(after)
                moved = np.linalg.norm(Ta[:3, 3] - Tz[:3, 3]) * 1000 if Ta is not None else float("nan")
                meta = {
                    "date": datetime.now().isoformat(timespec="seconds"),
                    "position_p": SERVO_P,
                    "joint_map_offset_rad": jm.offset.tolist(),
                    "Y_from_zed": Yr.tolist(),
                    "samples": [
                        {kk: (v.tolist() if isinstance(v, np.ndarray) else v) for kk, v in s.items()}
                        for s in samples
                    ],
                }
                (pdir / "samples.json").write_text(json.dumps(meta, indent=1))
                seen = sum(s["corners"] >= 12 for s in samples)
                print(
                    f"p{done + 1}: {seen}/{len(samples)} wrist views see the board; it moved {moved:.1f} mm."
                )
                done += 1
            print("\nDone. Support the arm: torque goes off next.")
    finally:
        wrist.close()
        zed.close()
        cv2.destroyAllWindows()
    if done:
        print(f"\nSaved {done} placements in {out}. Fit with: uv run so101-budget fit {out}")


# -- fit -----------------------------------------------------------------------------------------------------
def placements(dirs: list[Path]) -> list[Path]:
    out = []
    for d in dirs:
        out += [d] if (d / "samples.json").exists() else sorted(p.parent for p in d.glob("p*/samples.json"))
    return out


def wrist_boards(folders: list[Path], cal: Calib, intr, ptr) -> tuple[he.Fit, list[he.Sample]]:
    """All placements through the wrist chain at once, X and the offsets held fixed, one table height."""
    samples = []
    for b, folder in enumerate(folders):
        meta = json.loads((folder / "samples.json").read_text())
        jm_off = np.asarray(meta.get("joint_map_offset_rad", np.zeros(6)), float)
        for s in load_samples(folder, meta["samples"], intr, b):
            s.q = s.q.copy()
            s.q[:5] -= jm_off[:5]  # back to LeRobot readings, as the offsets are
            samples.append(s)
    return he.fit(samples, intr, ptr, fixed_X_off=(cal.X, cal.off), shared_z=True), samples


def ik_tool(
    ptr: Pointer,
    target: np.ndarray,
    q0: np.ndarray,
    lo,
    hi,
    pitches=(90, 75, 60),
    tool: np.ndarray = bu.TOOL,
) -> np.ndarray | None:
    """True joint angles (5,) putting `tool` (gripper frame) at target, the gripper pointing down as steeply
    as it can (pitch below horizontal, deg), wrist_roll 0. None if out of reach."""
    for pitch in pitches:

        def f(q, pitch=pitch):
            return np.concatenate(
                [
                    (ptr.tip(q, tool) - target) * 1000,
                    [0.05 * (ptr.pitch_deg(q) - pitch), 0.2 * np.degrees(q[4])],
                ]
            )

        r = least_squares(f, np.clip(q0[:5], lo + 1e-3, hi - 1e-3), bounds=(lo, hi))
        if np.linalg.norm(r.fun[:3]) < 0.5:
            return r.x
    return None


def heat_map(names, Ys, errs, zed_T, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    rms = [float(np.sqrt((e**2).sum(1).mean())) for e in errs]
    cmap, top = plt.get_cmap("viridis"), max(max(rms), 1e-6)
    fig, ax = plt.subplots(figsize=(7, 6))
    for name, Y, e, r in zip(names, Ys, errs, rms, strict=True):
        corners = np.array([[0, 0, 0], [BOARD_W, 0, 0], [BOARD_W, BOARD_H, 0], [0, BOARD_H, 0]])
        pts = (Y[:3, :3] @ corners.T).T + Y[:3, 3]
        ax.add_patch(Polygon(pts[:, :2] * 100, closed=True, color=cmap(r / top), alpha=0.75))
        c = bu.board_centre(Y) * 100
        m = e.mean(0)
        ax.annotate("", xy=(c[0] + m[0], c[1] + m[1]), xytext=(c[0], c[1]), arrowprops={"arrowstyle": "->"})
        ax.text(c[0], c[1] - 1.5, f"{name}\n{r:.1f} mm", ha="center", va="top", fontsize=8)
    ax.plot([0], [0], "ks")
    ax.text(0.5, 0.5, "base", fontsize=8)
    z = zed_T[:3, 3] * 100
    ax.plot([z[0]], [z[1]], "r^")
    ax.text(z[0], z[1] + 1, "ZED", color="r", fontsize=8, ha="center")
    ax.set_aspect("equal")
    ax.set_xlim(-5, 45)
    ax.set_ylim(-25, 35)
    ax.set_xlabel("x (cm, forward)")
    ax.set_ylabel("y (cm, left)")
    ax.set_title(
        "ZED ray -> board plane vs the wrist chain, placement left out of the ZED fit\n"
        "colour: RMS over the corners (mm); arrow: mean error x 1 cm per mm"
    )
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, top))
    fig.colorbar(sm, ax=ax, label="RMS (mm)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fit_cmd(dirs: list[Path], stack_mm: float | None) -> None:
    from so101_core.model import urdf_limits

    cal = Calib()
    intr, zi, ptr = wrist_intrinsics(), zed_left(), Pointer()
    folders = placements(dirs)
    if len(folders) < 3:
        raise SystemExit("need 3+ placements")
    folders = [
        fo for fo in folders if zed_views([cv2.imread(str(p)) for p in sorted(fo.glob("zed_before_*.png"))])
    ]
    wf, wsamples = wrist_boards(folders, cal, intr, ptr)
    names = [fo.name if fo.parent.name != "s4" else f"S4 {fo.name[-6:]}" for fo in folders]
    Ys = [wf.Ys[b] for b in range(len(folders))]
    zv = [zed_views([cv2.imread(str(p)) for p in sorted(fo.glob("zed_*.png"))]) for fo in folders]
    wviews = [[s for s in wsamples if s.board == b] for b in range(len(folders))]
    per_view = {id(s): e for s, e in zip(wsamples, wf.per_sample_px, strict=True)}
    Z, zrms = he.zed_fit([(v, Y) for vs, Y in zip(zv, Ys, strict=True) for v in vs], zi)
    _, loo = bu.zed_leave_one_out(zv, Ys, zi)
    inside = [bu.ray_plane_errors_mm(vs, Y, zi, Z) for vs, Y in zip(zv, Ys, strict=True)]
    mmpx = [bu.mm_per_px(Y, zi, Z) for Y in Ys]
    kin = he.board_mm_error(wf, wsamples, intr, ptr)

    # servos and kinematics at the pointing poses: the tool 1 cm above each board centre
    model, grav = cp.ServoModel.load(), st.Gravity()
    lo, hi = (np.array(x)[:5] for x in urdf_limits())
    servo_raw, servo_comp = [], []
    q0 = np.radians([0, -20, 40, 60, 0])
    for Y in Ys:
        q = ik_tool(ptr, bu.board_centre(Y) + [0, 0, ABOVE_M], q0, lo, hi)
        if q is not None:
            r, c = bu.servo_tip_mm(np.append(q, 0.0), model, grav, ptr)
            servo_raw.append(r)
            servo_comp.append(c)

    def rms(a) -> float:
        a = np.asarray(a)
        return float(np.sqrt((a**2).sum(-1).mean())) if a.ndim > 1 else float(np.sqrt((a**2).mean()))

    loo_all = np.vstack(loo)
    md = [
        f"# S5 error budget: {', '.join(names)}",
        "",
        f"{len(Ys)} placements of the board. The ZED (left eye) refitted to all of them: "
        f"{np.round(Z[:3, 3] * 100, 1).tolist()} cm, looking along {np.round(Z[:3, 2], 2).tolist()} "
        f"({zrms:.2f} px).",
        "",
        "Per placement: the ZED's ray through each corner meets the board's plane; the point is compared "
        "with the corner as the wrist chain (S4's camera and elbow offset) puts it. Left out: the ZED fitted "
        "without this "
        "placement.",
        "",
        "| Placement | Board centre (cm) | Wrist views | Wrist fit (px) | ZED corners | mm per ZED px "
        "| Left out: RMS / max (mm) | Mean error x, y (mm) | All in: RMS (mm) |",
        "|---|---|---:|---:|---:|---:|---|---|---:|",
    ]
    for i, name in enumerate(names):
        c = bu.board_centre(Ys[i]) * 100
        e = loo[i]
        md.append(
            f"| {name} | ({c[0]:.1f}, {c[1]:+.1f}) | {len(wviews[i])} | "
            f"{np.sqrt(np.mean([per_view[id(v)] ** 2 for v in wviews[i]])):.2f} | "
            f"{sum(len(v.ids) for v in zv[i])} | {mmpx[i]:.2f} | "
            f"{rms(e):.2f} / {np.linalg.norm(e, axis=1).max():.2f} "
            f"| {e.mean(0)[0]:+.2f}, {e.mean(0)[1]:+.2f} | {rms(inside[i]):.2f} |"
        )
    click = 1.0 * float(np.mean(mmpx))
    terms = [
        ("ZED pixel: a click 1 px off", "mean mm per px over the placements x 1 px", click),
        (
            "ZED pose (and the wrist chain's board)",
            "ZED ray-plane error, placement left out of the fit",
            rms(loo_all),
        ),
        (
            "Arm kinematics after the elbow offset",
            "board centre seen from each wrist pose vs the placement's fit",
            rms(kin),
        ),
        ("Servos, compensated (S1)", "0.15 deg per joint at the pointing poses", float(np.mean(servo_comp))),
    ]
    total = float(np.sqrt(sum(t[2] ** 2 for t in terms)))
    md += [
        "",
        "## Budget for pointing at a clicked ZED pixel (tool 1 cm above the board)",
        "",
        "| Term | How measured | RMS (mm) |",
        "|---|---|---:|",
    ]
    md += [f"| {a} | {b} | {c:.2f} |" for a, b, c in terms]
    md += [
        f"| **Total** (root sum of squares) | | **{total:.2f}** |",
        "",
        f"Servos without compensation (plain `move_to`): {np.mean(servo_raw):.1f} mm RMS at the same poses "
        "(S1b's sag and dead band).",
    ]
    out = {
        "source": [str(f) for f in folders],
        "T_base_zed_left": np.round(Z, 6).tolist(),
        "placements": {n: np.round(Y, 6).tolist() for n, Y in zip(names, Ys, strict=True)},
        "ipad_z_m": round(float(Ys[-1][2, 3]), 5),
        "loo_rms_mm": round(rms(loo_all), 3),
        "budget_mm": {a: round(c, 3) for a, _, c in terms} | {"total": round(total, 3)},
    }
    if stack_mm is not None:
        out["table_z_m"] = round(float(np.mean([Y[2, 3] for Y in Ys])) - stack_mm / 1000, 5)
        md.append(
            f"\nTable at {out['table_z_m'] * 1000:+.1f} mm in the base frame (board - {stack_mm:g} mm)."
        )
    RESULTS.mkdir(parents=True, exist_ok=True)
    stem = folders[-1].parent.name if folders[-1].name.startswith("p") else folders[-1].name
    heat = RESULTS / f"heatmap_{stem}.png"
    heat_map(names, Ys, loo, Z, heat)
    md += ["", f"![heat map]({heat.name})"]
    BUDGET.write_text(yaml.safe_dump(out, sort_keys=False))
    report = RESULTS / f"budget_{stem}.md"
    report.write_text("\n".join(md) + "\n")
    print("\n".join(md))
    print(f"Wrote {BUDGET}, {report}, {heat}")


# -- point (demo) --------------------------------------------------------------------------------------------
def plan_point(ptr, target, z, q_true, lo, hi, rest, extra, safety, coll, q_now):
    """Hover (6 cm) and point (1 cm) poses over target for the fixed jaw's tip: true angles (5,) and commands
    (6,) before compensation; or why not. The move there and the way down are collision-checked."""
    hover = ik_tool(ptr, target + [0, 0, HOVER_M], q_true, lo, hi, tool=bu.JAW_TIP)
    if hover is None:
        return "Out of reach"
    touch = ik_tool(ptr, target + [0, 0, ABOVE_M], hover, lo, hi, tool=bu.JAW_TIP)
    if touch is None:
        return "Out of reach"
    qa, qb = np.append(hover - extra, rest[5]), np.append(touch - extra, rest[5])
    step = POINT_SPEED / safety.cfg.rate_hz
    if bu.route_contacts(st.route([qa], q_now, safety, step), coll, ptr, z, extra, rest):
        return "The way there would come too close to the table or the arm"
    for q in st.ramp_path(qa, qb, step):
        qt = q.copy()
        qt[:5] += extra
        if coll.hits(qt) or bu.tool_clearance(qt, ptr, z) < ABOVE_M - 0.003:
            return "The way down would touch something"
    return hover, touch, qa, qb


def point(table: bool) -> None:
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap

    devices, jm, cal = DeviceConfig.load(), JointMap.load(), Calib()
    extra = cal.extra(jm)
    z = cal.table_z if table else cal.ipad_z
    if z is None:
        raise SystemExit("No table height: run `so101-budget fit ... --stack-mm 11` first.")
    zi, ptr = zed_left(), Pointer()
    model, grav = cp.ServoModel.load(), st.Gravity()
    if model.position_p != SERVO_P:
        raise SystemExit(f"servo model fitted at P = {model.position_p}, running at {SERVO_P}")
    arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
    lo, hi = arm.safety.lo[:5] + extra, arm.safety.hi[:5] + extra  # true-angle limits
    rest = arm.safety.clip(jm.rest)[0]
    coll = st.Collisions(clearance_m=0.005)
    print(
        f"Click a point in the ZED window ({'table' if table else 'iPad surface'} at {z * 1000:+.1f} mm, "
        f"ZED pose from {cal.zed_source}); the fixed jaw's tip goes "
        f"{ABOVE_M * 100:.0f} cm above it.\n"
        "In the window: click = pick, g = go, q = quit. Keep hands off the arm."
    )
    clicked: dict = {}

    def on_mouse(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN:
            clicked["px"] = (x, y)

    zed = Grabber("zed")
    win = "so101-budget: ZED left (click)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    try:
        if not ask_yes("The arm will be POWERED and move to the points you click."):
            return
        with arm.connect(torque=True):
            arm.move_to(rest, speed_rad_s=MOVE_SPEED)
            q_true = arm.read()[:5] + extra
            while True:
                disp = cam_left(zed.latest()).copy()
                target = None
                if "px" in clicked:
                    target = bu.ray_plane(np.array([clicked["px"]]), zi, cal.zed, z)[0]
                    cv2.drawMarker(disp, clicked["px"], (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
                    msg = f"target ({target[0] * 100:.1f}, {target[1] * 100:+.1f}) cm   g = go"
                else:
                    msg = "click a point"
                key = show(win, disp, None, msg)
                if key == ord("q"):
                    break
                if key != ord("g") or target is None:
                    continue
                plan = plan_point(ptr, target, z, q_true, lo, hi, rest, extra, arm.safety, coll, arm.read())
                if isinstance(plan, str):
                    print(plan + "; pick another point.")
                    continue
                hover, touch, qa, qb = plan
                # servo compensation (S1): command past the target by the predicted sag and dead band
                side = cp.motion_side(np.sign(touch - hover)[None])[0]
                comp = np.radians(model.error_deg(grav.tau(touch), side))
                cmd_h, cmd_t = qa.copy(), qb.copy()
                cmd_t[:5] += comp
                arm.move_to(cmd_h, speed_rad_s=POINT_SPEED)
                arm.move_to(cmd_t, speed_rad_s=POINT_SPEED / 2)
                time.sleep(0.8)
                q_now = np.mean([arm.read() for _ in range(10)], axis=0)
                q_now[:5] += extra
                tip = ptr.tip(q_now, bu.JAW_TIP)
                d = (tip - (target + [0, 0, ABOVE_M])) * 1000
                print(
                    f"At ({target[0] * 100:.1f}, {target[1] * 100:+.1f}) cm: the joints say the jaw "
                    f"tip is off by {np.round(d, 1).tolist()} mm (servo residual). Look: is it "
                    f"{ABOVE_M * 100:.0f} cm "
                    "above the "
                    "point you clicked? Press any key in the window to go back up."
                )
                while show(win, cam_left(zed.latest()), None, "any key = back up") == 255:
                    pass
                arm.move_to(cmd_h, speed_rad_s=POINT_SPEED)
                q_true = hover
                clicked.clear()
            arm.move_to(rest, speed_rad_s=MOVE_SPEED)
            print("Support the arm: torque goes off next.")
    finally:
        zed.close()
        cv2.destroyAllWindows()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["plan", "run", "fit", "point"])
    ap.add_argument("dirs", nargs="*", type=Path)
    ap.add_argument("--stack-mm", type=float, help="iPad + case thickness (mm): the table height")
    ap.add_argument("--table", action="store_true", help="point: aim at the table, not the iPad surface")
    a = ap.parse_args()
    if a.stage == "plan":
        plan_cmd()
    elif a.stage == "run":
        run()
    elif a.stage == "fit":
        if not a.dirs:
            raise SystemExit("so101-budget fit captures/s5/<time> [captures/s4/<time> ..]")
        fit_cmd(a.dirs, a.stack_mm)
    else:
        point(a.table)


if __name__ == "__main__":
    main()
