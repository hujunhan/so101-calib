"""so101-handeye: S4, the wrist camera on the gripper, the ZED in the base frame, and the joint zero offsets.
The iPad board (tools/ipad_charuco.html) lies still on the table in front of the arm the whole time.

    so101-handeye zed-view          live ZED view, no arm: place the ZED so it sees the board (and the arm)
    so101-handeye run               the ZED sees the board; you pose the LIMP arm once so the wrist camera
                                    sees it (space); then torque on (P = SERVO_P) and the arm moves itself
                                    through ~22 poses around it (SafeArm, slow, collision-checked with the
                                    iPad), taking a wrist image at each -> captures/s4/<time>/
    so101-handeye fit DIR [DIR..]   hand-eye of all runs together (one board placement per run), joint offset
                                    models compared on held-out views and on left-out runs, the ZED in the
                                    base frame -> results/s4/handeye.yaml and a report. --stack-mm: the
                                    iPad + case thickness, to check the table height against the URDF

Images stay in the git-ignored captures/; results/ gets numbers only.
"""

import argparse
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from so101_calib import SERVO_P, cam_cli
from so101_calib import charuco as ch
from so101_calib import handeye as he
from so101_calib import statics as st
from so101_calib.touch import Pointer

CAPTURES = Path("captures/s4")
RESULTS = Path("results/s4")
WRIST_YAML = Path("results/s3/wrist_intrinsics.yaml")
MOVE_SPEED = np.radians(12.0)
SETTLE_S = 0.4
IPAD_SIZE_M = (0.25, 0.18, 0.008)  # iPad Pro 11" in its case, roughly; for the collision check only
# Stage A: small steps around the taught pose (deg; pan, lift, elbow, wrist_flex, roll) that keep the board in
# view without knowing where the camera is; they start the fit.
STAGE_A = [
    (0, 0, 0, 0, 0),
    (0, 0, 0, 0, 35),
    (0, 0, 0, 0, -35),
    (0, 0, 0, 10, 0),
    (0, 0, 0, -10, 0),
    (6, 0, 0, 0, 20),
    (-6, 0, 0, 0, -20),
    (0, 5, -8, 3, 0),
]
N_STAGE_B = 20
SPAN_B_DEG = np.array([25, 15, 20, 30, 60])  # stage B samples around the taught pose within these


def wrist_intrinsics() -> ch.Intrinsics:
    return ch.Intrinsics.from_dict(yaml.safe_load(WRIST_YAML.read_text())["wrist"])


def zed_left() -> ch.Intrinsics:
    return ch.read_zed_conf(cam_cli.FACTORY, "HD")["left"]


class Grabber:
    """Reads a camera continuously in the background (so a frame taken after the arm settled is fresh)."""

    def __init__(self, name: str):
        self.cap = cam_cli.open_camera(name)
        self.frame, self.t = None, 0.0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self) -> None:
        while not self.stop.is_set():
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame, self.t = f, time.perf_counter()

    def latest(self, after: float = 0.0, timeout: float = 2.0) -> np.ndarray | None:
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < timeout:
            with self.lock:
                if self.frame is not None and self.t > after:
                    return self.frame.copy()
            time.sleep(0.005)
        return None

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=1.0)
        self.cap.release()


def show(win: str, img: np.ndarray, view: ch.View | None, text: str) -> int:
    disp = img.copy()
    if view is not None:
        for x, y in view.img:
            cv2.circle(disp, (int(x), int(y)), 4, (0, 200, 255), -1)
    for color, width in (((0, 0, 0), 4), ((255, 255, 255), 1)):
        cv2.putText(disp, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, width)
    cv2.imshow(win, disp)
    return cv2.waitKey(1) & 0xFF


def zed_board(frames: list[np.ndarray]) -> tuple[np.ndarray | None, int]:
    """The board in the ZED left eye's frame (PnP with the factory intrinsics), averaged over frames."""
    det, intr, Ts = ch.detector(), zed_left(), []
    for f in frames:
        v = ch.detect(cv2.cvtColor(cam_cli.halves(f, "zed")[0], cv2.COLOR_BGR2GRAY), det)
        if v is not None and len(v.ids) >= 16:
            Ts.append(he.pnp(v, intr))
    return (he.average_pose(Ts) if Ts else None), len(Ts)


def zed_views(frames: list[np.ndarray]) -> list[ch.View]:
    det = ch.detector()
    out = []
    for f in frames:
        v = ch.detect(cv2.cvtColor(cam_cli.halves(f, "zed")[0], cv2.COLOR_BGR2GRAY), det)
        if v is not None and len(v.ids) >= 16:
            out.append(v)
    return out


def zed_view() -> None:
    g = Grabber("zed")
    det, intr = ch.detector(), zed_left()
    print("Place the ZED: it must see the iPad board lying in front of the arm, and the arm. q / Esc: done.")
    while True:
        f = g.latest()
        left = cam_cli.halves(f, "zed")[0]
        v = ch.detect(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), det)
        text = "no board"
        if v is not None:
            T = he.pnp(v, intr)
            text = f"board: {len(v.ids)} corners, {np.linalg.norm(T[:3, 3]) * 100:.0f} cm away"
        if show("so101-handeye: ZED left", left, v, text) in (ord("q"), 27):
            break
    g.close()
    cv2.destroyAllWindows()


# -- the run -------------------------------------------------------------------------------------------------
def ask_yes(msg: str) -> bool:
    return input(f"\n>>> {msg} Type 'yes': ").strip().lower() == "yes"


def teach(arm, wrist: Grabber, intr) -> np.ndarray | None:
    """Limp arm: the user poses it so the wrist camera sees the whole board, then presses space."""
    det = ch.detector()
    print(
        "Move the LIMP arm so the wrist camera sees the whole board from ~25 cm, gripper pointing down at it."
    )
    print("Press SPACE in the window when it does (keep holding the arm afterwards), q to stop.")
    while True:
        f = wrist.latest()
        v = ch.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), det)
        n = 0 if v is None else len(v.ids)
        key = show("so101-handeye: wrist", f, v, f"board corners {n}/28   SPACE = this pose, q = stop")
        if key == ord("q"):
            return None
        if key == ord(" ") and n >= 20:
            return np.mean([arm.read() for _ in range(10)], axis=0)


def capture_at(arm, q, wrist: Grabber, out: Path, k: int, det) -> dict | None:
    arm.move_to(q, speed_rad_s=MOVE_SPEED)
    time.sleep(SETTLE_S)
    t = time.perf_counter()
    q_meas = np.mean([arm.read() for _ in range(10)], axis=0)
    f = wrist.latest(after=t)
    if f is None:
        return None
    cv2.imwrite(str(out / f"wrist_{k:03d}.png"), f)
    v = ch.detect(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), det)
    show("so101-handeye: wrist", f, v, f"pose {k}: {0 if v is None else len(v.ids)} corners")
    return {"k": k, "q_cmd": q, "q": q_meas, "corners": 0 if v is None else len(v.ids)}


def clearance(q: np.ndarray, ptr: Pointer, board_z: float) -> float:
    """Height (m) of the gripper's lowest point we track (jaw tips, wrist) above the board's surface."""
    G = ptr.gripper(q[:5])
    tips = [G[:3, :3] @ p + G[:3, 3] for p in ([0.0, 0.0, -0.105], [-0.025, 0.0, -0.10], [0.0, 0.0, 0.0])]
    return min(float(t[2]) for t in tips) - board_z


def plan_stage_b(q0, X, Y, intr, ptr, safety, rng) -> tuple[list[np.ndarray], list[str]]:
    """Poses around the taught one where the fitted camera should see the board well, spread out, in an
    order that keeps moves short; every move collision-checked with the iPad on the table."""
    board_z = float(board_centre(Y)[2])
    # the iPad: centred under the board, its top face at the board, lying on the table
    centre = board_centre(Y) - np.array([0.0, 0.0, IPAD_SIZE_M[2] / 2])
    coll = st.Collisions(world_boxes=[(centre, Y[:3, :3], IPAD_SIZE_M)])
    cands = []
    for _ in range(4000):
        q = q0.copy()
        q[:5] += np.radians(rng.uniform(-SPAN_B_DEG, SPAN_B_DEG))
        clipped, _ = safety.clip(q)
        if not np.allclose(clipped[:5], q[:5]):  # arm joints inside the safe range (the gripper stays)
            continue
        frac, angle, dist = he.predicted_view(q, X, Y, intr, ptr)
        if frac < 0.85 or angle > 50 or not 0.12 < dist < 0.45:
            continue
        if clearance(q, ptr, board_z) < 0.03 or coll.hits(q):
            continue
        cands.append(q)
    if len(cands) < N_STAGE_B:
        return [], [f"only {len(cands)} candidate poses see the board safely"]
    order = he.spread(cands, X, ptr, N_STAGE_B, q0)
    poses = [cands[i] for i in order]
    problems = []
    step = MOVE_SPEED / safety.cfg.rate_hz
    path = st.route(poses + [q0], q0, safety, step)
    for i, q in enumerate(path):
        if coll.hits(q) or clearance(q, ptr, board_z) < 0.02:
            problems.append(f"move {i}: contact at {np.round(np.degrees(q[:5])).tolist()} deg")
    return poses, problems


def run() -> None:
    from so101_core.arm import SafeArm
    from so101_core.config import DeviceConfig, JointMap

    devices, jm = DeviceConfig.load(), JointMap.load()
    intr, ptr = wrist_intrinsics(), Pointer()
    out = CAPTURES / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    meta: dict = {"date": datetime.now().isoformat(timespec="seconds"), "position_p": SERVO_P, "samples": []}
    print(
        "The iPad (board full screen, brightness ~50 %) lies flat on the table about 25 cm in front of the\n"
        "arm, and must not move until the end. The arm is at rest."
    )
    wrist, zed = Grabber("wrist"), Grabber("zed")
    try:
        zed_frames = [zed.latest(after=time.perf_counter()) for _ in range(10)]
        Tb, n = zed_board(zed_frames)
        if Tb is None:
            raise SystemExit("The ZED does not see the board; place it with `so101-handeye zed-view`.")
        for i, f in enumerate(zed_frames[:3]):
            cv2.imwrite(str(out / f"zed_before_{i}.png"), f)
        print(f"ZED sees the board ({n} frames), {np.linalg.norm(Tb[:3, 3]) * 100:.0f} cm away.")

        arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
        with arm.connect(torque=False):
            q0 = teach(arm, wrist, intr)
        if q0 is None:
            return
        print("Keep holding the arm: torque goes on now.")
        arm = SafeArm(devices, joint_map=jm, position_p=SERVO_P)
        det = ch.detector()
        with arm.connect(torque=True):
            print("Holding. You can let go.")
            if not ask_yes("The arm will MOVE (slowly) through ~24 poses above the iPad. Stay next to it."):
                return
            samples, k = [], 0
            for d in STAGE_A:
                q = q0.copy()
                q[:5] += np.radians(d)
                s = capture_at(arm, arm.safety.clip(q)[0], wrist, out, k, det)
                k += 1
                if s:
                    samples.append(s)
            good = [s for s in samples if s["corners"] >= 12]
            print(f"Stage A: {len(good)}/{len(STAGE_A)} views see the board.")
            views = load_samples(out, good, intr)
            if len(views) < 5:
                raise SystemExit(
                    "Too few views to start the fit; teach a pose where the whole board is seen."
                )
            first = he.fit(views, intr, ptr)
            print(
                f"First fit: {first.rms_px:.2f} px; camera {np.round(first.X[:3, 3] * 100, 1).tolist()} cm "
                "in the gripper frame. Planning stage B."
            )
            poses, problems = plan_stage_b(
                q0, first.X, first.Y, intr, ptr, arm.safety, np.random.default_rng(0)
            )
            if problems:
                print("Stage B skipped: " + "; ".join(problems[:3]))
            for q in poses:
                s = capture_at(arm, q, wrist, out, k, det)
                k += 1
                if s:
                    samples.append(s)
                meta["samples"] = [
                    {kk: (v.tolist() if isinstance(v, np.ndarray) else v) for kk, v in x.items()}
                    for x in samples
                ]
                (out / "samples.json").write_text(json.dumps(meta, indent=1))
            arm.move_to(q0, speed_rad_s=MOVE_SPEED)
            print(
                f"{sum(s['corners'] >= 12 for s in samples)} views with the board. Back at the taught pose."
            )
            print("Now support the arm: torque goes off next, then put it back to rest by hand.")
        input("\n>>> Arm at rest and out of the ZED's view of the iPad? Press Enter. ")
        zed_frames = [zed.latest(after=time.perf_counter()) for _ in range(10)]
        for i, f in enumerate(zed_frames[:3]):
            cv2.imwrite(str(out / f"zed_after_{i}.png"), f)
        Ta, _ = zed_board(zed_frames)
        if Ta is not None:
            moved = np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]) * 1000
            print(f"ZED: the board moved {moved:.1f} mm between before and after (should be ~0).")
    finally:
        wrist.close()
        zed.close()
        cv2.destroyAllWindows()
    print(f"\nSaved {out}. Fit with: uv run so101-handeye fit {out}")


# -- the fit -------------------------------------------------------------------------------------------------
def load_samples(folder: Path, samples: list[dict], intr, board: int = 0) -> list[he.Sample]:
    det, out = ch.detector(), []
    for s in samples:
        img = cv2.imread(str(folder / f"wrist_{s['k']:03d}.png"))
        if img is None:
            continue
        v = ch.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), det)
        if v is not None and len(v.ids) >= 12:
            out.append(he.Sample(np.asarray(s["q"], float), v, board))
    return out


def board_centre(Y: np.ndarray) -> np.ndarray:
    return Y[:3, :3] @ ch.board().getChessboardCorners().mean(0) + Y[:3, 3]


def base_bottom_z() -> float:
    """Height (m) of the base's lowest point in the base frame (the URDF mesh): where the table is."""
    import mujoco
    from so101_core.model import MJCF_PATH

    m = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    zs = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            k = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[k] : m.mesh_vertadr[k] + m.mesh_vertnum[k]]
            zs.append(((d.geom_xmat[g].reshape(3, 3) @ v.T).T + d.geom_xpos[g])[:, 2].min())
    return float(min(zs))


def choose(held: dict[str, float]) -> str:
    """The simplest model within 5 % of the best held-out error; offsets only if they beat no offsets by
    20 % or more (models are ordered simple to complex)."""
    best = min(held.values())
    for name in he.MODELS:
        if held[name] <= 1.05 * best:
            pick = name
            break
    return pick if held[pick] < 0.8 * held["none"] else "none"


def fit_cmd(folders: list[Path], stack_mm: float | None = None) -> None:
    intr, ptr = wrist_intrinsics(), Pointer()
    samples = []
    for b, folder in enumerate(folders):
        meta = json.loads((folder / "samples.json").read_text())
        samples += load_samples(folder, meta["samples"], intr, b)
    fits = {m: he.fit(samples, intr, ptr, j) for m, j in he.MODELS.items()}
    held = {m: he.heldout_px(samples, intr, ptr, j) for m, j in he.MODELS.items()}
    runs = {m: he.other_run_px(samples, intr, ptr, j) for m, j in he.MODELS.items()}
    mm = {m: he.board_mm_error(fits[m], samples, intr, ptr) for m in he.MODELS}
    crit = runs if len(folders) > 1 else held
    pick = choose(crit)
    names = ", ".join(f.name for f in folders)
    n_runs = [sum(s.board == b for s in samples) for b in range(len(folders))]
    md = [
        f"# S4 hand-eye: {names}",
        "",
        f"{len(samples)} wrist views ({' + '.join(map(str, n_runs))}), one board placement per run.",
        "",
        "| Offsets fitted | Fit RMS (px) | Held-out views (px) | Left-out run (px) "
        "| Board centre, camera vs fit: median / max (mm) | Offsets lift / elbow / wrist (deg) |",
        "|---|---:|---:|---:|---|---|",
    ]
    for m, j in he.MODELS.items():
        f = fits[m]
        offs = " / ".join(
            f"{f.offsets_deg[k]:+.2f} +- {f.offsets_std_deg[k]:.2f}" if k in j else "-" for k in (1, 2, 3)
        )
        r = f"{runs[m]:.2f}" if runs[m] is not None else "-"
        md.append(
            f"| {m} | {f.rms_px:.2f} | {held[m]:.2f} | {r} | "
            f"{np.median(mm[m]):.1f} / {mm[m].max():.1f} | {offs} |"
        )
    f = fits[pick]
    rv, t = he.rvec_t(f.X)
    md += [
        "",
        f"Kept: **{pick}** (the simplest model within 5 % of the best "
        f"{'left-out-run' if len(folders) > 1 else 'held-out'} error, and offsets only if they cut it "
        "by 20 % or more).",
        f"Wrist camera in the gripper frame: t = {np.round(t * 1000, 1).tolist()} mm, rotation vector "
        f"{np.round(np.degrees(rv), 2).tolist()} deg.",
    ]
    out = {
        "source": [fo.name for fo in folders],
        "offset_model": pick,
        "joint_offsets_deg": {
            j: round(float(f.offsets_deg[i]), 3)
            for i, j in enumerate(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"))
        },
        "T_gripper_wristcam": np.round(f.X, 6).tolist(),
        "T_base_board": {},
    }
    floor = base_bottom_z() if stack_mm is not None else None
    zed_seen: list = []
    for b, folder in enumerate(folders):
        Y = f.Ys[b]
        out["T_base_board"][folder.name] = np.round(Y, 6).tolist()
        centre = np.round(board_centre(Y) * 100, 1).tolist()
        line = f"Run {folder.name}: board centre {centre} cm in the base frame"
        if floor is not None:
            table = Y[2, 3] - stack_mm / 1000
            line += (
                f"; table at {table * 1000:+.1f} mm (board - {stack_mm:g} mm iPad and case), the URDF's base "
                f"bottom at {floor * 1000:+.1f} mm"
            )
        zb, _ = zed_board([cv2.imread(str(p)) for p in sorted(folder.glob("zed_before_*.png"))])
        za, _ = zed_board([cv2.imread(str(p)) for p in sorted(folder.glob("zed_after_*.png"))])
        if zb is not None:
            Z = he.zed_in_base(Y, zb)
            zed_seen += [(v, Y) for v in zed_views([cv2.imread(str(p)) for p in folder.glob("zed_*.png")])]
            line += f"; the ZED from this run alone at {np.round(Z[:3, 3] * 100, 1).tolist()} cm"
            if za is not None:
                line += f"; board moved {np.linalg.norm(za[:3, 3] - zb[:3, 3]) * 1000:.1f} mm during the run"
        md.append(line + ".")
    if zed_seen:
        # assumes the ZED stood still through all runs
        Z, zrms = he.zed_fit(zed_seen, zed_left())
        out["T_base_zed_left"] = np.round(Z, 6).tolist()
        md.append(
            f"ZED left eye, from all runs' board placements together: {np.round(Z[:3, 3] * 100, 1).tolist()} "
            f"cm in the base frame, looking along {np.round(Z[:3, 2], 2).tolist()} ({zrms:.2f} px)."
        )
    if floor is not None:
        out["table_z_m"] = round(
            float(np.mean([f.Ys[b][2, 3] for b in range(len(folders))]) - stack_mm / 1000), 5
        )
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "handeye.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
    suffix = f"_x{len(folders)}" if len(folders) > 1 else ""
    report = RESULTS / f"handeye_{folders[-1].name}{suffix}.md"
    report.write_text("\n".join(md) + "\n")
    print("\n".join(md))
    print(f"Wrote {RESULTS / 'handeye.yaml'} and {report}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["zed-view", "run", "fit"])
    ap.add_argument("dirs", nargs="*", type=Path)
    ap.add_argument("--stack-mm", type=float, help="iPad + case thickness (mm), for the table height check")
    a = ap.parse_args()
    if a.stage == "zed-view":
        zed_view()
    elif a.stage == "run":
        run()
    else:
        if not a.dirs:
            raise SystemExit("so101-handeye fit captures/s4/<time> [captures/s4/<time> ..]")
        fit_cmd(a.dirs, a.stack_mm)


if __name__ == "__main__":
    main()
