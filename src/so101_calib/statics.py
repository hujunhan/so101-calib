"""S1b: the servos' static error over the workspace.

Model (from S1a): a position servo with gain P is a torsion spring. At rest, holding a gravity torque tau,

    error = commanded - measured = c * tau + s * (b + k * |tau|) + o

c is the compliance (deg per N m, set by P), s = +1 when the joint arrived from above the target and -1 from
below, b + k |tau| is the half-width of the band the servo can stop anywhere in (friction against P x error;
the S1b data show it grows with the load the gears carry, hence k), and o a constant offset (expected ~0).
tau comes from the URDF masses (Pinocchio), plus an optional payload at the tool.
"""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from so101_core.model import ARM_JOINTS
from so101_core.safety import SafetyLayer
from so101_kinematics.models import PinArm

from so101_calib.servo_id import ramp_path

APPROACH_DEG = 3.0  # each approach starts this far above or below the target, on every arm joint
APPROACH_SPEED = np.radians(10.0)
HOLD_S = 1.0  # read the arm this long after it settles; the mean of the last half is the measurement
REPEATS = 2  # each pose: (+, -) approaches, this many times
N_POSES = 16
MIN_TOOL_Z = 0.08  # m above the base-frame origin (the base's bottom face)
MIN_TOOL_X = 0.12  # m in front of the base
# A payload box taped flat against the outside of the fixed jaw, long side along the gripper, its end flush
# with the jaw tip. Frame: the gripper body (MuJoCo "gripper" = URDF wrist_roll child link; the gripper points
# along -z, the fixed jaw's outer face is at x = -2.5 cm, the jaw tip at z = -10.4 cm; measured on the mesh).
JAW_OUTER_X = -0.025
JAW_TIP_Z = -0.104


def payload_center(box_m: tuple[float, float, float]) -> np.ndarray:
    """Centre (m, gripper frame) of a box of (length, width, thickness) mounted as above."""
    length, _, thickness = box_m
    return np.array([JAW_OUTER_X - thickness / 2, 0.0, JAW_TIP_Z + length / 2])


GRAVITY_JOINTS = (1, 2, 3)  # shoulder_lift, elbow_flex, wrist_flex; pan and roll carry no gravity torque


class Gravity:
    """Joint torques (N m) that hold the arm still, from the URDF masses, plus a payload of `payload_kg` whose
    centre of mass is at `payload_pos` (m, gripper frame; default: the tool point)."""

    def __init__(self, payload_kg: float = 0.0, payload_pos: np.ndarray | None = None):
        self.arm = PinArm()
        self.payload_kg = payload_kg
        m = self.arm.model
        tool = m.frames[self.arm.fid]
        # copy: addFrame below may move the frames, and the placement is a view into them
        pos = (
            tool.placement.translation.copy() if payload_pos is None else np.asarray(payload_pos, dtype=float)
        )
        self.payload_pos = pos
        self.pid = m.addFrame(
            pin.Frame(
                "payload", tool.parentJoint, self.arm.fid, pin.SE3(np.eye(3), pos), pin.FrameType.OP_FRAME
            )
        )
        self.arm.data = m.createData()

    def frames_z(self, q: np.ndarray) -> np.ndarray:
        """Heights (m) of every body frame (joint origins and the tool)."""
        m, d = self.arm.model, self.arm.data
        pin.framesForwardKinematics(m, d, self.arm._full(q[:5]))
        return np.array(
            [d.oMf[i].translation[2] for i, f in enumerate(m.frames) if f.type == pin.FrameType.BODY]
        )

    def tau(self, q: np.ndarray) -> np.ndarray:
        """(5,) holding torques of the arm joints for joint angles q (rad, 5 or 6 values)."""
        a = self.arm
        g = pin.computeGeneralizedGravity(a.model, a.data, a._full(q[:5]))[a.idx]
        if self.payload_kg:
            # A rigid payload: the joints hold J^T times its weight (pointing down) at its centre of mass.
            full = a._full(q[:5])
            pin.computeJointJacobians(a.model, a.data, full)
            pin.framesForwardKinematics(a.model, a.data, full)
            J = pin.getFrameJacobian(a.model, a.data, self.pid, pin.LOCAL_WORLD_ALIGNED)[:3, a.idx]
            weight = np.array([0.0, 0.0, -self.payload_kg * 9.81])
            g = g - J.T @ weight
        return g


@dataclass
class Candidate:
    q: np.ndarray  # (5,) rad
    tau: np.ndarray  # (5,) N m


def candidate_poses(safety: SafetyLayer, grav: Gravity) -> list[Candidate]:
    """A grid over lift, elbow and wrist (pan = roll = 0) inside the safe range, tool in front and above the
    table, with room for the approach offsets."""
    out = []
    margin = np.radians(APPROACH_DEG)
    for lift in range(-80, 31, 10):
        for elbow in range(-40, 91, 10):
            for wrist in range(-60, 91, 15):
                q = np.radians([0.0, lift, elbow, wrist, 0.0])
                if np.any(q - margin < safety.lo[:5]) or np.any(q + margin > safety.hi[:5]):
                    continue
                tool = grav.arm.fk(q)[:3, 3]
                if tool[2] < MIN_TOOL_Z or tool[0] < MIN_TOOL_X or grav.frames_z(q).min() < -1e-3:
                    continue
                out.append(Candidate(q, grav.tau(q)))
    return out


def spread_poses(cands: list[Candidate], start: np.ndarray, n: int) -> list[Candidate]:
    """Pick n poses that spread the gravity torques of lift, elbow and wrist (farthest-point sampling,
    torques scaled by their range), starting from the candidate nearest to `start`."""
    taus = np.array([c.tau[list(GRAVITY_JOINTS)] for c in cands])
    scaled = (taus - taus.min(0)) / np.ptp(taus, axis=0)
    first = int(np.argmin([np.abs(c.q - start[:5]).max() for c in cands]))
    chosen = [first]
    dist = np.linalg.norm(scaled - scaled[first], axis=1)
    while len(chosen) < n:
        nxt = int(np.argmax(dist))
        chosen.append(nxt)
        dist = np.minimum(dist, np.linalg.norm(scaled - scaled[nxt], axis=1))
    # Visit them in a short order (greedy nearest neighbour in joint space, from the first).
    order = [chosen[0]]
    left = set(chosen[1:])
    while left:
        here = cands[order[-1]].q
        nxt = min(left, key=lambda i: np.abs(cands[i].q - here).max())
        order.append(nxt)
        left.remove(nxt)
    return [cands[i] for i in order]


def approach_targets(q: np.ndarray, safety: SafetyLayer) -> list[tuple[int, np.ndarray]]:
    """(side, start) pairs for one pose: start APPROACH_DEG above (side +1) or below (-1), every arm joint."""
    out = []
    for _ in range(REPEATS):
        for side in (1, -1):
            start = q.copy()
            start[:5] += side * np.radians(APPROACH_DEG)
            out.append((side, safety.clip(start)[0]))
    return out


class Collisions:
    """Contacts between the arm's collision meshes, and with the floor plane at the base's bottom face, in
    so101-core's MuJoCo model. The folded rest pose itself overlaps in the model (the meshes are coarse and
    the real arm rests on the table), so routes are checked once they have left the rest pose."""

    def __init__(self, clearance_m: float = 0.005, payload_box_m: tuple[float, float, float] | None = None):
        import mujoco
        from so101_core.model import JOINTS, SCENE_PATH

        spec = mujoco.MjSpec.from_file(str(SCENE_PATH))
        if payload_box_m is not None:
            length, width, thickness = payload_box_m
            geom = spec.body("gripper").add_geom()
            geom.name = "payload"
            geom.type = mujoco.mjtGeom.mjGEOM_BOX
            geom.size = [thickness / 2, width / 2, length / 2]
            geom.pos = payload_center(payload_box_m)
            geom.contype, geom.conaffinity = 1, 1
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.qadr = np.array([self.model.joint(j).qposadr[0] for j in JOINTS])
        self.clearance = clearance_m
        self._mujoco = mujoco

    def hits(self, q: np.ndarray) -> list[tuple[str, str]]:
        m, d = self.model, self.data
        d.qpos[self.qadr] = q
        self._mujoco.mj_forward(m, d)
        out = []
        for c in d.contact[: d.ncon]:
            if c.dist < self.clearance:
                out.append(tuple(m.geom(g).name or m.body(m.geom_bodyid[g]).name for g in (c.geom1, c.geom2)))
        return out


def route(poses: list[np.ndarray], q_from: np.ndarray, safety: SafetyLayer, step: float) -> np.ndarray:
    """The `move_to` commands that visit `poses` in order from q_from (every `step` rad per joint)."""
    out = [safety.clip(q_from)[0][None]]
    q = out[0][0]
    for p in poses:
        out.append(ramp_path(q, p, step))
        q = p
    return np.vstack(out)


def route_problems(
    path: np.ndarray, coll: "Collisions", rest: np.ndarray, near_rest_rad: float = 0.35
) -> list[str]:
    """Contacts on the route, except next to the rest pose (which is in contact in the model): contacts are
    allowed only while every arm joint is within `near_rest_rad` of rest."""
    problems = []
    for i, q in enumerate(path):
        hits = coll.hits(q)
        if hits and np.abs(q[:5] - rest[:5]).max() > near_rest_rad:
            problems.append(f"command {i}: {sorted(set(hits))} at {np.round(np.degrees(q[:5])).tolist()} deg")
    return problems


# -- analysis ------------------------------------------------------------------------------------------------
def predict(f: dict, side: np.ndarray, tau: np.ndarray | None) -> np.ndarray:
    """The fitted model's error (deg) for approaches from `side` at holding torques `tau`."""
    side = np.asarray(side, dtype=float)
    out = f["b"] * side + f["o"]
    if f["c"] is not None:
        out = out + f["c"] * tau + f["k"] * side * np.abs(tau)
    return out


def fit_joint(err_deg: np.ndarray, side: np.ndarray, tau: np.ndarray | None) -> dict:
    """Least squares err = c * tau + side * (b + k * |tau|) + o; without tau (pan, roll): err = b * side + o.
    Also the fit with a constant band (k = 0), to show what the load-dependent term adds."""
    one = np.ones_like(side)
    out: dict = {"rms_before_deg": float(np.sqrt(np.mean(err_deg**2)))}
    if tau is None:
        (b, o), *_ = np.linalg.lstsq(np.column_stack([side, one]), err_deg, rcond=None)
        out |= {"c": None, "k": None, "b": float(b), "o": float(o)}
    else:
        m0 = np.column_stack([tau, side, one])
        coef0, *_ = np.linalg.lstsq(m0, err_deg, rcond=None)
        out["rms_resid_const_band_deg"] = float(np.sqrt(np.mean((err_deg - m0 @ coef0) ** 2)))
        m = np.column_stack([tau, side, side * np.abs(tau), one])
        (c, b, k, o), *_ = np.linalg.lstsq(m, err_deg, rcond=None)
        out |= {"c": float(c), "b": float(b), "k": float(k), "o": float(o)}
    resid = err_deg - predict(out, side, tau)
    out |= {"rms_resid_deg": float(np.sqrt(np.mean(resid**2))), "max_resid_deg": float(np.abs(resid).max())}
    return out


def joint_names() -> tuple[str, ...]:
    return ARM_JOINTS


def load_run(path) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """An S1b run: error (deg, command - measured, (n, 5)), holding torques (N m, (n, 5)), side, meta."""
    import json

    z = np.load(path)
    meta = json.loads(str(z["meta"]))
    grav = Gravity(meta["payload_g"] / 1000, meta.get("payload_pos_m"))
    err = np.degrees(z["q_cmd"] - z["q_meas"])[:, :5]
    tau = np.array([grav.tau(q) for q in z["q_cmd"]])
    return err, tau, z["side"].astype(float), meta
