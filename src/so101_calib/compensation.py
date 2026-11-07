"""S1c: feed-forward compensation from the S1 servo model, and how well the arm then tracks a path.

The S1b model says where a joint stops for a command:

    measured = command - (c * tau + side * (b + k * |tau|) + o)

so commanding `target + c * tau + side * (b + k * |tau|) + o` should put the joint on the target. On a moving
path, side follows the direction of motion (+1 moving down, -1 moving up: S1b's "approached from above" is a
joint moving down), and the servo answers about 80 ms late (S1a), so the command can also lead the target.
The parts are switched on one at a time to see what each one buys:

    none    command = target (M1)
    spring  + c * tau + o                    gravity sag, from the URDF torques
    band    + side * (b + k * |tau|)         the rest band, by direction of motion
    full    the above, on the target `lead` seconds ahead
"""

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from so101_core.model import ARM_JOINTS

from so101_calib import statics as st

VARIANTS = {"none": (), "spring": ("spring",), "band": ("spring", "band"), "full": ("spring", "band", "lead")}
# S1a at P = 32: steps reach half-way 56-67 ms after the command; the command is held for a tick, which adds
# half a tick (17 ms) against a smooth path. The sine delays (78-114 ms) include the band, compensated apart.
LEAD_S = 0.080
SIDE_SPEED = np.radians(0.5)  # rad/s: side = -tanh(v / this), so it turns over smoothly at a reversal
MODEL_PATH = Path("results/s1/servo_model.json")


@dataclass
class ServoModel:
    """Per arm joint (deg and deg per N m): error = c * tau + side * (b + k * |tau|) + o."""

    c: list[float]
    b: list[float]
    k: list[float]
    o: list[float]
    runs: list[str]
    position_p: int

    def error_deg(self, tau: np.ndarray, side: np.ndarray, parts=("spring", "band")) -> np.ndarray:
        """(..., 5) predicted command - measured, deg."""
        c, b, k, o = (np.array(v) for v in (self.c, self.b, self.k, self.o))
        out = np.zeros(np.broadcast_shapes(np.shape(tau), np.shape(side)))
        if "spring" in parts:
            out = out + c * tau + o
        if "band" in parts:
            out = out + side * (b + k * np.abs(tau))
        return out

    @classmethod
    def fit(cls, paths: list[Path]) -> "ServoModel":
        """Least squares on the S1b runs together (pan and roll: c = k = 0)."""
        runs = [st.load_run(p) for p in paths]
        ps = {r[3]["position_p"] for r in runs}
        if len(ps) != 1:
            raise ValueError(f"runs at different gains: {ps}")
        err = np.vstack([r[0] for r in runs])
        tau = np.vstack([r[1] for r in runs])
        side = np.concatenate([r[2] for r in runs])
        out = {"c": [], "b": [], "k": [], "o": []}
        for j in range(len(ARM_JOINTS)):
            f = st.fit_joint(err[:, j], side, tau[:, j] if j in st.GRAVITY_JOINTS else None)
            for key in out:
                out[key].append(0.0 if f[key] is None else round(f[key], 4))
        return cls(**out, runs=[Path(p).name for p in paths], position_p=ps.pop())

    def save(self, path: Path = MODEL_PATH) -> None:
        path.write_text(json.dumps({"joints": list(ARM_JOINTS), **asdict(self)}, indent=1) + "\n")

    @classmethod
    def load(cls, path: Path = MODEL_PATH) -> "ServoModel":
        d = json.loads(Path(path).read_text())
        d.pop("joints")
        return cls(**d)


def motion_side(v: np.ndarray) -> np.ndarray:
    """S1b's side from joint velocities (n, 5): +1 moving down, -1 up; at rest, the side it arrived from."""
    side = -np.tanh(v / SIDE_SPEED)
    moving = np.abs(v) > 1e-9
    for j in range(side.shape[1]):
        last = 0.0
        for i in range(len(side)):
            if moving[i, j]:
                last = side[i, j]
            else:
                side[i, j] = last
    return side


def compensate(
    target: np.ndarray,
    hz: float,
    model: ServoModel,
    grav: st.Gravity,
    parts: tuple[str, ...],
    lead_s: float = LEAD_S,
) -> np.ndarray:
    """Commands (n, 6) that should make the arm follow `target` (n, 6; one row per tick at `hz`)."""
    n = len(target)
    t = np.arange(n) / hz + (lead_s if "lead" in parts else 0.0)
    q = np.column_stack([np.interp(t, np.arange(n) / hz, target[:, j]) for j in range(target.shape[1])])
    if not {"spring", "band"} & set(parts):
        return q
    arm = q[:, :5]
    tau = np.array([grav.tau(x) for x in arm])
    side = motion_side(np.gradient(arm, axis=0) * hz)
    out = q.copy()
    out[:, :5] += np.radians(model.error_deg(tau, side, parts))
    return out


# -- evaluation ----------------------------------------------------------------------------------------------
SKIP_S = 0.5  # the first half second, while the arm gets going, is left out of the statistics


def desired_at(t: np.ndarray, t_sent: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Where the arm should be at times t: the target of the tick sent at that time, linearly in between."""
    return np.column_stack([np.interp(t, t_sent, target[:, j]) for j in range(target.shape[1])])


@dataclass
class Run:
    """One drawn path: the target per tick (sent at t_sent) and every read; tool positions (m) by FK."""

    t_sent: np.ndarray
    target: np.ndarray
    t_dense: np.ndarray
    q_dense: np.ndarray
    tool_target: np.ndarray
    tool_dense: np.ndarray

    @classmethod
    def make(cls, fk, t_sent, target, t_dense, q_dense) -> "Run":
        def tool(q):
            return np.array([fk(x[:5])[:3, 3] for x in q])

        return cls(t_sent, target, t_dense, q_dense, tool(target), tool(q_dense))

    def tracking(self, extra_lag_s: float = 0.0) -> dict:
        """Errors of the measured arm against the target (joint deg; tool mm, by FK), after SKIP_S, until
        the last command. `extra_lag_s` compares with the target that much earlier (a residual delay)."""
        m = (self.t_dense >= self.t_sent[0] + SKIP_S) & (self.t_dense <= self.t_sent[-1])
        t = self.t_dense[m] - extra_lag_s
        joint = np.degrees(self.q_dense[m, :5] - desired_at(t, self.t_sent, self.target)[:, :5])
        tool = (self.tool_dense[m] - desired_at(t, self.t_sent, self.tool_target)) * 1000
        dist = np.linalg.norm(tool, axis=1)
        return {
            "joint_rms_deg": np.sqrt((joint**2).mean(0)),
            "joint_mean_deg": joint.mean(0),
            "tool_median_mm": float(np.median(dist)),
            "tool_rms_mm": float(np.sqrt((dist**2).mean())),
            "tool_max_mm": float(dist.max()),
            "tool_mean_mm": tool.mean(0),
            "t": self.t_dense[m],
            "tool_err_mm": tool,
        }

    def residual_delay_s(self) -> float:
        """The extra lag (s) that best lines the measured path up with the target (RMS tool error). An
        uncompensated band counts in it too: a direction-dependent error lines up with the velocity."""
        lags = np.arange(-0.06, 0.161, 0.005)
        return float(lags[int(np.argmin([self.tracking(lag)["tool_rms_mm"] for lag in lags]))])


# -- a virtual servo -----------------------------------------------------------------------------------------
SIM_DELAY_S = 0.065  # S1a at P = 32: steps half-way after 56-67 ms


def simulate(
    cmd: np.ndarray, hz: float, model: ServoModel, grav: st.Gravity, delay_s: float = SIM_DELAY_S
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The S1 servo model run forward: each command is held for a tick, the joint answers `delay_s` later
    and stops short by the model error (side from the direction the commands move). Returns t_sent (n,),
    t_dense and q_dense (reads 1 ms apart): what `SafeArm.follow(dense=True)` gives on the real arm."""
    n = len(cmd)
    t_sent = np.arange(n) / hz
    t_dense = np.arange(0.0, n / hz, 0.001)
    i = np.clip(np.floor((t_dense - delay_s) * hz).astype(int), 0, n - 1)
    tau = np.array([grav.tau(q) for q in cmd[:, :5]])
    side = motion_side(np.gradient(cmd[:, :5], axis=0) * hz)
    q = cmd[i].copy()
    q[:, :5] -= np.radians(model.error_deg(tau[i], side[i]))
    return t_sent, t_dense, q
