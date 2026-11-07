"""S3: camera intrinsics from a ChArUco board shown on an iPad.

The board image is generated here (OpenCV) and shown pixel for pixel on the iPad (tools/ipad_charuco.html,
264 ppi), so its squares are 25 mm to the iPad's pixel pitch. Intrinsics (focal length, centre, distortion)
do not depend on the square size at all; only lengths (the stereo baseline, a triangulated size) do.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

SQUARES_X, SQUARES_Y = 8, 5
SQUARE_M, MARKER_M = 0.025, 0.018
DICTIONARY = cv2.aruco.DICT_4X4_50
IPAD_PPI = 264.0
MIN_CORNERS = 8  # a view with fewer detected corners is not used (partial views at the image edge count)


def board() -> cv2.aruco.CharucoBoard:
    d = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_M, MARKER_M, d)


def board_image(ppi: float = IPAD_PPI, margin_px: int = 0) -> np.ndarray:
    """The board at the screen's pixel pitch: one image pixel per screen pixel."""
    px_per_m = ppi / 0.0254
    size = (round(SQUARES_X * SQUARE_M * px_per_m), round(SQUARES_Y * SQUARE_M * px_per_m))
    return board().generateImage(size, marginSize=margin_px, borderBits=1)


def detector() -> cv2.aruco.CharucoDetector:
    params = cv2.aruco.CharucoParameters()
    # Look again for the markers the first pass missed, where the board says they must be: on the wrist
    # camera's first frames (iPad glare, the gripper in front) 10.7 -> 19.8 of 20 markers, 8.5 -> 27 corners.
    params.tryRefineMarkers = True
    dparams = cv2.aruco.DetectorParameters()
    dparams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.CharucoDetector(board(), params, dparams)


def spread_out(ids: np.ndarray) -> bool:
    """The corners span at least two rows and two columns of the board with two corners each: corners on
    (nearly) one line fix no pose (a ZED view with 7 corners in one row and 1 below broke the calibration)."""
    cols = SQUARES_X - 1
    _, counts_r = np.unique(ids // cols, return_counts=True)
    _, counts_c = np.unique(ids % cols, return_counts=True)
    return (counts_r >= 2).sum() >= 2 and (counts_c >= 2).sum() >= 2


@dataclass
class View:
    """Detected ChArUco corners in one image: ids (n,), image points (n, 2), board points (n, 3) in m."""

    ids: np.ndarray
    img: np.ndarray
    obj: np.ndarray


def detect(gray: np.ndarray, det: cv2.aruco.CharucoDetector | None = None) -> View | None:
    det = det or detector()
    corners, ids, _, _ = det.detectBoard(gray)
    if ids is None or len(ids) < MIN_CORNERS:
        return None
    ids = ids.ravel()
    if not spread_out(ids):
        return None
    obj = board().getChessboardCorners()[ids]
    return View(ids, corners.reshape(-1, 2).astype(np.float64), obj.astype(np.float64))


@dataclass
class Intrinsics:
    size: tuple[int, int]  # (width, height) px
    K: np.ndarray  # 3x3
    D: np.ndarray  # (5,) k1 k2 p1 p2 k3, OpenCV order

    def to_dict(self) -> dict:
        return {
            "width": self.size[0],
            "height": self.size[1],
            "fx": float(self.K[0, 0]),
            "fy": float(self.K[1, 1]),
            "cx": float(self.K[0, 2]),
            "cy": float(self.K[1, 2]),
            "dist_k1_k2_p1_p2_k3": [float(x) for x in self.D.ravel()[:5]],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Intrinsics":
        K = np.array([[d["fx"], 0, d["cx"]], [0, d["fy"], d["cy"]], [0, 0, 1.0]])
        return cls((d["width"], d["height"]), K, np.array(d["dist_k1_k2_p1_p2_k3"], dtype=float))


@dataclass
class Calibration:
    intr: Intrinsics
    rms_px: float
    per_view_px: np.ndarray
    std: np.ndarray  # std of fx, fy, cx, cy, k1, k2, p1, p2, k3
    rvecs: list
    tvecs: list
    used: np.ndarray  # indices of the views kept


def calibrate(views: list[View], size: tuple[int, int], drop_factor: float = 3.0) -> Calibration:
    """OpenCV calibration with the 5-parameter distortion model (the ZED's factory model); views whose RMS is
    more than drop_factor x the median are dropped once and the calibration is repeated."""
    used = np.arange(len(views))
    for _ in range(2):
        obj = [views[i].obj.astype(np.float32) for i in used]
        img = [views[i].img.astype(np.float32) for i in used]
        rms, K, D, rv, tv, std_in, _, per = cv2.calibrateCameraExtended(obj, img, size, None, None)
        per = per.ravel()
        keep = per <= drop_factor * np.median(per)
        if keep.all():
            break
        used = used[keep]
    return Calibration(Intrinsics(size, K, D.ravel()[:5]), float(rms), per, std_in.ravel()[:9], rv, tv, used)


def reprojection_px(view: View, intr: Intrinsics) -> float:
    """RMS reprojection error of one view with given intrinsics (the board pose fitted by PnP)."""
    ok, rv, tv = cv2.solvePnP(view.obj, view.img, intr.K, intr.D, flags=cv2.SOLVEPNP_ITERATIVE)
    proj, _ = cv2.projectPoints(view.obj, rv, tv, intr.K, intr.D)
    return float(np.sqrt(((proj.reshape(-1, 2) - view.img) ** 2).sum(1).mean()))


def cross_validated_px(views: list[View], size: tuple[int, int], folds: int = 2) -> float:
    """RMS reprojection error of views not used for the calibration (k-fold): a fair number to set against
    a calibration made elsewhere (the factory's)."""
    idx = np.arange(len(views))
    errs = []
    for f in range(folds):
        test = idx[f::folds]
        train = np.setdiff1d(idx, test)
        cal = calibrate([views[i] for i in train], size)
        errs += [reprojection_px(views[i], cal.intr) ** 2 for i in test]
    return float(np.sqrt(np.mean(errs)))


# -- ZED factory calibration ---------------------------------------------------------------------------------
def read_zed_conf(path: Path, mode: str = "HD") -> dict:
    """The factory file (INI): left and right intrinsics for one resolution, and the stereo extrinsics
    (baseline in mm; rotations in rad)."""
    sections: dict[str, dict[str, float]] = {}
    name = None
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if line.startswith("[") and line.endswith("]"):
            name = line[1:-1]
            sections[name] = {}
        elif "=" in line and name is not None:
            k, v = line.split("=", 1)
            try:
                sections[name][k.strip()] = float(v)
            except ValueError:
                pass
    size = {"2K": (2208, 1242), "FHD": (1920, 1080), "HD": (1280, 720), "VGA": (672, 376)}[mode]

    def intr(side: str) -> Intrinsics:
        s = sections[f"{side}_CAM_{mode}"]
        K = np.array([[s["fx"], 0, s["cx"]], [0, s["fy"], s["cy"]], [0, 0, 1.0]])
        return Intrinsics(size, K, np.array([s["k1"], s["k2"], s["p1"], s["p2"], s["k3"]]))

    st = sections["STEREO"]
    return {
        "left": intr("LEFT"),
        "right": intr("RIGHT"),
        "baseline_mm": st["Baseline"],
        "t_mm": np.array([-st["Baseline"], st["TY"], st["TZ"]]),
        "rot_rad": np.array([st[f"RX_{mode}"], st[f"CV_{mode}"], st[f"RZ_{mode}"]]),
    }
