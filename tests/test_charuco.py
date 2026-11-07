"""S3: the ChArUco calibration against a known stereo camera.

The calibration maths is tested on corners projected exactly with the known camera (plus noise); image
rendering is only smoke-tested (the board image's squares are rounded to whole pixels, so rendered corners
are not a clean ground truth).
"""

import cv2
import numpy as np
import pytest

from so101_calib import cam_cli
from so101_calib import charuco as ch

SIZE = (1280, 720)
K_L = np.array([[534.7, 0, 638.1], [0, 534.9, 348.4], [0, 0, 1.0]])
K_R = np.array([[534.0, 0, 647.7], [0, 534.1, 354.2], [0, 0, 1.0]])
D_L = np.array([-0.0496, 0.0235, 0.0003, -0.0003, -0.0097])
D_R = np.array([-0.0473, 0.0197, -0.0001, -0.0002, -0.0081])
T_RL = np.array([-0.120, 0.0, 0.0])  # right eye in the left eye's frame: x_r = x_l + T
NOISE_PX = 0.2


def poses(n: int, seed: int = 0):
    """Board poses that cover the whole image, near and far, tilted up to 45 deg. (Covering only part of
    the image lets the distortion trade off against the focal length: 1.2 % off with the board on one
    side only.)"""
    rng = np.random.default_rng(seed)
    for _ in range(n):
        rvec = np.radians(rng.uniform([-45, -45, -20], [45, 45, 20]))
        z = rng.uniform(0.25, 0.6)
        tvec = np.array([rng.uniform(-0.8, 0.4) * z, rng.uniform(-0.5, 0.3) * z, z])
        yield rvec, tvec


def project(K, D, rvec, tvec, rng) -> ch.View | None:
    obj = ch.board().getChessboardCorners().astype(np.float64)
    img, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    img = img.reshape(-1, 2) + rng.normal(0, NOISE_PX, (len(obj), 2))
    inside = (img[:, 0] > 5) & (img[:, 0] < SIZE[0] - 5) & (img[:, 1] > 5) & (img[:, 1] < SIZE[1] - 5)
    if inside.sum() < ch.MIN_CORNERS:
        return None
    ids = np.flatnonzero(inside)
    return ch.View(ids, img[inside], obj[inside])


@pytest.fixture(scope="module")
def eyes():
    rng = np.random.default_rng(1)
    left, right = [], []
    for rvec, tvec in poses(60):
        vl, vr = project(K_L, D_L, rvec, tvec, rng), project(K_R, D_R, rvec, tvec + T_RL, rng)
        if vl is not None and vr is not None:
            left.append(vl)
            right.append(vr)
    return [left, right]


def factory() -> dict:
    return {
        "left": ch.Intrinsics(SIZE, K_L, D_L),
        "right": ch.Intrinsics(SIZE, K_R, D_R),
        "baseline_mm": 120.0,
        "t_mm": T_RL * 1000,
        "rot_rad": np.zeros(3),
    }


def test_calibration_recovers_the_camera(eyes):
    cal = ch.calibrate(eyes[0], SIZE)
    assert cal.rms_px == pytest.approx(NOISE_PX * np.sqrt(2), rel=0.2)
    assert np.allclose(cal.intr.K, K_L, atol=1.5)
    # the true camera reprojects the views as well as the fitted one (both at the noise level)
    true_px = np.sqrt(np.mean([ch.reprojection_px(v, factory()["left"]) ** 2 for v in eyes[0]]))
    assert true_px == pytest.approx(ch.cross_validated_px(eyes[0], SIZE), rel=0.15)


def test_stereo_gives_the_baseline_and_the_square_size(eyes):
    cals = {"left": ch.calibrate(eyes[0], SIZE), "right": ch.calibrate(eyes[1], SIZE)}
    lines = cam_cli.stereo_report(eyes, SIZE, cals, factory())
    ours = next(x for x in lines if x.startswith("| ours"))
    fac = next(x for x in lines if x.startswith("| factory"))
    assert float(ours.split("|")[3]) == pytest.approx(120.0, abs=1.0)
    for row in (ours, fac):
        assert float(row.split("|")[-2]) == pytest.approx(25.0, abs=0.1)  # triangulated square size


def test_a_rendered_board_is_detected():
    img = ch.board_image()
    view = ch.detect(cv2.copyMakeBorder(img, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))
    assert view is not None and len(view.ids) == (ch.SQUARES_X - 1) * (ch.SQUARES_Y - 1)


def test_board_page_matches_the_board():
    html = (cam_cli.TOOLS / "ipad_charuco.html").read_text()
    assert f"BOARD_PX = [{ch.board_image().shape[1]}, {ch.board_image().shape[0]}]" in html


def test_factory_file_is_parsed(tmp_path):
    conf = tmp_path / "f.conf"
    conf.write_text(
        "[LEFT_CAM_HD]\nfx=534.7\nfy=534.91\ncx=638.09\ncy=348.35\nk1=-0.05\nk2=0.02\np1=0\np2=0\nk3=-0.01\n\n"
        "[RIGHT_CAM_HD]\nfx=533.95\nfy=534.1\ncx=647.7\ncy=354.2\nk1=-0.04\nk2=0.02\np1=0\np2=0\nk3=-0.008\n\n"
        "[STEREO]\nBaseline=120.01\nTY=0.03\nTZ=0.05\nCV_HD=0.0005\nRX_HD=0.002\nRZ_HD=0.0002\n"
    )
    f = ch.read_zed_conf(conf)
    assert f["left"].K[0, 0] == 534.7 and f["baseline_mm"] == 120.01
    assert np.allclose(f["rot_rad"], [0.002, 0.0005, 0.0002])


def test_corners_on_one_line_are_rejected():
    assert not ch.spread_out(np.array([0, 1, 2, 3, 4, 5, 6, 13]))  # one row and a single corner below
    assert ch.spread_out(np.array([0, 1, 2, 7, 8, 9]))
