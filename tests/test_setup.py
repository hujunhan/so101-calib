"""Smoke tests: the pinned dependencies install and agree with each other."""

import numpy as np


def test_m0_and_m1_are_importable_and_consistent():
    from so101_core.model import ARM_JOINTS
    from so101_kinematics.models import N, PinArm

    assert N == len(ARM_JOINTS) == 5
    T = PinArm().fk(np.zeros(N))
    assert T.shape == (4, 4) and np.allclose(T[3], [0, 0, 0, 1])


def test_opencv_has_charuco():
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((5, 7), 0.03, 0.022, dictionary)
    assert board.getChessboardSize() == (5, 7)
