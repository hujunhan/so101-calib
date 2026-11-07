"""SO-101 calibration (M2 of the SO-101 series).

Conventions (from so101-core): SI units and radians inside; transforms are named T_<to>_<from>, 4x4,
stored in YAML; devices are opened by their by-id path; every motion of the real arm goes through
so101-core's SafeArm. Kinematics (FK, IK) come from so101-kinematics (M1).
"""
