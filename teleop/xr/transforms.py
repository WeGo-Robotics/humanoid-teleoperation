"""OpenXR -> robot convention, on the host.

Ported from `tv_wrapper.get_tele_data()` so that **every** XR device shares one
implementation of the safety-relevant geometry. The alternative -- each device
transforming before it transmits -- gives every new client a fresh chance to get
a sign wrong, in a language and on a machine where it is hardest to debug.

Conventions
-----------
* OpenXR basis: y up, z back, x right.
* Robot basis:  z up, y left, x front.

`T_ROBOT_OPENXR` is a change of basis, applied as a similarity transform
``T @ M @ T^-1`` so that a pose expressed in OpenXR produces the same physical
rotation when expressed in robot axes.

Controller vs hand tracking
---------------------------
Controller poses already follow the Unitree arm URDF initial-pose convention, so
only the basis change and the frame offsets apply. Hand-tracking wrist poses use
the OpenXR hand convention and need an extra 90 degree roll per side.
"""
from __future__ import annotations

import numpy as np

# --- basis change -----------------------------------------------------------
T_ROBOT_OPENXR = np.array([[0, 0, -1, 0],
                           [-1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0, 1]], dtype=float)

T_OPENXR_ROBOT = np.array([[0, -1, 0, 0],
                           [0, 0, 1, 0],
                           [-1, 0, 0, 0],
                           [0, 0, 0, 1]], dtype=float)

# --- OpenXR hand pose -> Unitree arm URDF initial pose ----------------------
# 90 degrees about the wrist's own x-axis: counter-clockwise for the left arm,
# clockwise for the right.
T_TO_UNITREE_LEFT_ARM = np.array([[1, 0, 0, 0],
                                  [0, 0, -1, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 0, 1]], dtype=float)

T_TO_UNITREE_RIGHT_ARM = np.array([[1, 0, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, -1, 0, 0],
                                   [0, 0, 0, 1]], dtype=float)

# Fallbacks substituted when a pose is unusable, matching tv_wrapper.
CONST_HEAD_POSE = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 1.5],
                            [0, 0, 1, -0.2],
                            [0, 0, 0, 1]], dtype=float)

CONST_LEFT_ARM_POSE = np.array([[1, 0, 0, -0.15],
                                [0, 1, 0, 1.13],
                                [0, 0, 1, -0.3],
                                [0, 0, 0, 1]], dtype=float)

CONST_RIGHT_ARM_POSE = np.array([[1, 0, 0, 0.15],
                                 [0, 1, 0, 1.13],
                                 [0, 0, 1, -0.3],
                                 [0, 0, 0, 1]], dtype=float)

#: Head -> waist origin offset. The IK solver's origin sits near the waist
#: joint, but wrist poses arrive relative to the head.
WAIST_OFFSET_X = 0.15
WAIST_OFFSET_Z = 0.45


def is_usable(mat) -> bool:
    """A pose is usable if it is finite and non-singular.

    Mirrors `tv_wrapper.safe_mat_update`. Note what this does *not* catch: a
    frozen but perfectly valid pose. Staleness is the watchdog's job, not this
    function's -- see teleop/safety.
    """
    arr = np.asarray(mat, dtype=float)
    if arr.shape != (4, 4) or not np.all(np.isfinite(arr)):
        return False
    det = np.linalg.det(arr)
    return bool(np.isfinite(det) and not np.isclose(det, 0.0, atol=1e-6))


def change_basis(mat: np.ndarray) -> np.ndarray:
    """Express an OpenXR pose in robot axes."""
    return T_ROBOT_OPENXR @ np.asarray(mat, dtype=float) @ T_OPENXR_ROBOT


def openxr_to_robot(head, left_wrist, right_wrist, hand_tracking: bool):
    """Full chain: raw OpenXR poses -> (head, left, right) IK targets.

    Returns ``(head_pose, left_target, right_target, valid)`` where `valid` is
    ``(head_ok, left_ok, right_ok)``. Unusable poses fall back to the same
    constants tv_wrapper uses, and are reported so the caller can mark the side
    untracked rather than silently acting on a fabricated pose.
    """
    head_ok = is_usable(head)
    left_ok = is_usable(left_wrist)
    right_ok = is_usable(right_wrist)

    head_xr = np.asarray(head, dtype=float) if head_ok else CONST_HEAD_POSE
    left_xr = np.asarray(left_wrist, dtype=float) if left_ok else CONST_LEFT_ARM_POSE
    right_xr = np.asarray(right_wrist, dtype=float) if right_ok else CONST_RIGHT_ARM_POSE

    head_robot = change_basis(head_xr)
    left_robot = change_basis(left_xr)
    right_robot = change_basis(right_xr)

    if hand_tracking:
        # Only rotate a pose we actually trust; rotating the fallback would move
        # it somewhere the operator never was.
        if left_ok:
            left_robot = left_robot @ T_TO_UNITREE_LEFT_ARM
        if right_ok:
            right_robot = right_robot @ T_TO_UNITREE_RIGHT_ARM

    # world -> head (translation only), then head -> waist
    left_target = left_robot.copy()
    right_target = right_robot.copy()
    for target in (left_target, right_target):
        target[0:3, 3] -= head_robot[0:3, 3]
        target[0, 3] += WAIST_OFFSET_X
        target[2, 3] += WAIST_OFFSET_Z

    return head_robot, left_target, right_target, (head_ok, left_ok, right_ok)


def hand_joints_to_robot(joints_xr, wrist_xr_robot) -> np.ndarray:
    """(25,3) OpenXR hand joint positions -> the arm frame, robot axes.

    Only needed once hand tracking ships; kept here so the whole transform
    story lives in one file.
    """
    pts = np.asarray(joints_xr, dtype=float)
    homogeneous = np.concatenate([pts.T, np.ones((1, pts.shape[0]))])
    world = T_ROBOT_OPENXR @ homogeneous
    inv = np.eye(4)
    inv[:3, :3] = wrist_xr_robot[:3, :3].T
    inv[:3, 3] = -wrist_xr_robot[:3, :3].T @ wrist_xr_robot[:3, 3]
    return (inv @ world)[0:3, :].T
