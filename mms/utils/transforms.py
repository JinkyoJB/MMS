# mms/utils/transforms.py
#
# 좌표계 변환 유틸리티
#
# Notation: T_A_B maps coordinates from frame A to frame B
#   x_B = T_A_B @ x_A
#
# Frames
# ------
# B : Base frame  — xArm7 base (≡ World)
# O : Object frame — turntable/object (z-up, origin at center)
# E : End-Effector frame — robot flange / TCP
# S : Sensor frame — per-device camera frame (S_femto, S_phoxi)

from __future__ import annotations

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


# ---------- 기본 회전 유틸 ----------

def rotz(theta: float) -> np.ndarray:
    """3x3 rotation matrix for rotation about z-axis by theta (rad)."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


# ---------- YAML 로더 ----------

def load_transform(yaml_path: str, key: str) -> np.ndarray:
    """
    Load an SE3 transform from a YAML file.

    YAML schema
    -----------
    <key>:
      translation: [x, y, z]
      rotation_quat: [qx, qy, qz, qw]

    Returns
    -------
    T : (4,4) np.ndarray
        Homogeneous transform matrix.
    """
    with open(yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    entry = cfg[key]
    t = np.array(entry["translation"], dtype=float)
    q = np.array(entry["rotation_quat"], dtype=float)

    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


# ---------- Object frame (O) ----------

class WorldTransformConfig:
    """
    Static world transform configuration (base <-> object frames).

    Notation
    --------
    T_A_B maps coordinates from frame A to frame B:
        x_B = T_A_B @ x_A

    Parameters
    ----------
    T_B_O0 : (4,4) np.ndarray
        Base → Object transform at theta = 0.
        Load with: load_transform("config/object_frame.yaml", "T_B_O0")
    """

    def __init__(self, T_B_O0: np.ndarray):
        assert T_B_O0.shape == (4, 4)
        self.T_B_O0 = T_B_O0
        self._T_O_B0 = np.linalg.inv(T_B_O0)   # cached; T_B_O0 is constant

    def T_O_B(self, theta: float) -> np.ndarray:
        """
        Object → Base transform at turntable angle theta (rad).

        x_B = T_O_B(theta) @ x_O
        """
        T = self._T_O_B0.copy()
        T[:3, :3] = self._T_O_B0[:3, :3] @ rotz(theta)
        return T

    def T_B_O(self, theta: float) -> np.ndarray:
        """
        Base → Object transform at turntable angle theta (rad).

        x_O = T_B_O(theta) @ x_B
        """
        return np.linalg.inv(self.T_O_B(theta))


# ---------- Sensor frame (S) / EE frame (E) ----------

def compute_T_S_B(T_E_B: np.ndarray, T_E_S: np.ndarray) -> np.ndarray:
    """
    Compute Sensor → Base transform.

    T_S_B = T_E_B @ inv(T_E_S)
    x_B   = T_S_B @ x_S

    Parameters
    ----------
    T_E_B : (4,4) np.ndarray  —  EE → Base  (robot FK result)
    T_E_S : (4,4) np.ndarray  —  EE → Sensor  (hand-eye calibration)

    Returns
    -------
    T_S_B : (4,4) np.ndarray  —  Sensor → Base
    """
    assert T_E_B.shape == (4, 4)
    assert T_E_S.shape == (4, 4)
    return T_E_B @ np.linalg.inv(T_E_S)


def sensor_pose_to_ee_pose(
    T_B_S: np.ndarray,
    T_E_S: np.ndarray,
) -> np.ndarray:
    """
    Compute EE target pose from sensor target pose.

    T_B_E = T_B_S @ inv(T_E_S)

    Parameters
    ----------
    T_B_S : (4,4) np.ndarray
        Base → Sensor target transform.
    T_E_S : (4,4) np.ndarray
        EE → Sensor transform (hand-eye calibration result).

    Returns
    -------
    T_B_E : (4,4) np.ndarray
        Base → EE target transform.
    """
    assert T_B_S.shape == (4, 4)
    assert T_E_S.shape == (4, 4)
    return T_B_S @ np.linalg.inv(T_E_S)


# ---------- 포즈/점/법선 유틸 ----------

def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """
    Apply SE3 transform T to one or more 3D points.

    Parameters
    ----------
    T : (4,4) np.ndarray
    pts : (3,) or (N,3) np.ndarray

    Returns
    -------
    np.ndarray, same shape as pts
    """
    assert T.shape == (4, 4)
    if pts.ndim == 1:
        return (T[:3, :3] @ pts) + T[:3, 3]
    return pts @ T[:3, :3].T + T[:3, 3]


def transform_normals(T: np.ndarray, normals: np.ndarray) -> np.ndarray:
    """
    Apply the rotation part of T to one or more surface normals.
    Translation is ignored (normals are direction vectors).

    Parameters
    ----------
    T : (4,4) np.ndarray
    normals : (3,) or (N,3) np.ndarray

    Returns
    -------
    np.ndarray, same shape as normals
    """
    assert T.shape == (4, 4)
    if normals.ndim == 1:
        return T[:3, :3] @ normals
    return normals @ T[:3, :3].T


def pose_mat_to_6d(T: np.ndarray, order: str = "xyz") -> np.ndarray:
    """
    Convert SE3 matrix to (x, y, z, r1, r2, r3).

    Parameters
    ----------
    T : (4,4) np.ndarray
    order : str, default='xyz'
        Euler angle convention passed to scipy.Rotation.as_euler.

    Returns
    -------
    np.ndarray, shape (6,)
        [x, y, z, r1, r2, r3] with Euler angles in radians.
    """
    assert T.shape == (4, 4)
    rpy = R.from_matrix(T[:3, :3]).as_euler(order, degrees=False)
    return np.concatenate([T[:3, 3], rpy])


def pose6d_to_mat(pose6d: np.ndarray, order: str = "xyz") -> np.ndarray:
    """
    Convert [x(mm), y(mm), z(mm), r1, r2, r3] to (4,4) SE3 matrix.

    xArm get_position() 결과를 T_E_B 행렬로 변환할 때 사용.
    Translation은 mm → m 변환이 자동 적용된다.

    Parameters
    ----------
    pose6d : np.ndarray, shape (6,)
        [x(mm), y(mm), z(mm), r1(rad), r2(rad), r3(rad)]
    order : str, default='xyz'
        Euler angle convention (scipy.Rotation.from_euler).

    Returns
    -------
    T : (4,4) np.ndarray  —  SE3 transform (translation in meters)
    """
    pose6d = np.asarray(pose6d, dtype=float)
    assert pose6d.shape == (6,)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_euler(order, pose6d[3:6], degrees=False).as_matrix()
    T[:3, 3] = pose6d[:3] / 1000.0   # mm → m
    return T
