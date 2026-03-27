# mms/core/transforms.py

from __future__ import annotations

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


# ---------- 기본 회전 유틸 ----------

def rotz(theta: float) -> np.ndarray:
    """
    3x3 rotation matrix for rotation about z-axis by theta (rad).
    """
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


# ---------- Object(Frame O) 관련 ----------

def load_T_B_O0(yaml_path: str) -> np.ndarray:
    """
    Load base->object transform at theta = 0 from config/object_frame.yaml.

    YAML schema
    -----------
    T_B_O0:
      translation: [x, y, z]
      rotation_quat: [qx, qy, qz, qw]
    """
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    cfg_o = cfg["T_B_O0"]
    t = np.array(cfg_o["translation"], dtype=float)          # (3,)
    q = np.array(cfg_o["rotation_quat"], dtype=float)        # (4,)

    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


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
        Loaded from config/object_frame.yaml (key: T_B_O0).
    """

    def __init__(self, T_B_O0: np.ndarray):
        assert T_B_O0.shape == (4, 4)
        self.T_B_O0 = T_B_O0

    def T_O_B(self, theta: float) -> np.ndarray:
        """
        Object → Base transform at turntable angle theta (rad).

        x_B = T_O_B(theta) @ x_O
        """
        T_O_B0 = np.linalg.inv(self.T_B_O0)
        Rz = rotz(theta)
        T = T_O_B0.copy()
        T[:3, :3] = T_O_B0[:3, :3] @ Rz
        return T

    def T_B_O(self, theta: float) -> np.ndarray:
        """
        Base → Object transform at turntable angle theta (rad).

        x_O = T_B_O(theta) @ x_B
        """
        return np.linalg.inv(self.T_O_B(theta))


# ---------- Sensor(Frame S) 관련 (EE <-> Sensor) ----------

def load_T_E_S(yaml_path: str, key: str) -> np.ndarray:
    """
    Load EE->Sensor transform T_E_S from config/sensor_frames.yaml.

    YAML schema
    -----------
    T_E_S_femto:
      translation: [tx, ty, tz]
      rotation_quat: [qx, qy, qz, qw]
    """
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)

    cfg_s = cfg[key]
    t = np.array(cfg_s["translation"], dtype=float)
    q = np.array(cfg_s["rotation_quat"], dtype=float)

    T = np.eye(4, dtype=float)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def sensor_target_to_ee_target(
    T_B_S_target: np.ndarray,
    T_E_S: np.ndarray,
) -> np.ndarray:
    """
    Compute EE target pose from sensor target pose.

    Parameters
    ----------
    T_B_S_target : (4,4) np.ndarray
        Base->Sensor target transform T_B^S,target.

    T_E_S : (4,4) np.ndarray
        EE->Sensor transform T_E^S (hand-eye result).

    Returns
    -------
    T_B_E_target : (4,4) np.ndarray
        Base->EE target transform T_B^E,target.
    """
    assert T_B_S_target.shape == (4, 4)
    assert T_E_S.shape == (4, 4)
    T_S_E = np.linalg.inv(T_E_S)
    return T_B_S_target @ T_S_E


# ---------- 포즈/점/법선 유틸 ----------

def transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    """
    Apply SE3 transform T to 3D point p.

    x_out = T @ [p; 1]
    """
    assert T.shape == (4, 4)
    p_h = np.ones(4, dtype=float)
    p_h[:3] = p
    return (T @ p_h)[:3]


def transform_normal(R_mat: np.ndarray, n: np.ndarray) -> np.ndarray:
    """
    Apply rotation R to normal n (no translation).
    """
    assert R_mat.shape == (3, 3)
    return R_mat @ n


def pose_mat_to_6d(T: np.ndarray, order: str = "xyz") -> np.ndarray:
    """
    Convert SE3 matrix to (x, y, z, r1, r2, r3) in base frame.

    Parameters
    ----------
    T : (4,4) np.ndarray
        SE3 transform.

    order : {'xyz', 'zyx'}, default='xyz'
        Euler angle convention passed to scipy.Rotation.as_euler.

    Returns
    -------
    pose6d : np.ndarray, shape (6,)
        [x, y, z, r1, r2, r3] where (r1,r2,r3) are Euler angles.
    """
    assert T.shape == (4, 4)
    t = T[:3, 3]
    R_mat = T[:3, :3]
    rpy = R.from_matrix(R_mat).as_euler(order, degrees=False)
    return np.concatenate([t, rpy], axis=0)