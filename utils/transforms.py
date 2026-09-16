# mms/utils/transforms.py
#
# 좌표계 변환 유틸리티
#
# Notation: T_AB maps coordinates from frame A to frame B
#   x_B = T_AB @ x_A
#
# Frames
# ------
# B : Base frame        — xArm7 base (≡ World)
# F : Turntable frame   — turntable rotation axis center
# O : Object frame      — internal global frame (first-scan reference, Artec-style)
# E : End-Effector frame — robot flange / TCP
# C : Camera frame      — per-device camera frame (C_femto, C_phoxi)

from __future__ import annotations

from pathlib import Path

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


def update_transform(yaml_path: str, key: str, T: np.ndarray,
                     meta: dict | None = None, backup: bool = True) -> None:
    """`yaml_path` 의 `key` **하나만** 갱신한다. 다른 센서 키는 그대로 둔다.

    왜 이 함수가 있나
    ----------------
    예전에는 hand-eye 결과가 `config/calibration/hand_eye_<sensor>.yaml` 에만
    저장되고, 시스템이 실제로 읽는 `config/sensor_frames.yaml` 은 **사람이 손으로
    옮겨 적어야** 했다("갱신하세요" 출력만 있었다). 2026-09-16 에 그 단계가
    빠진 채로 다음 단계를 돌려서, 새로 구한 값이 아니라 **구 스캐너의 T_EC** 로
    rim 캘리브가 진행될 뻔했다. 산출물과 시스템 설정을 한 파일로 합치고,
    캘리브가 **직접** 여기에 쓰도록 바꿨다.

    `meta` 는 같은 매핑 안에 나란히 쓴다(date/method/n_poses/잔차 등).
    `load_transform` 은 translation·rotation_quat 만 읽으므로 무해하고,
    파일만 열어봐도 그 값이 언제·어떻게 나온 건지 알 수 있다.
    """
    path = Path(yaml_path)
    cfg = {}
    if path.exists():
        cfg = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if backup:
            bak = path.with_suffix(path.suffix + ".bak")
            bak.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")

    T = np.asarray(T, dtype=float)
    entry = {
        "translation": [float(v) for v in T[:3, 3]],
        "rotation_quat": [float(v) for v in R.from_matrix(T[:3, :3]).as_quat()],
    }
    entry.update(meta or {})
    cfg[key] = entry

    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# 센서별 T_EC (E→C). **캘리브 스크립트가 직접 쓴다 — 손으로 옮겨 적지 않는다.**\n"
        "#   artec : scripts/artec/calibrate.py --only 2\n"
        "# translation 단위 m, rotation_quat 는 [qx, qy, qz, qw].\n"
        "# 각 키의 date/method/n_poses/t_err_mm 은 그 값이 어디서 나왔는지 기록이다.\n\n"
    )
    path.write_text(
        header + yaml.dump(cfg, default_flow_style=None, allow_unicode=True,
                           sort_keys=False),
        encoding="utf-8",
    )


# ---------- Object frame (O) ----------

class TurntableTransformConfig:
    """
    Static turntable transform configuration (base <-> turntable frames).

    Notation
    --------
    T_AB maps coordinates from frame A to frame B:
        x_B = T_AB @ x_A

    Frames: B = Base (xArm7 root), F = Turntable frame (z-up, origin at center)

    Parameters
    ----------
    T_BF0 : (4,4) np.ndarray
        B → F transform at theta = 0.
        Load with: load_transform("config/calibration/turntable_frame.yaml", "T_B_F0")
    """

    def __init__(self, T_BF0: np.ndarray):
        assert T_BF0.shape == (4, 4)
        self.T_BF0 = T_BF0
        self._T_FB0 = np.linalg.inv(T_BF0)   # cached; T_BF0 is constant

    def T_FB(self, theta: float) -> np.ndarray:
        """
        F → B transform at turntable angle theta (rad).

        x_B = T_FB(theta) @ x_F
        """
        T = self._T_FB0.copy()
        T[:3, :3] = self._T_FB0[:3, :3] @ rotz(theta)
        return T

    def T_BF(self, theta: float) -> np.ndarray:
        """
        B → F transform at turntable angle theta (rad).

        x_F = T_BF(theta) @ x_B
        """
        return np.linalg.inv(self.T_FB(theta))


# backwards-compatible alias used by existing callers
WorldTransformConfig = TurntableTransformConfig


# ---------- Sensor frame (S) / EE frame (E) ----------

def compute_T_CB(T_EB: np.ndarray, T_EC: np.ndarray) -> np.ndarray:
    """
    Compute Camera → Base transform.

    T_CB = T_EB @ inv(T_EC)
    x_B  = T_CB @ x_C

    Parameters
    ----------
    T_EB : (4,4) np.ndarray  —  E → B  (robot FK result)
    T_EC : (4,4) np.ndarray  —  E → C  (hand-eye calibration)

    Returns
    -------
    T_CB : (4,4) np.ndarray  —  C → B
    """
    assert T_EB.shape == (4, 4)
    assert T_EC.shape == (4, 4)
    return T_EB @ np.linalg.inv(T_EC)


# backwards-compatible alias
compute_T_S_B = compute_T_CB


def sensor_pose_to_ee_pose(
    T_BC: np.ndarray,
    T_EC: np.ndarray,
) -> np.ndarray:
    """
    Compute EE target pose from camera target pose.

    T_BE = T_BC @ inv(T_EC)

    Parameters
    ----------
    T_BC : (4,4) np.ndarray
        B → C target transform.
    T_EC : (4,4) np.ndarray
        E → C transform (hand-eye calibration result).

    Returns
    -------
    T_BE : (4,4) np.ndarray
        B → E target transform.
    """
    assert T_BC.shape == (4, 4)
    assert T_EC.shape == (4, 4)
    return T_BC @ np.linalg.inv(T_EC)


# ---------- O–C 체인 (NBV ↔ Hardware 인터페이스) ----------

def compute_T_CO(
    theta: float,
    T_EB: np.ndarray,
    T_OF: np.ndarray,
    T_EC: np.ndarray,
    tt: TurntableTransformConfig,
) -> np.ndarray:
    """
    Compute camera pose in object frame: T_CO (C → O).

    Chain: C → E → B → F → O
    T_CO = inv(T_OF) @ inv(T_FB(θ)) @ T_EB @ inv(T_EC)

    x_O = T_CO @ x_C
    T_CO[:3, 3] = camera origin expressed in O frame (NBV input).

    Parameters
    ----------
    theta : float
        Turntable angle (rad).
    T_EB : (4,4) np.ndarray
        E → B transform from robot FK.
    T_OF : (4,4) np.ndarray
        O → F transform (fixed after first-scan initialisation).
        Use np.eye(4) when O ≡ F at θ = 0.
    T_EC : (4,4) np.ndarray
        E → C transform (hand-eye calibration result).
    tt : TurntableTransformConfig
        Provides T_FB(theta).

    Returns
    -------
    T_CO : (4,4) np.ndarray  —  C → O
    """
    T_FO = np.linalg.inv(T_OF)
    T_BF = np.linalg.inv(tt.T_FB(theta))
    T_CE = np.linalg.inv(T_EC)
    return T_FO @ T_BF @ T_EB @ T_CE


def solve_T_EB(
    theta: float,
    T_CO_des: np.ndarray,
    T_OF: np.ndarray,
    T_EC: np.ndarray,
    tt: TurntableTransformConfig,
) -> np.ndarray:
    """
    Solve for robot target T_EB given a desired camera pose T_CO_des.

    Inverts the T_CO chain:
    T_EB = T_FB(θ) @ T_OF @ T_CO_des @ T_EC

    Derivation:
        T_CO = inv(T_OF) @ inv(T_FB) @ T_EB @ inv(T_EC)
        T_EB = inv(inv(T_OF) @ inv(T_FB)) @ T_CO @ T_EC
             = T_FB @ T_OF @ T_CO @ T_EC

    Parameters
    ----------
    theta : float
        Turntable angle (rad).
    T_CO_des : (4,4) np.ndarray
        Desired camera pose in O frame (C → O).
        Produced by the NBV layer.
    T_OF : (4,4) np.ndarray
        O → F transform (fixed after first-scan initialisation).
    T_EC : (4,4) np.ndarray
        E → C transform (hand-eye calibration result).
    tt : TurntableTransformConfig
        Provides T_FB(theta).

    Returns
    -------
    T_EB : (4,4) np.ndarray  —  E → B  (robot FK target pose)
    """
    return tt.T_FB(theta) @ T_OF @ T_CO_des @ T_EC


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
