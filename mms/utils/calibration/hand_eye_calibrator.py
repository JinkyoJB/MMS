# mms/calibration/hand_eye_calibrator.py
#
# Hand-Eye 캘리브레이션: T_E^S (End-Effector → Sensor) 추정.
#
# AX = XB 방정식 (Eye-in-Hand):
#   A_i = inv(T_E_B[i]) @ T_E_B[i+1]   ... EE 간 상대 운동 (base frame)
#   B_i = T_M_S[i+1] @ inv(T_M_S[i])   ... 마커 간 상대 운동 (sensor frame)
#   X   = T_E_S                          ... 구하는 변환
#
# 입력 단위:
#   T_E_B : translation meters  (robot FK 출력, mm → m 변환 후 전달)
#   T_M_S : translation mm      (PhoxiClient.detect_marker_transform() 출력)
#           → add_sample() 내부에서 m 으로 변환
#
# 저장 형식 (config/sensor_frames.yaml):
#   T_E_S_phoxi:
#     translation: [tx, ty, tz]        # meters
#     rotation_quat: [qx, qy, qz, qw]

from __future__ import annotations

import logging
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import yaml

log = logging.getLogger(__name__)


def _rot_to_quat(R: np.ndarray) -> np.ndarray:
    """(3,3) rotation matrix → [qx, qy, qz, qw]."""
    m = R
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


class HandEyeCalibrator:
    """
    Hand-Eye 캘리브레이션 (AX=XB, Eye-in-Hand 방식).

    사용 흐름: add_sample() 반복 → calibrate() → save_yaml()

    Parameters
    ----------
    method : int
        cv2.CALIB_HAND_EYE_* 상수. 기본 = TSAI.
        calibrate()는 모든 방법을 시도하고 잔차 최소 결과를 선택한다.

    Notation
    --------
    T_E_B : (4,4)  EE → Base        (robot FK, translation meters)
    T_M_S : (4,4)  Marker → Sensor  (PhoxiClient.detect_marker_transform(), translation mm)
    T_E_S : (4,4)  EE → Sensor      (캘리브레이션 결과)
    """

    def __init__(self, method: int = cv2.CALIB_HAND_EYE_TSAI) -> None:
        self._method = method
        self._T_E_B_list: List[np.ndarray] = []
        self._T_M_S_list: List[np.ndarray] = []
        self._T_E_S: Optional[np.ndarray] = None

    @property
    def n_samples(self) -> int:
        return len(self._T_E_B_list)

    @property
    def T_E_S(self) -> Optional[np.ndarray]:
        return self._T_E_S

    def add_sample(self, T_E_B: np.ndarray, T_M_S: np.ndarray) -> None:
        """
        캘리브레이션 샘플 추가.

        Parameters
        ----------
        T_E_B : (4,4)  EE → Base, translation in meters.
        T_M_S : (4,4)  Marker → Sensor, translation in mm.
                       내부적으로 m 단위로 변환하여 저장한다.
        """
        assert T_E_B.shape == (4, 4) and T_M_S.shape == (4, 4)
        self._T_E_B_list.append(T_E_B.copy())
        T_M_S_m = T_M_S.copy()
        T_M_S_m[:3, 3] /= 1000.0   # mm → m
        self._T_M_S_list.append(T_M_S_m)
        log.info(
            f"[HandEye] 샘플 추가 ({self.n_samples}개)  "
            f"t_M_S={np.linalg.norm(T_M_S_m[:3, 3])*1000:.1f}mm"
        )

    def calibrate(self) -> np.ndarray:
        """
        OpenCV calibrateHandEye로 T_E_S 계산.
        모든 방법(TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS)을 시도하고
        AX=XB 잔차가 가장 작은 결과를 반환한다.

        Returns
        -------
        T_E_S : (4,4) float64

        Raises
        ------
        RuntimeError : 샘플 부족 또는 모든 방법 실패 시.
        """
        if self.n_samples < 3:
            raise RuntimeError(f"샘플 부족: {self.n_samples}개 (최소 3개 필요)")

        R_g2b = [T[:3, :3] for T in self._T_E_B_list]
        t_g2b = [T[:3, 3:4] for T in self._T_E_B_list]
        R_t2c = [T[:3, :3] for T in self._T_M_S_list]
        t_t2c = [T[:3, 3:4] for T in self._T_M_S_list]

        methods = [
            ("TSAI",       cv2.CALIB_HAND_EYE_TSAI),
            ("PARK",       cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD",     cv2.CALIB_HAND_EYE_HORAUD),
            ("ANDREFF",    cv2.CALIB_HAND_EYE_ANDREFF),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]

        best_T_E_S = None
        best_t_err = np.inf
        best_r_err = np.inf
        best_name  = None

        for name, method in methods:
            try:
                R_c2e, t_c2e = cv2.calibrateHandEye(
                    R_g2b, t_g2b, R_t2c, t_t2c, method=method
                )
            except Exception as e:
                log.warning(f"[HandEye] {name} 예외: {e}")
                continue

            T_S_E = np.eye(4)
            T_S_E[:3, :3] = R_c2e
            T_S_E[:3, 3]  = t_c2e.ravel()

            if np.linalg.norm(T_S_E[:3, 3]) < 1e-6:
                log.warning(f"[HandEye] {name}: translation≈0 (실패)")
                continue
            if abs(np.linalg.det(R_c2e) - 1.0) > 0.05:
                log.warning(f"[HandEye] {name}: det(R)={np.linalg.det(R_c2e):.3f} (비정상)")
                continue

            T_E_S_cand = np.linalg.inv(T_S_E)
            t_err, r_err = self._compute_residuals(T_E_S_cand)
            log.info(
                f"[HandEye] {name}: t_err={t_err*1000:.2f}mm  r_err={r_err:.2f}°  "
                f"t_norm={np.linalg.norm(T_S_E[:3,3])*1000:.1f}mm"
            )

            if t_err < best_t_err:
                best_t_err, best_r_err = t_err, r_err
                best_T_E_S = T_E_S_cand
                best_name  = name

        if best_T_E_S is None:
            raise RuntimeError(
                "[HandEye] 모든 캘리브레이션 방법 실패.\n"
                "  → 포즈의 회전 다양성을 늘리세요 (최소 5개, 다양한 tilting 포함)."
            )

        if best_t_err * 1000 > 50.0:
            log.warning(
                f"[HandEye] 잔차 t={best_t_err*1000:.1f}mm 큼 — "
                "마커 감지 품질 또는 포즈 다양성 확인 필요"
            )

        self._T_E_S = best_T_E_S
        log.info(
            f"[HandEye] 최적: {best_name}  "
            f"t_err={best_t_err*1000:.2f}mm  r_err={best_r_err:.2f}°  "
            f"T_E_S t={self._T_E_S[:3,3]*1000} mm"
        )
        return self._T_E_S

    def _compute_residuals(self, T_E_S: np.ndarray) -> tuple[float, float]:
        """
        T_B_M 일관성 잔차 (mean_t_m, mean_r_deg).

        올바른 체인: T(M→B) = T_E_B @ inv(T_E_S) @ T_M_S
        마커가 고정돼 있으면 모든 포즈에서 T_B_M 이 동일해야 한다.
        """
        if self.n_samples < 2:
            return np.inf, np.inf
        T_S_E = np.linalg.inv(T_E_S)
        errs_t, errs_r = [], []
        for i in range(self.n_samples - 1):
            T_B_M_i  = self._T_E_B_list[i]     @ T_S_E @ self._T_M_S_list[i]
            T_B_M_i1 = self._T_E_B_list[i + 1] @ T_S_E @ self._T_M_S_list[i + 1]
            diff = np.linalg.inv(T_B_M_i1) @ T_B_M_i
            errs_t.append(np.linalg.norm(diff[:3, 3]))
            angle = np.arccos(np.clip((np.trace(diff[:3, :3]) - 1) / 2, -1.0, 1.0))
            errs_r.append(np.degrees(angle))
        return float(np.mean(errs_t)), float(np.mean(errs_r))

    def save_yaml(
        self,
        yaml_path: Path,
        sensor: str = "phoxi_m",
        method: str = "photoneo_a4rev23a",
    ) -> None:
        """
        T_E_S를 YAML로 저장 (config/calibration/hand_eye_<sensor>.yaml 권장).

        형식:
          sensor, date, method, n_poses, T_E_C: {translation, rotation_quat, matrix}

        translation 단위: meters.
        """
        import datetime

        if self._T_E_S is None:
            raise RuntimeError("calibrate()를 먼저 호출하세요.")
        if np.linalg.norm(self._T_E_S[:3, 3]) * 1000 < 1.0:
            raise RuntimeError("T_E_S translation≈0 — 캘리브레이션 결과가 유효하지 않습니다.")

        t = self._T_E_S[:3, 3]
        q = _rot_to_quat(self._T_E_S[:3, :3])
        mat = self._T_E_S.tolist()

        yaml_path = Path(yaml_path)
        yaml_path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "sensor": sensor,
            "date": datetime.date.today().isoformat(),
            "method": method,
            "n_poses": self.n_samples,
            "T_E_C": {
                "translation":   [float(v) for v in t],
                "rotation_quat": [float(v) for v in q],
                "matrix":        [[float(v) for v in row] for row in mat],
            },
        }
        yaml_path.write_text(
            yaml.dump(data, default_flow_style=None, allow_unicode=True),
            encoding="utf-8",
        )
        log.info(f"[HandEye] 저장: {yaml_path}")
        print(f"[HandEye] 저장: {yaml_path}")
        print(f"  translation (m) : {t.tolist()}")
        print(f"  rotation_quat   : {q.tolist()}")
