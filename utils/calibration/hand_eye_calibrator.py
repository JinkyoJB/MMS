# mms/calibration/hand_eye_calibrator.py
#
# Hand-Eye 캘리브레이션: T_EC (End-Effector → Camera) 추정.
#
# AX = XB 방정식 (Eye-in-Hand):
#   A_i = inv(T_EB[i]) @ T_EB[i+1]   ... EE 간 상대 운동 (base frame)
#   B_i = T_MC[i+1] @ inv(T_MC[i])   ... 마커 간 상대 운동 (camera frame)
#   X   = T_EC                         ... 구하는 변환
#
# 입력 단위:
#   T_EB : translation meters  (robot FK 출력, mm → m 변환 후 전달)
#   T_MC : translation mm      (PhoxiClient.detect_marker_transform() 출력)
#          → add_sample() 내부에서 m 으로 변환
#
# 저장 형식 (config/calibration/hand_eye_phoxi.yaml):
#   T_E_C:
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
    T_EB : (4,4)  E → B  (robot FK, translation meters)
    T_MC : (4,4)  M → C  (PhoxiClient.detect_marker_transform(), translation mm)
    T_EC : (4,4)  E → C  (캘리브레이션 결과)
    """

    def __init__(self, method: int = cv2.CALIB_HAND_EYE_TSAI) -> None:
        self._method = method
        self._T_EB_list: List[np.ndarray] = []
        self._T_MC_list: List[np.ndarray] = []
        self._T_EC: Optional[np.ndarray] = None
        #: `calibrate()` 가 고른 해의 잔차. 결과 yaml 메타데이터로 남긴다 —
        #  예전엔 로그에만 찍혀서, 저장된 T_EC 가 얼마나 믿을 만한지
        #  파일만 봐서는 알 수 없었다.
        self._t_err_m: Optional[float] = None
        self._r_err_deg: Optional[float] = None

    @property
    def n_samples(self) -> int:
        return len(self._T_EB_list)

    @property
    def residuals(self) -> tuple[Optional[float], Optional[float]]:
        """(t_err [m], r_err [deg]) — `calibrate()` 전이면 (None, None)."""
        return self._t_err_m, self._r_err_deg

    @property
    def T_EC(self) -> Optional[np.ndarray]:
        return self._T_EC

    # backwards-compatible alias
    @property
    def T_E_S(self) -> Optional[np.ndarray]:
        return self._T_EC

    def add_sample(self, T_EB: np.ndarray, T_MC: np.ndarray) -> None:
        """
        캘리브레이션 샘플 추가.

        Parameters
        ----------
        T_EB : (4,4)  E → B, translation in meters.
        T_MC : (4,4)  M → C, translation in mm.
                      내부적으로 m 단위로 변환하여 저장한다.
        """
        assert T_EB.shape == (4, 4) and T_MC.shape == (4, 4)
        self._T_EB_list.append(T_EB.copy())
        T_MC_m = T_MC.copy()
        T_MC_m[:3, 3] /= 1000.0   # mm → m
        self._T_MC_list.append(T_MC_m)
        log.info(
            f"[HandEye] 샘플 추가 ({self.n_samples}개)  "
            f"t_MC={np.linalg.norm(T_MC_m[:3, 3])*1000:.1f}mm"
        )

    def calibrate(self) -> np.ndarray:
        """
        OpenCV calibrateHandEye로 T_EC 계산.
        모든 방법(TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS)을 시도하고
        AX=XB 잔차가 가장 작은 결과를 반환한다.

        Returns
        -------
        T_EC : (4,4) float64

        Raises
        ------
        RuntimeError : 샘플 부족 또는 모든 방법 실패 시.
        """
        if self.n_samples < 3:
            raise RuntimeError(f"샘플 부족: {self.n_samples}개 (최소 3개 필요)")

        R_g2b = [T[:3, :3] for T in self._T_EB_list]
        t_g2b = [T[:3, 3:4] for T in self._T_EB_list]
        R_t2c = [T[:3, :3] for T in self._T_MC_list]
        t_t2c = [T[:3, 3:4] for T in self._T_MC_list]

        methods = [
            ("TSAI",       cv2.CALIB_HAND_EYE_TSAI),
            ("PARK",       cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD",     cv2.CALIB_HAND_EYE_HORAUD),
            ("ANDREFF",    cv2.CALIB_HAND_EYE_ANDREFF),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]

        best_T_EC = None
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

            T_CE = np.eye(4)
            T_CE[:3, :3] = R_c2e
            T_CE[:3, 3]  = t_c2e.ravel()

            if np.linalg.norm(T_CE[:3, 3]) < 1e-6:
                log.warning(f"[HandEye] {name}: translation≈0 (실패)")
                continue
            if abs(np.linalg.det(R_c2e) - 1.0) > 0.05:
                log.warning(f"[HandEye] {name}: det(R)={np.linalg.det(R_c2e):.3f} (비정상)")
                continue

            T_EC_cand = np.linalg.inv(T_CE)
            t_err, r_err = self._compute_residuals(T_EC_cand)
            log.info(
                f"[HandEye] {name}: t_err={t_err*1000:.2f}mm  r_err={r_err:.2f}°  "
                f"t_norm={np.linalg.norm(T_CE[:3,3])*1000:.1f}mm"
            )

            if t_err < best_t_err:
                best_t_err, best_r_err = t_err, r_err
                best_T_EC = T_EC_cand
                best_name  = name

        if best_T_EC is None:
            raise RuntimeError(
                "[HandEye] 모든 캘리브레이션 방법 실패.\n"
                "  → 포즈의 회전 다양성을 늘리세요 (최소 5개, 다양한 tilting 포함)."
            )

        if best_t_err * 1000 > 50.0:
            log.warning(
                f"[HandEye] 잔차 t={best_t_err*1000:.1f}mm 큼 — "
                "마커 감지 품질 또는 포즈 다양성 확인 필요"
            )

        self._T_EC = best_T_EC
        self._t_err_m, self._r_err_deg = float(best_t_err), float(best_r_err)
        log.info(
            f"[HandEye] 최적: {best_name}  "
            f"t_err={best_t_err*1000:.2f}mm  r_err={best_r_err:.2f}°  "
            f"T_EC t={self._T_EC[:3,3]*1000} mm"
        )
        return self._T_EC

    def _compute_residuals(self, T_EC: np.ndarray) -> tuple[float, float]:
        """
        T_MB 일관성 잔차 (mean_t_m, mean_r_deg).

        체인: T_MB = T_EB @ T_CE @ T_MC   (M → C → E → B)
        마커가 고정돼 있으면 모든 포즈에서 T_MB 가 동일해야 한다.
        """
        if self.n_samples < 2:
            return np.inf, np.inf
        T_CE = np.linalg.inv(T_EC)
        errs_t, errs_r = [], []
        for i in range(self.n_samples - 1):
            T_MB_i  = self._T_EB_list[i]     @ T_CE @ self._T_MC_list[i]
            T_MB_i1 = self._T_EB_list[i + 1] @ T_CE @ self._T_MC_list[i + 1]
            diff = np.linalg.inv(T_MB_i1) @ T_MB_i
            errs_t.append(np.linalg.norm(diff[:3, 3]))
            angle = np.arccos(np.clip((np.trace(diff[:3, :3]) - 1) / 2, -1.0, 1.0))
            errs_r.append(np.degrees(angle))
        return float(np.mean(errs_t)), float(np.mean(errs_r))

    def save_yaml(
        self,
        yaml_path: Path,
        sensor: str = "phoxi_s",
        method: str = "photoneo_a4rev23a",
    ) -> None:
        """
        T_EC를 YAML로 저장 (config/calibration/hand_eye_<sensor>.yaml 권장).

        형식:
          sensor, date, method, n_poses, T_E_C: {translation, rotation_quat, matrix}

        translation 단위: meters.
        """
        import datetime

        if self._T_EC is None:
            raise RuntimeError("calibrate()를 먼저 호출하세요.")
        if np.linalg.norm(self._T_EC[:3, 3]) * 1000 < 1.0:
            raise RuntimeError("T_EC translation≈0 — 캘리브레이션 결과가 유효하지 않습니다.")

        t = self._T_EC[:3, 3]
        q = _rot_to_quat(self._T_EC[:3, :3])
        mat = self._T_EC.tolist()

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
