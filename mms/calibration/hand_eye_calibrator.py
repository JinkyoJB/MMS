# mms/calibration/hand_eye_calibrator.py
#
# Hand-Eye 캘리브레이션: T_E^S (End-Effector → Sensor) 추정.
#
# ----------------------
#   AX = XB  방정식:
#     A_i = T_E_B^(i+1) @ inv(T_E_B^i)  ... EE 간 상대 운동 (base frame)
#     B_i = T_M_S^(i+1) @ inv(T_M_S^i)  ... 마커 간 상대 운동 (sensor frame)
#     X   = T_E_S                         ... 구하는 변환
#
#   OpenCV calibrateHandEye는 내부적으로 AX=XB를 풀어줌.
#   입력: R_gripper2base, t_gripper2base  (= T_E^B 의 R, t)
#         R_target2cam,   t_target2cam    (= T_M^S 의 R, t)
#
# 저장 형식 (config/sensor_frames.yaml)
# --------------------------------------
#   T_E_S_phoxi:
#     translation: [tx, ty, tz]        # meters
#     rotation_quat: [qx, qy, qz, qw]

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import yaml

log = logging.getLogger(__name__)


# ------------------------------------------------------------------
# 쿼터니언 유틸
# ------------------------------------------------------------------

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


# ------------------------------------------------------------------
# HandEyeCalibrator
# ------------------------------------------------------------------

class HandEyeCalibrator:
    """
    Hand-Eye 캘리브레이션 (AX=XB, Eye-in-Hand 방식).

    샘플 수집 → calibrate() → save_yaml()

    Parameters
    ----------
    method : int
        cv2.CALIB_HAND_EYE_* 상수.
        기본 = cv2.CALIB_HAND_EYE_TSAI

    Notation
    --------
    T_E_B : (4,4)  EE → Base  (= robot FK 출력)
    T_M_S : (4,4)  Marker → Sensor  (= A4REV23ADetector.detect() 출력)
    T_E_S : (4,4)  EE → Sensor  (캘리브레이션 결과, 구하는 변환)
    """

    def __init__(self, method: int = cv2.CALIB_HAND_EYE_TSAI) -> None:
        self._method = method
        self._T_E_B_list: List[np.ndarray] = []   # (4,4) each
        self._T_M_S_list: List[np.ndarray] = []   # (4,4) each
        self._T_E_S: Optional[np.ndarray] = None

    # ------------------------------------------------------------------

    def add_sample(self, T_E_B: np.ndarray, T_M_S: np.ndarray) -> None:
        """
        캘리브레이션 샘플 추가.

        Parameters
        ----------
        T_E_B : (4,4)  EE-frame → Base-frame  (robot FK), translation in meters.
        T_M_S : (4,4)  Marker-frame → Sensor-frame  (marker detector),
                       translation in mm (marker_detector.py 출력 단위).
                       내부적으로 m 단위로 변환하여 저장한다.

        Note
        ----
        calibrateHandEye는 T_E_B, T_M_S 의 translation 단위가 일치해야 한다.
        T_E_B translation 은 meters, marker_detector 출력(T_M_S) translation 은 mm
        이므로 여기서 T_M_S[:3, 3] / 1000 으로 통일한다.
        """
        assert T_E_B.shape == (4, 4), "T_E_B must be (4,4)"
        assert T_M_S.shape == (4, 4), "T_M_S must be (4,4)"
        self._T_E_B_list.append(T_E_B.copy())
        T_M_S_m = T_M_S.copy()
        T_M_S_m[:3, 3] /= 1000.0   # mm → m  (T_E_B 단위와 통일)
        self._T_M_S_list.append(T_M_S_m)
        log.info(
            f"[HandEye] 샘플 추가 (총 {len(self._T_E_B_list)}개)  "
            f"T_M_S t_norm={np.linalg.norm(T_M_S_m[:3, 3]) * 1000:.1f}mm"
        )

    @property
    def n_samples(self) -> int:
        return len(self._T_E_B_list)

    # ------------------------------------------------------------------

    def calibrate(self) -> np.ndarray:
        """
        OpenCV calibrateHandEye로 T_E^S 계산.
        여러 방법을 시도하고 잔차가 가장 작은 결과를 반환한다.

        Returns
        -------
        T_E_S : (4,4) float64

        Raises
        ------
        RuntimeError
            모든 방법이 실패하거나 결과가 유효하지 않을 때.
        """
        if self.n_samples < 3:
            raise RuntimeError(
                f"샘플 부족: {self.n_samples}개 (최소 3개 필요)"
            )

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

            # 유효성 검사: translation이 0에 가까우면 실패로 간주
            t_norm = np.linalg.norm(T_S_E[:3, 3])
            if t_norm < 1e-6:
                log.warning(f"[HandEye] {name}: translation≈0 (실패)")
                continue

            # R이 올바른 회전행렬인지 확인
            det = np.linalg.det(R_c2e)
            if abs(det - 1.0) > 0.05:
                log.warning(f"[HandEye] {name}: det(R)={det:.3f} (비정상)")
                continue

            T_E_S_cand = np.linalg.inv(T_S_E)
            t_err, r_err = self._compute_residuals(T_E_S_cand)
            log.info(f"[HandEye] {name}: t_err={t_err*1000:.2f}mm  r_err={r_err:.2f}°  "
                     f"t_norm={t_norm*1000:.1f}mm")

            if t_err < best_t_err:
                best_t_err  = t_err
                best_r_err  = r_err
                best_T_E_S  = T_E_S_cand
                best_name   = name

        if best_T_E_S is None:
            raise RuntimeError(
                "[HandEye] 모든 캘리브레이션 방법 실패.\n"
                "  → 로봇 포즈의 회전 다양성을 늘려주세요 (센서 tilting 포함).\n"
                "  → 최소 5개 이상의 서로 다른 방향 포즈 필요."
            )

        # 결과 sanity check: 잔차가 비정상적으로 크면 경고
        if best_t_err * 1000 > 50.0:   # 50mm 이상이면 경고
            log.warning(
                f"[HandEye] 잔차 t={best_t_err*1000:.1f}mm 가 큽니다. "
                "마커 감지 품질 또는 포즈 다양성을 확인하세요."
            )

        self._T_E_S = best_T_E_S
        log.info(f"[HandEye] 최적 방법: {best_name}  "
                 f"t_err={best_t_err*1000:.2f}mm  r_err={best_r_err:.2f}°")
        log.info(f"  T_E_S translation (mm): {self._T_E_S[:3, 3] * 1000}")
        return self._T_E_S

    # ------------------------------------------------------------------

    def _compute_residuals(self, T_E_S: np.ndarray) -> tuple[float, float]:
        """AX=XB 잔차 계산. (mean_t_m, mean_r_deg) 반환."""
        if self.n_samples < 2:
            return np.inf, np.inf
        errs_t, errs_r = [], []
        for i in range(self.n_samples - 1):
            A = self._T_E_B_list[i + 1] @ np.linalg.inv(self._T_E_B_list[i])
            B = self._T_M_S_list[i + 1] @ np.linalg.inv(self._T_M_S_list[i])
            diff = np.linalg.inv(A @ T_E_S) @ (T_E_S @ B)
            errs_t.append(np.linalg.norm(diff[:3, 3]))
            angle = np.arccos(np.clip((np.trace(diff[:3, :3]) - 1) / 2, -1.0, 1.0))
            errs_r.append(np.degrees(angle))
        return float(np.mean(errs_t)), float(np.mean(errs_r))

    # ------------------------------------------------------------------

    def save_yaml(
        self,
        yaml_path: Path,
        key: str = "T_E_S_phoxi",
    ) -> None:
        """
        T_E^S를 config YAML 파일에 저장 (기존 키 업데이트 또는 추가).

        단위: translation은 meters.

        Parameters
        ----------
        yaml_path : Path
            config/sensor_frames.yaml 경로.
        key : str
            저장할 YAML 키 이름.
        """
        if self._T_E_S is None:
            raise RuntimeError("calibrate()를 먼저 호출하세요.")
        t_norm_mm = np.linalg.norm(self._T_E_S[:3, 3]) * 1000
        if t_norm_mm < 1.0:
            raise RuntimeError(
                f"[HandEye] T_E_S translation≈0 — 캘리브레이션 결과가 유효하지 않습니다. "
                "포즈 다양성을 늘리고 다시 시도하세요."
            )

        R = self._T_E_S[:3, :3]
        t = self._T_E_S[:3, 3]        # meters 단위 (add_sample에서 T_M_S mm→m 변환 완료)
        q = _rot_to_quat(R)

        # 기존 YAML 로드
        yaml_path = Path(yaml_path)
        if yaml_path.exists():
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        data[key] = {
            "translation": [float(v) for v in t],
            "rotation_quat": [float(v) for v in q],
        }

        with open(yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=None, allow_unicode=True)

        log.info(f"[HandEye] 저장 완료: {yaml_path}  key={key}")
        print(f"[HandEye] 저장: {yaml_path}  key={key}")
        print(f"  translation : {t.tolist()}")
        print(f"  rotation_quat: {q.tolist()}")

    @property
    def T_E_S(self) -> Optional[np.ndarray]:
        return self._T_E_S


# ==================================================================
# 자동 캘리브레이션 스크립트
# ==================================================================

if __name__ == "__main__":
    import time
    from scipy.spatial.transform import Rotation as ScipyR

    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_PROJECT_ROOT))

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    from mms.robot.xarm_interface import XArmInterface
    from mms.sensor.phoxi_client import PhoxiClient, PhoxiConfig
    from mms.calibration.marker_detector import A4REV23ADetector

    # ------------------------------------------------------------------
    # 설정
    # ------------------------------------------------------------------
    ROBOT_IP       = "192.168.1.210"
    POSES_YAML     = _PROJECT_ROOT / "config" / "calibration_poses.yaml"
    SENSOR_YAML    = str(_PROJECT_ROOT / "config" / "sensor_frames.yaml")
    OUTPUT_YAML    = _PROJECT_ROOT / "config" / "sensor_frames.yaml"
    OUTPUT_KEY     = "T_E_S_phoxi"
    MOVE_SPEED_DEG = 10      # deg/s — 천천히 이동
    SETTLE_TIME_S  = 0.5     # 이동 완료 후 정착 대기 (진동 감쇠)

    # ------------------------------------------------------------------
    # 캘리브레이션 포즈 로드
    # ------------------------------------------------------------------
    with open(POSES_YAML, "r", encoding="utf-8") as f:
        poses_data = yaml.safe_load(f)

    pose_list = poses_data.get("poses", [])
    if not pose_list:
        raise RuntimeError(f"[HandEye] {POSES_YAML} 에 poses가 없습니다.")

    print("=" * 60)
    print("  Hand-Eye Calibration  (A4-REV-23A, Eye-in-Hand, 자동)")
    print("=" * 60)
    print(f"  포즈 수     : {len(pose_list)}")
    print(f"  포즈 파일   : {POSES_YAML}")
    print(f"  출력 키     : {OUTPUT_KEY}")
    print(f"  이동 속도   : {MOVE_SPEED_DEG} deg/s")

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    robot = XArmInterface(ip=ROBOT_IP)

    cfg = PhoxiConfig(
        sensor_frames_yaml=SENSOR_YAML,
        T_E_S_key=OUTPUT_KEY,
        serial_number="SEA-023",
        trigger_timeout_s=30.0,   # PhoXi S Gen3 고품질 스캔은 최대 ~25초 소요 가능
    )
    sensor = PhoxiClient(cfg)
    sensor.initialize()

    detector  = A4REV23ADetector()
    calibrator = HandEyeCalibrator(method=cv2.CALIB_HAND_EYE_TSAI)

    # 디버그 이미지 저장 폴더
    DEBUG_DIR = _PROJECT_ROOT / "debug_calib"
    DEBUG_DIR.mkdir(exist_ok=True)
    print(f"  디버그 이미지 → {DEBUG_DIR}")

    # ------------------------------------------------------------------
    # 시작 전 확인
    # ------------------------------------------------------------------
    print("\n로봇이 각 캘리브레이션 포즈로 자동 이동합니다.")
    input("준비되면 Enter를 누르세요... ")

    # ------------------------------------------------------------------
    # 포즈 순회 → 캡처 → 마커 감지 → 샘플 수집
    # ------------------------------------------------------------------
    failed_poses = []

    try:
        for idx, pose_entry in enumerate(pose_list):
            name = pose_entry.get("name", f"pose_{idx}")

            # ── 포즈 포맷 분기: joints 또는 ee_pose ──────────────────────
            if "joints" in pose_entry:
                # 기존 포맷: joint angles (degrees, 7축)
                joints_deg = pose_entry["joints"]
                print(f"\n[{idx+1}/{len(pose_list)}] {name}  joints={joints_deg}")

            elif "ee_pose" in pose_entry:
                # 신규 포맷: [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]
                ee = pose_entry["ee_pose"]
                ee_pose_rad = [ee[0], ee[1], ee[2],
                               np.radians(ee[3]), np.radians(ee[4]), np.radians(ee[5])]
                print(f"\n[{idx+1}/{len(pose_list)}] {name}  "
                      f"ee=[{ee[0]:.1f},{ee[1]:.1f},{ee[2]:.1f}mm  "
                      f"rpy={ee[3]:.1f},{ee[4]:.1f},{ee[5]:.1f}°]")

                # IK로 joint angles 계산
                code_ik, ik_joints = robot.arm.get_inverse_kinematics(
                    pose=ee_pose_rad,
                    input_is_radian=True,
                    return_is_radian=False,
                )
                if code_ik != 0:
                    print(f"  [!] IK 실패 (code={code_ik}) — 건너뜀")
                    failed_poses.append(name)
                    continue
                joints_deg = ik_joints[:7]
                print(f"  IK joints={[round(j,1) for j in joints_deg]}")

            else:
                print(f"\n[{idx+1}/{len(pose_list)}] {name}  [!] joints 또는 ee_pose 키 없음 — 건너뜀")
                failed_poses.append(name)
                continue

            # 로봇 이동
            code = robot.arm.set_servo_angle(
                angle=joints_deg,
                speed=MOVE_SPEED_DEG,
                is_radian=False,
                wait=True,
            )
            if code != 0:
                print(f"  [!] 이동 실패 (code={code}) — 건너뜀")
                failed_poses.append(name)
                continue

            # 진동 정착 대기
            time.sleep(SETTLE_TIME_S)

            # FK로 현재 T_E_B 획득
            pose6 = robot.get_pose(is_radian=False)   # [x,y,z, roll,pitch,yaw] mm, deg
            rpy_rad = np.radians(pose6[3:6])
            R_E_B = ScipyR.from_euler("xyz", rpy_rad).as_matrix()
            T_E_B = np.eye(4)
            T_E_B[:3, :3] = R_E_B
            T_E_B[:3, 3]  = pose6[:3] / 1000.0   # mm → m

            print(f"  TCP  x={pose6[0]:.1f}  y={pose6[1]:.1f}  z={pose6[2]:.1f} mm")

            # 캡처
            try:
                frame = sensor.capture_frame(
                    ee_pose_mat_B=T_E_B,
                    frame_id=idx,
                    timestamp=time.perf_counter(),
                )
            except Exception as cap_err:
                print(f"  [!] 캡처 예외: {cap_err} — 건너뜀")
                failed_poses.append(name)
                continue
            if frame is None:
                print("  [!] 캡처 실패 — 건너뜀")
                failed_poses.append(name)
                continue

            # 마커 감지
            img_gray = sensor._last_intensity
            pts_org  = sensor._last_organized_pts

            if img_gray is None or pts_org is None:
                print("  [!] 강도 이미지/조직화 포인트클라우드 없음 — 건너뜀")
                failed_poses.append(name)
                continue

            # --- 디버그: 강도 이미지 저장 ---
            cv2.imwrite(str(DEBUG_DIR / f"{name}_intensity.png"), img_gray)

            # --- 디버그: 블롭 감지 상세 로그 ---
            _dbg_params = cv2.SimpleBlobDetector_Params()
            _dbg_params.filterByArea        = True
            _dbg_params.minArea             = detector._detector.getParams().minArea if hasattr(detector._detector, 'getParams') else 50
            _dbg_params.maxArea             = 3000
            _dbg_params.filterByCircularity = True
            _dbg_params.minCircularity      = 0.6
            _dbg_params.filterByConvexity   = True
            _dbg_params.minConvexity        = 0.7
            _dbg_params.filterByInertia     = True
            _dbg_params.minInertiaRatio     = 0.5
            for _bc, _label in [(255, "bright"), (0, "dark")]:
                _dbg_params.blobColor = _bc
                _dbg_det = cv2.SimpleBlobDetector_create(_dbg_params)
                _kps = _dbg_det.detect(img_gray)
                if _kps:
                    areas = [p.size**2 * 3.14 / 4 for p in _kps]
                    print(f"    블롭({_label}): {len(_kps)}개  "
                          f"size_range=[{min(p.size for p in _kps):.1f}~{max(p.size for p in _kps):.1f}px]  "
                          f"area_range=[{min(areas):.0f}~{max(areas):.0f}px²]")
                    # 시각화 저장
                    _vis = cv2.drawKeypoints(
                        cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR),
                        _kps, None,
                        color=(0, 255, 0) if _bc == 255 else (0, 0, 255),
                        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
                    )
                    cv2.imwrite(str(DEBUG_DIR / f"{name}_blobs_{_label}.png"), _vis)
                else:
                    print(f"    블롭({_label}): 0개 감지")

            T_M_S = detector.detect(img_gray, pts_org)
            if T_M_S is None:
                print("  [!] 마커 감지 실패 — 건너뜀")
                print(f"      → {DEBUG_DIR / (name + '_intensity.png')} 확인하세요")
                failed_poses.append(name)
                continue

            calibrator.add_sample(T_E_B, T_M_S)
            t_mm = T_M_S[:3, 3]
            # 성공 시각화 저장
            vis = detector.visualize(img_gray, pts_org, T_M_S)
            cv2.imwrite(str(DEBUG_DIR / f"{name}_detected.png"), vis)
            print(
                f"  ✓ 샘플 {calibrator.n_samples} 수집  "
                f"marker=({t_mm[0]:.0f},{t_mm[1]:.0f},{t_mm[2]:.0f})mm"
            )

    finally:
        sensor.shutdown()
        robot.disconnect()

    # ------------------------------------------------------------------
    # 결과 보고
    # ------------------------------------------------------------------
    print(f"\n{'='*60}")
    print(f"  수집 완료: {calibrator.n_samples}/{len(pose_list)} 샘플")
    if failed_poses:
        print(f"  실패 포즈: {failed_poses}")

    # ------------------------------------------------------------------
    # 캘리브레이션 & 저장
    # ------------------------------------------------------------------
    MIN_SAMPLES = 3
    if calibrator.n_samples >= MIN_SAMPLES:
        print(f"\n캘리브레이션 실행 ({calibrator.n_samples}샘플) ...")
        T_E_S = calibrator.calibrate()
        calibrator.save_yaml(OUTPUT_YAML, key=OUTPUT_KEY)
        print("\n완료. config/sensor_frames.yaml 업데이트됨.")
    else:
        print(
            f"\n샘플 부족 ({calibrator.n_samples}/{MIN_SAMPLES}) — "
            f"calibration_poses.yaml의 포즈에서 마커가 보이는지 확인하세요."
        )
