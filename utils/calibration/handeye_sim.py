"""handeye_sim — hand-eye sim 검증의 **센서·Isaac 무관 공통 로직**.

GUI 하니스(`sim_harness/MMS_ext_calibration.py`)와 standalone 러너
(`scripts/sim/calib_handeye_sim.py`)가 이 모듈을 공유한다. 두 경로가 서로 다른
코드를 돌면 sim 검증의 의미가 없어지므로, 실제 판정 로직은 여기 한 곳에만 둔다.

여기 없는 것 (omni/pxr 의존 → 각 드라이버가 담당)
  · USD 보드 prim 생성·재질 바인딩   · 카메라 렌더·intrinsic 조회
  · 관절 구동·물리 스텝              · prim world transform 조회

규약
  · `T_EC` = E→C (`x_C = T_EC · x_E`), HandEyeCalibrator 반환과 동일
  · USD/Isaac 카메라는 광축 −Z·+Y up, OpenCV(solvePnP)는 +Z·+Y down
    → `T_FLIP` 으로 환산. T_EC 는 카메라가 출력측이라 **왼쪽곱**이다.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Callable, Sequence

import numpy as np

# ── 카메라 프레임 규약 (USD ↔ OpenCV) ────────────────────────────────────────
R_FLIP = np.diag([1.0, -1.0, -1.0])
T_FLIP = np.eye(4)
T_FLIP[:3, :3] = R_FLIP

#: 판정 임계 — 실물 기준(2026-04-29) t 3.55mm / r 1.30° 보다 여유를 둔 값
PASS_T_ERR_MM = 5.0
PASS_R_ERR_DEG = 2.0


@dataclass
class BoardConfig:
    """ChArUco 보드 — 실물 프리셋과 **동일해야 한다**(검출기·텍스처 공용).

    ★ 이 기본값은 `mms_artec/utils/calibration/artec_charuco_detector.py` 의
      `BOARD_PRESETS["spider_dense"]` 와 맞춰 둔 사본이다. 2026-09-16 에 실물
      기본 보드를 `spider`(5×3/20mm, 코너 8개) → `spider_dense`(7×5/12mm, 코너 24개)
      로 바꿨는데 여기가 5×3 에 남아 있어서, sim 과 real 이 **다른 보드**를 쓰게 됐다.
      sim 은 자기 텍스처를 자기 스펙으로 만들므로 혼자서는 잘 돌지만, 그러면
      **sim 으로 real 을 검증한다는 전제가 깨진다.**

      `utils/` 는 `mms_artec/` 를 import 하지 않는 계층 규약이라(그래서
      `make_board_spec` 이 `CharucoBoardSpec` 을 **인자로** 받는다) 여기서 직접
      프리셋을 끌어올 수 없다. 대신 양쪽을 다 import 하는 sim 하니스가
      `BoardConfig.from_spec()` 으로 실물 프리셋에서 만들어 쓴다.
    """
    squares_x: int = 7
    squares_y: int = 5
    square_len_mm: float = 12.0
    marker_len_mm: float = 9.0
    aruco_dict: str = "DICT_4X4_50"
    img_px: tuple[int, int] = (840, 600)     # 10 px/mm — 84×60mm
    thick_m: float = 0.006

    @classmethod
    def from_spec(cls, spec, *, px_per_mm: float = 10.0, thick_m: float = 0.006):
        """실물 `CharucoBoardSpec` → sim `BoardConfig`. 이게 정본 경로다.

        `aruco_dict` 는 이름이 필요하므로 cv2 상수에서 역인용한다.
        """
        import cv2
        name = next((n for n in dir(cv2.aruco)
                     if n.startswith("DICT_")
                     and getattr(cv2.aruco, n) == spec.aruco_dict), "DICT_4X4_50")
        return cls(
            squares_x=int(spec.squares_x), squares_y=int(spec.squares_y),
            square_len_mm=float(spec.square_length_mm),
            marker_len_mm=float(spec.marker_length_mm),
            aruco_dict=name,
            img_px=(int(round(spec.squares_x * spec.square_length_mm * px_per_mm)),
                    int(round(spec.squares_y * spec.square_length_mm * px_per_mm))),
            thick_m=thick_m,
        )

    @property
    def width_m(self) -> float:
        return self.squares_x * self.square_len_mm / 1000.0

    @property
    def height_m(self) -> float:
        return self.squares_y * self.square_len_mm / 1000.0


def _env_list(name: str, default: Sequence[float]) -> Sequence[float]:
    """쉼표 구분 실수 목록 env override. 예: MMS_CALIB_POLARS=0,15,30,45"""
    v = os.environ.get(name)
    if not v:
        return default
    return tuple(float(x) for x in v.replace(" ", "").split(",") if x)


@dataclass
class PoseConfig:
    """캘리브 자세 — 보드 위 반구에서 내려다본다.

    자세 수 = 1(nadir) + (polar>0 개수) × (azimuth 개수).
    기본 (0,15,30,45)×8방위 → **25개 생성 → 충돌 게이트 통과 15개**.

    hand-eye(AX=ZB)는 **자세 간 회전 다양성**이 있어야 잘 풀린다(권장: 15~25 자세,
    자세 간 회전 ≥30°). 아래 값은 sim 실측(2026-09-09)으로 고른 것이다.

    충돌·특이점 게이트를 통과해야 실제로 쓰이므로 **후보를 넉넉히 생성**한다.
    sim 실측(2026-09-09, 게이트 ON):

    | 구성 | 생성 | 제외 | 유효 | t_err | r_err |
    |---|---|---|---|---|---|
    | (0,12,22)×5 (구) | 11 | 0 | 11 | 1.10mm | 0.15° |
    | (0,15,30,45)×5 | 16 | 5 | 11 | 1.30mm | 0.15° |
    | (0,15,30)×6 | 13 | 3 | 10 | 0.93mm | 0.15° |
    | (0,15,30)×8 | 17 | 4 | 13 | 1.13mm | 0.16° |
    | **(0,15,30,45)×8** | **25** | 10 | **15** | **0.95mm** | **0.10°** |

    polar 45° 까지 넓히면 회전 다양성이 좋아지지만 자가·환경 충돌로 걸러지는 자세가
    늘어난다. 방위를 8개로 늘려 **걸러지고도 15개가 남도록** 후보를 확보했다.
    env 로 조정 가능(`MMS_CALIB_POLARS` 등).
    """
    distance_m: float = float(os.environ.get("MMS_CALIB_DIST", 0.25))
    polars_deg: Sequence[float] = field(
        default_factory=lambda: _env_list("MMS_CALIB_POLARS", (0.0, 15.0, 30.0, 45.0)))
    azis_deg: Sequence[float] = field(
        default_factory=lambda: _env_list("MMS_CALIB_AZIS",
                                  (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)))
    rolls_deg: Sequence[float] = field(
        default_factory=lambda: _env_list("MMS_CALIB_ROLLS", (-20.0, 0.0, 20.0)))
    dist_jitter: Sequence[float] = field(
        default_factory=lambda: _env_list("MMS_CALIB_JITTER", (-0.02, 0.0, 0.02)))
    #: look-at 기준 x축(이미지 up). 관절공간 구동에선 IK 가 분기를 고르므로
    #: 또아리와 무관하나, 이미지 방향 일관성을 위해 유지.
    up_hint: tuple[float, float, float] = (1.0, 0.0, 0.0)


def make_board_spec(cfg: BoardConfig, CharucoBoardSpec, cv2):
    """실물과 동일한 `CharucoBoardSpec` (mm 규약).

    mm 규약이라 solvePnP 가 내는 `T_MC` 도 mm 이고, HandEyeCalibrator 기대치와 맞는다.
    """
    return CharucoBoardSpec(
        squares_x=cfg.squares_x, squares_y=cfg.squares_y,
        square_length_mm=cfg.square_len_mm, marker_length_mm=cfg.marker_len_mm,
        aruco_dict=getattr(cv2.aruco, cfg.aruco_dict))


def write_board_texture(board, png_path: str, cfg: BoardConfig, cv2, log=print) -> str:
    """cv2 보드 → USD 텍스처 PNG. `marginSize=0` 이라 평면과 1:1 로 대응한다."""
    os.makedirs(os.path.dirname(png_path), exist_ok=True)
    img = board.generateImage(cfg.img_px, marginSize=0, borderBits=1)
    cv2.imwrite(png_path, img)
    log(f"[CALIB] ChArUco board image: {png_path} ({img.shape[1]}x{img.shape[0]})")
    return png_path


def build_calibration_poses(board_center, board_normal, T_EC_gt,
                            cfg: PoseConfig, generate_hemisphere_poses, log=print):
    """보드 위 반구 자세 목록. 자세 생성 자체는 `handeye_geometry` 가 한다."""
    poses = generate_hemisphere_poses(
        board_center, board_normal, T_EC_gt,
        distance_m=cfg.distance_m, polars_deg=list(cfg.polars_deg),
        azis_deg=list(cfg.azis_deg), rolls_deg=list(cfg.rolls_deg),
        dist_jitter=list(cfg.dist_jitter), up_hint=cfg.up_hint)
    log(f"[CALIB] 캘리브 자세: {len(poses)} (보드중심 {np.round(board_center,3).tolist()})")
    return poses


def solve_and_report(calibrator, T_EC_gt, ik_mode: str, out_dir: str,
                     rot_angle_deg: Callable, log=print) -> dict | None:
    """`calibrateHandEye` 로 T_EC 를 풀고 **GT 와 비교**한다 (sim 전용 검증).

    Returns
    -------
    dict | None : {T_EC_est, t_err_mm, r_err_deg, n_samples, passed}. 실패 시 None.
    """
    if calibrator.n_samples < 3:
        log(f"[CALIB][ERROR] 샘플 부족 ({calibrator.n_samples}). 최소 3. 자세/검출 확인.")
        return None

    log("\n[CALIB] ===== HAND-EYE 결과 (ground-truth 대비) =====")
    log(f"[CALIB] 유효 샘플: {calibrator.n_samples} | IK: {ik_mode}")
    log(f"[CALIB] GT  T_E_C  t(mm)={np.round(T_EC_gt[:3,3]*1000,2).tolist()}")

    try:
        # HandEyeCalibrator: T_EC = E→C, OpenCV 카메라 프레임, 단위 m
        T_EC_ocv = calibrator.calibrate()
    except Exception as exc:
        log(f"[CALIB][ERROR] HandEyeCalibrator.calibrate 실패: {exc}")
        return None

    T_EC_est = T_FLIP @ T_EC_ocv        # OpenCV cam → USD cam (왼쪽곱)
    t_err = float(np.linalg.norm(T_EC_est[:3, 3] - T_EC_gt[:3, 3]) * 1000.0)
    r_err = float(rot_angle_deg(T_EC_est[:3, :3], T_EC_gt[:3, :3]))
    passed = t_err < PASS_T_ERR_MM and r_err < PASS_R_ERR_DEG

    log(f"[CALIB] >>> t_err={t_err:.2f} mm  r_err={r_err:.2f}°")
    log(f"[CALIB]     EST t(mm)={np.round(T_EC_est[:3,3]*1000,2).tolist()}")
    log(f"[CALIB]     판정: {'PASS ✅' if passed else 'CHECK ⚠'}")

    os.makedirs(out_dir, exist_ok=True)
    np.savez(os.path.join(out_dir, "handeye_result.npz"),
             T_EC_gt=T_EC_gt, T_EC_est=T_EC_est, t_err_mm=t_err, r_err_deg=r_err,
             n_samples=calibrator.n_samples, ik_mode=ik_mode)
    log("[CALIB] ===== COMPLETE =====\n")

    return dict(T_EC_est=T_EC_est, t_err_mm=t_err, r_err_deg=r_err,
                n_samples=calibrator.n_samples, passed=passed)
