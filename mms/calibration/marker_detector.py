# mms/calibration/marker_detector.py
#
# Photoneo A4-REV-23A 마커보드 감지 및 T_M^S 추정.
#
# 감지 파이프라인
# --------------
# 1. PhoXi intensity 이미지에서 SimpleBlobDetector로 동심원 마커 감지 (2D)
# 2. 감지된 픽셀 좌표 → 조직화된 포인트클라우드에서 3D 좌표 추출
# 3. 알려진 보드 좌표(positions.txt) ↔ 감지된 3D 좌표 RANSAC 매칭
# 4. Kabsch SVD로 T_M^S 계산
#
# Coordinate convention (CLAUDE.md)
# ----------------------------------
# T_M^S : Marker frame → Sensor frame
#   x_S = T_M^S @ x_M

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger(__name__)

_DEFAULT_POSITIONS_FILE = Path(
    r"C:\Program Files\Photoneo\PhoXiControl-1.16.5"
    r"\MarkerPatterns\patterns_with_metadata\A4-REV-23A_positions.txt"
)


# ------------------------------------------------------------------
# 포지션 파일 로드
# ------------------------------------------------------------------

def _load_positions(path: Path) -> np.ndarray:
    """
    A4-REV-23A_positions.txt 파싱.

    실제 포맷 (공백 구분): <-1>  <x_mm>  <y_mm>  <z_mm>

    Returns
    -------
    (N, 3) float64, mm 단위, board frame (z=0 평면).
    """
    pts = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            # 첫 컬럼이 숫자(플래그 -1)이고 나머지가 x y z
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            pts.append([x, y, z])
    arr = np.array(pts, dtype=np.float64)
    log.info(f"[Marker] Loaded {len(arr)} board positions from {path}")
    if len(arr) == 0:
        raise RuntimeError(f"[Marker] 보드 좌표 로드 실패 (0개): {path}")
    return arr


# ------------------------------------------------------------------
# Kabsch 3D-3D rigid transform (SVD)
# ------------------------------------------------------------------

def _kabsch(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """
    Compute T such that dst ≈ T @ src (homogeneous).

    Parameters
    ----------
    src : (N, 3) — source points (board frame)
    dst : (N, 3) — destination points (sensor frame)

    Returns
    -------
    (4, 4) float64
    """
    c_src = src.mean(axis=0)
    c_dst = dst.mean(axis=0)
    A = src - c_src
    B = dst - c_dst
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = c_dst - R @ c_src
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ------------------------------------------------------------------
# RANSAC 3D-3D 매칭
# ------------------------------------------------------------------

def _ransac_match(
    board_pts: np.ndarray,
    sensor_pts: np.ndarray,
    dist_tol_mm: float = 5.0,
    n_iter: int = 500,
    min_inliers: int = 6,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    RANSAC으로 board_pts ↔ sensor_pts 최적 대응쌍 탐색.

    Parameters
    ----------
    board_pts  : (M, 3) 알려진 보드 좌표 (mm, board frame)
    sensor_pts : (N, 3) 감지된 3D 좌표 (mm, sensor frame)
    dist_tol_mm: 인라이어 판정 거리 허용오차 (mm)
    n_iter     : RANSAC 반복 횟수
    min_inliers: 최소 인라이어 수

    Returns
    -------
    (T_M_S, src_inliers, dst_inliers) or None
    """
    M = len(board_pts)
    N = len(sensor_pts)
    if N < 3 or M < 3:
        return None

    # 보드 쌍간 거리 테이블 (매칭 기준)
    board_dists = np.linalg.norm(
        board_pts[:, None, :] - board_pts[None, :, :], axis=-1
    )

    rng = np.random.default_rng(42)
    best_inliers = 0
    best_T = None
    best_src_idx = None
    best_dst_idx = None

    for _ in range(n_iter):
        # 감지된 점 중 3개 랜덤 선택
        if N < 3:
            break
        si = rng.choice(N, 3, replace=False)
        sq = sensor_pts[si]

        # 감지된 3점 간 쌍간 거리
        d01 = np.linalg.norm(sq[0] - sq[1])
        d02 = np.linalg.norm(sq[0] - sq[2])
        d12 = np.linalg.norm(sq[1] - sq[2])

        # 보드에서 거리가 일치하는 3점 쌍 탐색
        tol = dist_tol_mm
        cands = []
        for b0 in range(M):
            for b1 in range(M):
                if b1 == b0:
                    continue
                if abs(board_dists[b0, b1] - d01) > tol:
                    continue
                for b2 in range(M):
                    if b2 == b0 or b2 == b1:
                        continue
                    if (abs(board_dists[b0, b2] - d02) < tol and
                            abs(board_dists[b1, b2] - d12) < tol):
                        cands.append((b0, b1, b2))

        if not cands:
            continue

        for (b0, b1, b2) in cands:
            bp = board_pts[[b0, b1, b2]]
            T_cand = _kabsch(bp, sq)

            # 전체 보드 점을 sensor space로 투영 후 인라이어 계산
            board_h = np.hstack([board_pts, np.ones((M, 1))])
            proj = (T_cand @ board_h.T).T[:, :3]

            # 각 투영 점에 대해 가장 가까운 감지 점 거리
            diffs = np.linalg.norm(
                proj[:, None, :] - sensor_pts[None, :, :], axis=-1
            )  # (M, N)
            min_dists = diffs.min(axis=1)   # 각 보드 점의 최근접 거리
            inlier_mask = min_dists < dist_tol_mm
            n_inliers = inlier_mask.sum()

            if n_inliers > best_inliers:
                best_inliers = n_inliers
                best_T = T_cand
                best_src_idx = np.where(inlier_mask)[0]
                best_dst_idx = diffs[inlier_mask].argmin(axis=1)

    if best_inliers < min_inliers or best_T is None:
        log.warning(f"[Marker] RANSAC 실패: inliers={best_inliers} < {min_inliers}")
        return None

    # 인라이어로 최종 Kabsch refine
    T_final = _kabsch(board_pts[best_src_idx], sensor_pts[best_dst_idx])
    log.info(f"[Marker] RANSAC 성공: inliers={best_inliers}/{M}")
    return T_final, board_pts[best_src_idx], sensor_pts[best_dst_idx]


# ==================================================================
# A4REV23ADetector
# ==================================================================

class A4REV23ADetector:
    """
    Photoneo A4-REV-23A 마커보드 감지.

    PhoXi intensity 이미지 + 조직화된 포인트클라우드 →  T_M^S (4×4)

    Parameters
    ----------
    positions_file : Path, optional
        A4-REV-23A_positions.txt 경로. 기본값 = PhoXiControl 설치 경로.
    blob_min_area  : int
        블롭 최소 픽셀 면적 (센서 거리에 따라 조정).
    blob_max_area  : int
        블롭 최대 픽셀 면적.
    dist_tol_mm    : float
        RANSAC 인라이어 거리 허용오차 (mm).
    min_inliers    : int
        최소 인라이어 수 (기본 6 / 전체 15).
    """

    def __init__(
        self,
        positions_file: Optional[Path] = None,
        blob_min_area: int = 50,
        blob_max_area: int = 3000,
        dist_tol_mm: float = 5.0,
        min_inliers: int = 6,
    ) -> None:
        pos_path = positions_file or _DEFAULT_POSITIONS_FILE
        self._board_pts = _load_positions(pos_path)   # (15, 3) mm, board frame
        self._dist_tol = dist_tol_mm
        self._min_inliers = min_inliers

        # SimpleBlobDetector 설정 (Photoneo 동심원: 밝은 원 on 어두운 배경)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea       = True
        params.minArea            = blob_min_area
        params.maxArea            = blob_max_area
        params.filterByCircularity = True
        params.minCircularity     = 0.6
        params.filterByConvexity  = True
        params.minConvexity       = 0.7
        params.filterByInertia    = True
        params.minInertiaRatio    = 0.5
        # 밝은 원 / 어두운 원 모두 시도
        params.blobColor          = 255   # 밝은 원 우선
        self._detector = cv2.SimpleBlobDetector_create(params)

    # ------------------------------------------------------------------

    def detect(
        self,
        img_gray: np.ndarray,
        pts_organized_S: np.ndarray,
    ) -> Optional[np.ndarray]:
        """
        마커보드 감지 → T_M^S 반환.

        Parameters
        ----------
        img_gray : (H, W) uint8 또는 uint16 강도 이미지.
        pts_organized_S : (H, W, 3) float32, mm 단위, 센서(S) 프레임.
                          무효점 = (0,0,0).

        Returns
        -------
        T_M^S : (4, 4) float64 또는 None (감지 실패).
        """
        H, W = img_gray.shape[:2]

        # uint8 정규화
        if img_gray.dtype != np.uint8:
            img_u8 = cv2.normalize(
                img_gray.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
        else:
            img_u8 = img_gray

        # 밝은 원 감지
        keypoints = self._detector.detect(img_u8)

        # 감지 안되면 반전 이미지로 재시도 (어두운 원)
        if len(keypoints) < self._min_inliers:
            params2 = cv2.SimpleBlobDetector_Params()
            params2.filterByArea        = True
            params2.minArea             = self._detector.getParams().minArea if hasattr(self._detector, 'getParams') else 50
            params2.maxArea             = 3000
            params2.filterByCircularity = True
            params2.minCircularity      = 0.6
            params2.filterByConvexity   = True
            params2.minConvexity        = 0.7
            params2.filterByInertia     = True
            params2.minInertiaRatio     = 0.5
            params2.blobColor           = 0   # 어두운 원
            det2 = cv2.SimpleBlobDetector_create(params2)
            kp2 = det2.detect(img_u8)
            if len(kp2) > len(keypoints):
                keypoints = kp2

        if len(keypoints) < self._min_inliers:
            log.warning(f"[Marker] 블롭 감지 부족: {len(keypoints)}개 (최소 {self._min_inliers})")
            return None

        log.debug(f"[Marker] 블롭 감지: {len(keypoints)}개")

        # 2D → 3D 매핑 (조직화된 포인트클라우드 사용)
        sensor_pts_3d = []
        valid_kp = []
        for kp in keypoints:
            u, v = int(round(kp.pt[0])), int(round(kp.pt[1]))
            u = np.clip(u, 0, W - 1)
            v = np.clip(v, 0, H - 1)
            pt = pts_organized_S[v, u]
            if np.all(pt == 0.0) or not np.isfinite(pt).all():
                continue  # 무효점 제외
            sensor_pts_3d.append(pt.astype(np.float64))
            valid_kp.append(kp)

        sensor_pts_3d = np.array(sensor_pts_3d)
        if len(sensor_pts_3d) < self._min_inliers:
            log.warning(f"[Marker] 유효 3D 점 부족: {len(sensor_pts_3d)}개")
            return None

        log.debug(f"[Marker] 유효 3D 점: {len(sensor_pts_3d)}개")

        # ------------------------------------------------------------------
        # 이웃 필터링: 보드 마커는 서로 20~150mm 거리에 최소 2개 이웃이 있어야 함.
        # 배경의 고립된 원형 블롭(나사, 조명 반사 등)을 제거한다.
        # ------------------------------------------------------------------
        if len(sensor_pts_3d) > self._min_inliers * 2:
            pd = np.linalg.norm(
                sensor_pts_3d[:, None, :] - sensor_pts_3d[None, :, :], axis=-1
            )
            np.fill_diagonal(pd, np.inf)
            neighbor_mask = (pd > 20.0) & (pd < 150.0)
            neighbor_counts = neighbor_mask.sum(axis=1)
            keep = neighbor_counts >= 2
            if keep.sum() >= self._min_inliers:
                sensor_pts_3d = sensor_pts_3d[keep]
                log.debug(f"[Marker] 이웃 필터 후: {len(sensor_pts_3d)}개 "
                          f"(제거: {(~keep).sum()}개)")
            else:
                log.debug("[Marker] 이웃 필터 조건 미달, 전체 점 사용")

        # n_iter 자동 스케일: N개 중 3개 정답 뽑을 확률 기반, 99% 성공 보장
        N = len(sensor_pts_3d)
        M_board = len(self._board_pts)
        if N > M_board and M_board >= 3:
            p_triplet = float((M_board / N) ** 3)
            if p_triplet > 1e-12:
                n_iter = int(np.ceil(np.log(0.01) / np.log(1.0 - p_triplet)))
            else:
                n_iter = 8000
            n_iter = int(np.clip(n_iter, 500, 8000))
        else:
            n_iter = 500
        log.debug(f"[Marker] RANSAC 반복: {n_iter}회 (N={N}, M_board={M_board})")

        # RANSAC 3D-3D 매칭 → T_M^S
        result = _ransac_match(
            self._board_pts,
            sensor_pts_3d,
            dist_tol_mm=self._dist_tol,
            n_iter=n_iter,
            min_inliers=self._min_inliers,
        )
        if result is None:
            return None

        T_M_S, _, _ = result

        # 잔차 확인
        board_h = np.hstack([self._board_pts, np.ones((len(self._board_pts), 1))])
        proj = (T_M_S @ board_h.T).T[:, :3]
        diffs = np.linalg.norm(
            proj[:, None, :] - sensor_pts_3d[None, :, :], axis=-1
        ).min(axis=1)
        rmse = np.sqrt((diffs ** 2).mean())
        log.info(f"[Marker] T_M^S 추정 완료  RMSE={rmse:.2f} mm")

        return T_M_S

    def visualize(
        self,
        img_gray: np.ndarray,
        pts_organized_S: np.ndarray,
        T_M_S: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        감지 결과를 이미지에 그려서 반환 (디버그용).

        Returns
        -------
        (H, W, 3) uint8 BGR 이미지.
        """
        if img_gray.dtype != np.uint8:
            vis = cv2.normalize(
                img_gray.astype(np.float32), None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
        else:
            vis = img_gray.copy()
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        keypoints = self._detector.detect(img_gray if img_gray.dtype == np.uint8
                                           else vis[:, :, 0])
        vis = cv2.drawKeypoints(
            vis, keypoints, None,
            color=(0, 255, 0),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        if T_M_S is not None:
            # 보드 원점 투영 표시
            H, W = img_gray.shape[:2]
            origin = T_M_S[:3, 3]
            cv2.putText(
                vis, f"T_M^S OK  origin=({origin[0]:.0f},{origin[1]:.0f},{origin[2]:.0f})mm",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )
        return vis
