# mms/utils/calibration/artec_charuco_detector.py
#
# Artec Spider hand-eye 캘리브용 ChArUco 검출기.
#
# 입력  : Artec FrameMeshHandle (vertices N×3 mm, uv N×2 [0..1], image H×W×3)
# 출력  : T_M_C (Marker → Camera, 4×4, translation mm)
#
# 절차
# ----
# 1. texture image 에서 ChArUco corner pixel 검출 (cv2.aruco)
# 2. 각 corner pixel 에 대해 vertices 중 UV 가 가장 가까운 점의 3D 좌표 (C 프레임 mm) 획득
# 3. {board_pts (M)} ↔ {camera_pts (C)} 의 3D-3D Procrustes 로 R, t 산출
# 4. T_M_C = [R | t] (mm)
#
# 카메라 intrinsics 가 필요 없는 게 핵심 (UV 매핑 = SDK 가 이미 calibrated 한 결과).

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# 보드 사양
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CharucoBoardSpec:
    """ChArUco 보드 물리 사양 (인쇄 후 실측 보정 권장)."""
    squares_x: int = 7
    squares_y: int = 5
    square_length_mm: float = 30.0
    marker_length_mm: float = 22.0
    aruco_dict: int = aruco.DICT_5X5_100

    def make_board(self) -> aruco.CharucoBoard:
        d = aruco.getPredefinedDictionary(self.aruco_dict)
        # 새 OpenCV API: CharucoBoard((sx,sy), sq, mk, dict_)
        try:
            return aruco.CharucoBoard(
                (self.squares_x, self.squares_y),
                self.square_length_mm,
                self.marker_length_mm,
                d,
            )
        except TypeError:
            # 구 API: CharucoBoard_create
            return aruco.CharucoBoard_create(
                self.squares_x, self.squares_y,
                self.square_length_mm, self.marker_length_mm, d,
            )


# ─────────────────────────────────────────────────────────────────────────────
# 검출 결과
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CharucoDetection:
    n_corners: int                    # 검출된 ChArUco corner 수
    corner_ids: np.ndarray            # (n,) int — 보드 상의 corner index
    pixels: np.ndarray                # (n, 2) float — texture image 픽셀 좌표
    pts_M_mm: np.ndarray              # (n, 3) — 마커 프레임 좌표 (z=0)
    pts_C_mm: np.ndarray              # (n, 3) — 카메라 프레임 좌표 (UV → vertex 매핑 결과)
    T_MC: np.ndarray                  # (4, 4) — Procrustes 결과, translation mm
    procrustes_rmse_mm: float         # 정합 잔차 (3D-3D)
    debug_image: Optional[np.ndarray] = None    # (H, W, 3) BGR — 디버그용 시각화


# ─────────────────────────────────────────────────────────────────────────────
# 검출기
# ─────────────────────────────────────────────────────────────────────────────

class ArtecCharucoDetector:
    """
    Artec FrameMeshHandle (또는 호환 dataclass) 에서 ChArUco 검출 → T_M_C 산출.
    """

    def __init__(
        self,
        board_spec: Optional[CharucoBoardSpec] = None,
        intrinsic: Optional[dict] = None,
    ):
        """
        Parameters
        ----------
        board_spec : CharucoBoardSpec
        intrinsic : dict, optional
            {"K": (3,3) list, "dist": (5,) list, "image_size": [W,H]} — 있으면
            solvePnP 경로 사용 (UV→3D 우회, 더 정확).
            없으면 UV→3D Procrustes (legacy fallback).
        """
        self.spec = board_spec or CharucoBoardSpec()
        self.board = self.spec.make_board()
        self._dict = aruco.getPredefinedDictionary(self.spec.aruco_dict)

        # Camera intrinsics (solvePnP 용 — 옵션)
        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        if intrinsic is not None:
            self.K = np.asarray(intrinsic["K"], dtype=np.float64).reshape(3, 3)
            self.dist = np.asarray(
                intrinsic.get("dist", [0, 0, 0, 0, 0]), dtype=np.float64
            ).reshape(-1)

        # 신 API (OpenCV ≥ 4.7) — CharucoDetector 한 번에 처리
        # 구 API (≤ 4.6) — detectMarkers + interpolateCornersCharuco
        # ArucoDetector 는 ArUco 4-corner fallback 에 항상 필요하므로
        # CharucoDetector 와 무관하게 가능하면 만든다.
        self._charuco_det = None
        self._aruco_det = None
        if hasattr(aruco, "CharucoDetector"):
            try:
                self._charuco_det = aruco.CharucoDetector(self.board)
            except Exception:
                self._charuco_det = None
        if hasattr(aruco, "ArucoDetector"):
            try:
                self._aruco_det = aruco.ArucoDetector(
                    self._dict, aruco.DetectorParameters()
                )
            except Exception:
                self._aruco_det = None

    # ── 검출 (low-level) ──────────────────────────────────────────────

    def detect_aruco_only(
        self,
        image_rgb: np.ndarray,
    ) -> Tuple[Optional[list], Optional[np.ndarray]]:
        """
        ArUco 마커만 검출 (디버그용 — ChArUco 실패 시 어떤 마커가 검출됐는지 확인).
        """
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)
        if self._aruco_det is not None:
            mc, mi, _ = self._aruco_det.detectMarkers(gray)
        elif hasattr(aruco, "detectMarkers"):
            mc, mi, _ = aruco.detectMarkers(gray, self._dict)
        else:
            return None, None
        return mc, mi

    def detect_charuco_corners(
        self,
        image_rgb: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        """
        텍스처 이미지에서 ChArUco corner pixel 검출.

        Returns
        -------
        corners : (n, 1, 2) float32 픽셀 좌표 또는 None
        ids     : (n, 1) int32 corner ID 또는 None
        n_aruco : 감지된 ArUco 마커 수 (디버그용)
        """
        gray = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2GRAY)

        # 신 API: CharucoDetector 가 ChArUco corner 까지 한 번에
        if self._charuco_det is not None:
            ch_corners, ch_ids, marker_corners, marker_ids = \
                self._charuco_det.detectBoard(gray)
            n_aruco = 0 if marker_ids is None else int(len(marker_ids))
            if ch_corners is None or ch_ids is None or len(ch_ids) == 0:
                return None, None, n_aruco
            return ch_corners, ch_ids, n_aruco

        # 구 API
        if self._aruco_det is not None:
            marker_corners, marker_ids, _ = self._aruco_det.detectMarkers(gray)
        else:
            marker_corners, marker_ids, _ = aruco.detectMarkers(gray, self._dict)

        if marker_ids is None or len(marker_ids) == 0:
            return None, None, 0

        n_aruco = int(len(marker_ids))

        if not hasattr(aruco, "interpolateCornersCharuco"):
            # 구 API 도 없음 — OpenCV 빌드 문제
            return None, None, n_aruco

        ret, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, self.board,
        )
        if ret is None or ret == 0 or ch_corners is None:
            return None, None, n_aruco

        return ch_corners, ch_ids, n_aruco

    # ── UV → 3D 매핑 ──────────────────────────────────────────────────

    @staticmethod
    def _uv_to_3d(
        target_pixels: np.ndarray,    # (n, 2)
        vertices_mm: np.ndarray,      # (N, 3)
        uv: np.ndarray,               # (N, 2) ∈ [0,1]
        image_wh: Tuple[int, int],
        max_dist_px: float = 6.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        각 target pixel 에 대해 UV 가 가장 가까운 vertex 의 3D 좌표를 반환.

        Returns
        -------
        pts_3d : (n, 3) float64 mm    — 카메라 프레임
        valid  : (n,) bool            — max_dist_px 이내에 매칭된 픽셀만 True
        """
        W, H = image_wh
        # vertex UV → 픽셀
        pix_uv = uv.astype(np.float64).copy()
        pix_uv[:, 0] *= float(W)
        pix_uv[:, 1] *= float(H)
        # KDTree 가 있으면 빠르지만 의존성 추가 회피 — broadcasting 으로 충분
        # vertices 가 너무 많으면 chunking
        pts_3d = np.zeros((len(target_pixels), 3), dtype=np.float64)
        valid = np.zeros(len(target_pixels), dtype=bool)
        for i, (u, v) in enumerate(target_pixels):
            d2 = (pix_uv[:, 0] - u) ** 2 + (pix_uv[:, 1] - v) ** 2
            j = int(np.argmin(d2))
            if d2[j] <= max_dist_px * max_dist_px:
                pts_3d[i] = vertices_mm[j].astype(np.float64)
                valid[i] = True
        return pts_3d, valid

    # ── Procrustes (Kabsch) ───────────────────────────────────────────

    @staticmethod
    def _procrustes(
        pts_M: np.ndarray,    # (n, 3) mm — 마커 프레임
        pts_C: np.ndarray,    # (n, 3) mm — 카메라 프레임
    ) -> Tuple[np.ndarray, float]:
        """
        p_C = R @ p_M + t  를 만족하는 SE(3) 산출.

        Returns
        -------
        T_MC : (4, 4) — translation mm
        rmse : float — 3D 잔차 (mm)
        """
        assert pts_M.shape == pts_C.shape and pts_M.shape[1] == 3
        n = pts_M.shape[0]
        if n < 3:
            raise ValueError(f"Procrustes needs ≥3 points, got {n}")

        c_M = pts_M.mean(axis=0)
        c_C = pts_C.mean(axis=0)
        X = pts_M - c_M
        Y = pts_C - c_C

        H = X.T @ Y
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T
        t = c_C - R @ c_M

        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t

        # rmse
        residuals = (pts_M @ R.T + t) - pts_C
        rmse = float(np.sqrt((residuals ** 2).sum(axis=1).mean()))
        return T, rmse

    # ── 보드 메타 (마커 ID → 4 corner 좌표) ──────────────────────────

    def _board_marker_points(self) -> Tuple[np.ndarray, list]:
        """
        보드의 ArUco 마커 ID 리스트와 각 마커의 4 corner 보드프레임 좌표 (mm, z=0).

        Returns
        -------
        ids   : (N_markers,) int — 보드에 사용된 마커 ID
        objs  : list of (4, 3) float — 같은 길이, 각 마커의 4 corner 보드 프레임 좌표
        """
        # OpenCV ≥ 4.7
        if hasattr(self.board, "getIds"):
            ids = np.asarray(self.board.getIds(), dtype=np.int64).reshape(-1)
            objs = [np.asarray(p, dtype=np.float64).reshape(4, 3)
                    for p in self.board.getObjPoints()]
        else:
            ids = np.asarray(self.board.ids, dtype=np.int64).reshape(-1)
            objs = [np.asarray(p, dtype=np.float64).reshape(4, 3)
                    for p in self.board.objPoints]
        return ids, objs

    # ── solvePnP path (intrinsics 있을 때, 권장) ─────────────────────

    def _detect_pnp(
        self,
        image_rgb: np.ndarray,
        min_pts: int = 6,
        draw_debug: bool = True,
        verbose: bool = True,
    ) -> Optional[CharucoDetection]:
        """
        2D pixel + 3D board 좌표 → cv2.solvePnP → T_M_C.
        ChArUco corner (sub-pixel) + ArUco 4-corner 모두 사용.
        UV→3D 매핑이 없어 양자화 오차 없음.
        """
        H, W = image_rgb.shape[:2]

        ch_corners, ch_ids, n_aruco = self.detect_charuco_corners(image_rgb)
        chessboard_corners = getattr(self.board, "chessboardCorners", None)
        if chessboard_corners is None:
            chessboard_corners = self.board.getChessboardCorners()
        chessboard_corners = np.asarray(chessboard_corners, dtype=np.float64).reshape(-1, 3)

        pixels = []
        pts_M = []

        if ch_corners is not None and ch_ids is not None and len(ch_ids) > 0:
            n_ch = int(len(ch_ids))
            ch_pix = ch_corners.reshape(n_ch, 2).astype(np.float64)
            ch_id_flat = ch_ids.reshape(n_ch).astype(np.int64)
            for k in range(n_ch):
                pixels.append(ch_pix[k])
                pts_M.append(chessboard_corners[ch_id_flat[k]])

        # ArUco 4-corner — 보강
        marker_corners, marker_ids = self.detect_aruco_only(image_rgb)
        if marker_ids is not None and len(marker_ids) > 0:
            n_aruco = int(len(marker_ids))
            board_ids, board_objs = self._board_marker_points()
            id_to_obj = {int(bi): bo for bi, bo in zip(board_ids, board_objs)}
            for i, mid in enumerate(marker_ids.reshape(-1).tolist()):
                obj = id_to_obj.get(int(mid))
                if obj is None:
                    continue
                pix4 = np.asarray(marker_corners[i], dtype=np.float64).reshape(4, 2)
                for j in range(4):
                    pixels.append(pix4[j])
                    pts_M.append(obj[j])

        if len(pixels) < min_pts:
            if verbose:
                print(f"    [pnp] 점 부족 {len(pixels)} < {min_pts}")
            return None

        pixels = np.asarray(pixels, dtype=np.float64)
        pts_M_arr = np.asarray(pts_M, dtype=np.float64)

        # solvePnP
        ok, rvec, tvec = cv2.solvePnP(
            pts_M_arr, pixels, self.K, self.dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            if verbose:
                print("    [pnp] solvePnP 실패")
            return None

        # LM refinement
        rvec, tvec = cv2.solvePnPRefineLM(
            pts_M_arr, pixels, self.K, self.dist, rvec, tvec,
        )

        R, _ = cv2.Rodrigues(rvec)
        T_MC = np.eye(4, dtype=np.float64)
        T_MC[:3, :3] = R
        T_MC[:3, 3] = tvec.flatten()    # mm (pts_M 이 mm 라)

        # 재투영 오차 (px)
        proj, _ = cv2.projectPoints(pts_M_arr, rvec, tvec, self.K, self.dist)
        proj = proj.reshape(-1, 2)
        reproj_rmse_px = float(np.sqrt(((proj - pixels) ** 2).sum(axis=1).mean()))

        if verbose:
            print(f"    [pnp] pts={len(pixels)}  reproj rmse={reproj_rmse_px:.2f}px")

        debug = None
        if draw_debug:
            vis = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
            for (u, v) in pixels.astype(int):
                cv2.circle(vis, (int(u), int(v)), 5, (0, 255, 0), 1)
            for (u, v) in proj.astype(int):
                cv2.drawMarker(vis, (int(u), int(v)), (0, 0, 255),
                               markerType=cv2.MARKER_CROSS, markerSize=8, thickness=1)
            cv2.putText(
                vis,
                f"PnP pts={len(pixels)} reproj={reproj_rmse_px:.2f}px",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )
            debug = vis

        return CharucoDetection(
            n_corners=int(len(pixels)),
            corner_ids=np.arange(len(pixels), dtype=np.int64),
            pixels=pixels,
            pts_M_mm=pts_M_arr,
            pts_C_mm=np.zeros_like(pts_M_arr),    # PnP 경로에선 미사용
            T_MC=T_MC,
            procrustes_rmse_mm=reproj_rmse_px,    # PnP 에선 px 단위 (이름만 historical)
            debug_image=debug,
        )

    # ── 메인 진입점 ───────────────────────────────────────────────────

    def detect(
        self,
        image_rgb: np.ndarray,        # (H, W, 3) uint8
        vertices_mm: Optional[np.ndarray] = None,    # (N, 3) — UV→3D path 에서만 필요
        uv: Optional[np.ndarray] = None,             # (N, 2) — 동일
        max_uv_match_px: Optional[float] = None,
        min_corners: int = 6,
        draw_debug: bool = True,
        verbose: bool = True,
    ) -> Optional[CharucoDetection]:
        """
        한 프레임에서 T_M_C 산출. 실패 시 None.

        검출 전략:
        1) ChArUco corner (sub-pixel chess corner) 검출
        2) ArUco 마커 4개 corner 도 함께 사용 (보강)
        3) 둘 합쳐서 UV→3D 매핑 → Procrustes
        ChArUco 가 0개 나와도 ArUco 마커 ≥2개면 24-32 점으로 Procrustes 가능.
        """
        if image_rgb is None:
            return None

        # intrinsics 있으면 PnP 경로 (정확도 ↑)
        if self.K is not None:
            return self._detect_pnp(
                image_rgb, min_pts=min_corners,
                draw_debug=draw_debug, verbose=verbose,
            )

        # legacy: UV → 3D Procrustes (intrinsics 없을 때)
        if vertices_mm is None or uv is None:
            return None
        H, W = image_rgb.shape[:2]

        # max_uv_match_px 자동 — UV 정점이 이미지 평면에 펼쳐졌을 때
        # 평균 정점 간격의 ~3배 (보수적). 정점 적으면 임계값 커짐.
        if max_uv_match_px is None:
            n_v = max(int(len(vertices_mm)), 1)
            spacing = float(np.sqrt(H * W / n_v))
            max_uv_match_px = float(np.clip(spacing * 3.0, 8.0, 40.0))

        # 1) ChArUco corner
        ch_corners, ch_ids, n_aruco = self.detect_charuco_corners(image_rgb)

        # 보드 chessboard corner 좌표 (마커 프레임)
        chessboard_corners = getattr(self.board, "chessboardCorners", None)
        if chessboard_corners is None:
            chessboard_corners = self.board.getChessboardCorners()
        chessboard_corners = np.asarray(chessboard_corners, dtype=np.float64).reshape(-1, 3)

        pixels_list = []   # (n, 2) 픽셀
        pts_M_list = []    # (n, 3) 보드 프레임 mm

        if ch_corners is not None and ch_ids is not None and len(ch_ids) > 0:
            n = int(len(ch_ids))
            pixels_list.append(ch_corners.reshape(n, 2).astype(np.float64))
            ids_flat = ch_ids.reshape(n).astype(np.int64)
            pts_M_list.append(chessboard_corners[ids_flat])

        # 2) ArUco 4-corner per marker — ChArUco 보강 / fallback
        marker_corners, marker_ids = self.detect_aruco_only(image_rgb)
        if marker_ids is not None and len(marker_ids) > 0:
            n_aruco = int(len(marker_ids))
            board_ids, board_objs = self._board_marker_points()
            id_to_obj = {int(bi): bo for bi, bo in zip(board_ids, board_objs)}
            for i, mid in enumerate(marker_ids.reshape(-1).tolist()):
                obj = id_to_obj.get(int(mid))
                if obj is None:
                    continue
                pix4 = np.asarray(marker_corners[i], dtype=np.float64).reshape(4, 2)
                pixels_list.append(pix4)
                pts_M_list.append(obj)

        n_charuco = pixels_list[0].shape[0] if (pixels_list and ch_ids is not None and len(ch_ids) > 0) else 0
        n_aruco_pts = sum(p.shape[0] for p in pixels_list) - n_charuco

        if verbose:
            print(f"    [det] aruco markers={n_aruco}  charuco corners={n_charuco}  "
                  f"aruco corners={n_aruco_pts}  total={n_charuco + n_aruco_pts}  "
                  f"max_uv_px={max_uv_match_px:.1f}")

        if not pixels_list:
            if verbose:
                print("    [det] 검출된 corner 없음")
            return None

        pixels = np.vstack(pixels_list)
        pts_M_all = np.vstack(pts_M_list)

        # UV → 3D
        pts_C_all, valid = self._uv_to_3d(
            pixels, vertices_mm.astype(np.float64), uv.astype(np.float64),
            image_wh=(W, H), max_dist_px=max_uv_match_px,
        )
        n_valid = int(valid.sum())
        if verbose:
            print(f"    [det] UV→3D 매칭 valid={n_valid}/{len(pixels)} "
                  f"(min={min_corners})")
        if n_valid < min_corners:
            return None

        pts_M = pts_M_all[valid]
        pts_C = pts_C_all[valid]
        pixels_v = pixels[valid]

        T_MC, rmse = self._procrustes(pts_M, pts_C)

        debug = None
        if draw_debug:
            ids_v = np.arange(len(pixels_v), dtype=np.int64)   # 의미 없는 라벨 (혼합)
            debug = self._draw_debug(
                image_rgb, pixels_v, ids_v, n_aruco, len(pts_M_all), rmse,
            )

        return CharucoDetection(
            n_corners=int(len(pts_M)),
            corner_ids=np.arange(len(pts_M), dtype=np.int64),
            pixels=pixels_v,
            pts_M_mm=pts_M,
            pts_C_mm=pts_C,
            T_MC=T_MC,
            procrustes_rmse_mm=rmse,
            debug_image=debug,
        )

    @staticmethod
    def _draw_debug(
        image_rgb: np.ndarray,
        pixels: np.ndarray,
        ids: np.ndarray,
        n_aruco: int,
        n_charuco: int,
        rmse: float,
    ) -> np.ndarray:
        vis = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        for (u, v), cid in zip(pixels.astype(int), ids):
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), 2)
            cv2.putText(vis, str(int(cid)), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(
            vis,
            f"aruco={n_aruco}  charuco={n_charuco}  used={len(ids)}  rmse={rmse:.2f}mm",
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
        )
        return vis
