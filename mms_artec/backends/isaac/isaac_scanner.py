"""
IsaacArtecScanner — 스캐너 백엔드 (Isaac Sim).

`mms_artec.sensor.artec_client.ArtecClient` 와 덕타이핑 호환.

[Phase A] 현재는 lifecycle stub. 실제 스캔(카메라 depth→포인트클라우드→정합/퓨전)
경로는 Phase B 에서 sim_model / sim_mesh_ops / isaac_scan_session 와 함께 구현한다.
"""

from __future__ import annotations

import numpy as np


class IsaacArtecScanner:
    def __init__(self, world, artec_cfg):
        self._world = world
        self.cfg = artec_cfg
        self._initialized = False

    # ── lifecycle ──────────────────────────────────────────────────────────
    def initialize(self) -> None:
        self._world.ensure_camera()
        self._initialized = True
        print("[IsaacArtecScanner] (sim) initialized — Isaac 카메라 사용")

    def shutdown(self) -> None:
        self._initialized = False
        print("[IsaacArtecScanner] (sim) shutdown")

    # ── 점군 캡처 (로봇 base 프레임) ─────────────────────────────────────────
    def capture_points_base(self, robot=None, T_EC=None, settle: int = 2) -> np.ndarray:
        """
        현재 스캐너 시점의 점군을 **로봇 base 프레임**(m)으로 반환.

        실물 ArtecClient.capture_points_base 와 동일 인터페이스. sim 은 Isaac 카메라가
        world 프레임 점군을 주므로 base(=xarm7 root) 로 변환한다. (robot/T_EC 미사용 —
        sim 카메라 포즈가 ground-truth. SLAM 추적 개념 없음.)
        """
        cam = self._world.ensure_camera()
        for _ in range(int(settle)):
            self._world.world.step(render=True)
        pc = cam.get_pointcloud()
        if pc is None or len(pc) == 0:
            return np.zeros((0, 3))
        pc = np.asarray(pc, dtype=float)
        pos, R = self._world.base_world_pose()        # base→world
        T_bw = np.eye(4); T_bw[:3, :3] = R; T_bw[:3, 3] = pos
        T_wb = np.linalg.inv(T_bw)                     # world→base
        ph = np.concatenate([pc, np.ones((len(pc), 1))], axis=1)
        return (ph @ T_wb.T)[:, :3]

    # ── rim-클릭(방법 1.2)용 조직화 캡처 ────────────────────────────────────
    def capture_organized(self, settle: int = 2):
        """
        rim 3점 클릭용 **조직화 캡처** → (intensity, organized_pts, T_CB).

        rim_picker(utils.calibration.rim_picker) 계약에 맞춘다:
          intensity     : (H,W) uint8        — 표시용(RGB→gray)
          organized_pts : (H,W,3) float32    — 픽셀별 3D, **base 프레임·mm**
          T_CB          : (4,4) = 단위행렬   — organized 가 이미 base 라서

        ★ get_camera_points_from_image_coords(광학 프레임: x→右, y→下, z=+depth, 검증됨)
          로 픽셀 3D 를 얻고, 신뢰되는 prim_world_pose(CAMERA_PRIM) + 광학↔USD-prim
          flip(diag(1,-1,-1))으로 world→base 로 옮겨 mm 로 저장한다. 무효 깊이 픽셀은 0.
          (Isaac 의 get_world_points_from_image_coords 는 depth 와 불일치 — 사용 안 함.)
        """
        from mms_artec.backends.isaac.isaac_world import CAMERA_PRIM
        cam = self._world.ensure_camera()
        for _ in range(int(settle)):
            self._world.world.step(render=True)
        depth = np.asarray(cam.get_depth())
        rgba = np.asarray(cam.get_rgba())
        H, Wd = depth.shape
        gray = (0.299 * rgba[:, :, 0] + 0.587 * rgba[:, :, 1]
                + 0.114 * rgba[:, :, 2]).astype(np.uint8)
        org = np.zeros((H, Wd, 3), dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 1e-4) & (depth < 10.0)
        vs, us = np.nonzero(valid)
        if len(vs):
            coords = np.stack([us, vs], axis=1).astype(float)     # (N,2) = (u,v)
            cpts = np.asarray(cam.get_camera_points_from_image_coords(coords, depth[vs, us]))
            cpos, cR = self._world.prim_world_pose(CAMERA_PRIM)    # USD prim → world
            F = np.diag([1.0, -1.0, -1.0])                        # 광학 → USD prim
            world = (cR @ F @ cpts.T).T + cpos                    # 광학 → world
            pos, R = self._world.base_world_pose()                # base→world
            T_wb = np.eye(4); T_wb[:3, :3] = R; T_wb[:3, 3] = pos
            T_bw = np.linalg.inv(T_wb)                            # world→base
            ph = np.concatenate([world, np.ones((len(world), 1))], axis=1)
            base_pts = (ph @ T_bw.T)[:, :3]
            org[vs, us] = (base_pts * 1000.0).astype(np.float32)  # base, mm
        return gray, org, np.eye(4)

    # ── Phase B 에서 구현 ───────────────────────────────────────────────────
    def _not_impl(self, name):
        raise NotImplementedError(
            f"IsaacArtecScanner.{name} 은 Phase B 에서 구현됩니다 "
            f"(sim 스캔 파이프라인).")

    @staticmethod
    def serial_registration(model):     # noqa: D401
        return model

    @staticmethod
    def global_registration(model):
        return model
