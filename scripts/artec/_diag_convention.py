#!/usr/bin/env python
# scripts/_diag_artec_convention.py
#
# Artec mesh vertices 의 좌표계 convention 진단.
# 한 프레임만 캡처해서 vertex z 분포와 우리 T_CO 적용 결과를 비교.
#
# Usage: python scripts/_diag_artec_convention.py
#   - 로봇이 home pose 에 있어야 함 (보드 또는 객체 시야 안에)
#   - 턴테이블 무관

from __future__ import annotations

import sys
from pathlib import Path
import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms_artec.system import ArtecMMS as MMS, ArtecMMSConfig as MMSConfig
from mms_artec.sensor.artec_client import ArtecConfig
from utils.robot.xarm_interface import XArmInterface


def main():
    cfg = MMSConfig(
        artec=ArtecConfig(serial_number=None, capture_texture=True),
        turntable_frame_yaml=str(_PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
        sensor_frames_yaml=str(_PROJECT_ROOT / "config/sensor_frames.yaml"),
        T_EC_key="T_EC_artec",
    )
    robot = XArmInterface("192.168.1.210")

    with MMS(cfg) as mms:
        T_EB = robot.get_ee_pose_mat()
        print(f"T_EB translation (m): {T_EB[:3, 3]}")
        print(f"T_EC translation (m): {mms._T_EC[:3, 3]}")

        fmh = mms.sensor.capture_frame(capture_texture=False)
        if fmh is None or fmh.vertex_count() == 0:
            print("capture 실패")
            return
        v = fmh.vertices().astype(np.float64)    # (N, 3) mm
        print(f"\n[Vertices in C frame]  shape={v.shape}")
        for axis, name in enumerate(["x", "y", "z"]):
            print(f"  {name}: min={v[:, axis].min():+.1f}  "
                  f"max={v[:, axis].max():+.1f}  "
                  f"mean={v[:, axis].mean():+.1f} mm")

        z_sign = np.sign(v[:, 2].mean())
        print(f"\n→ vertex z mean sign: {'POSITIVE (OpenCV-style)' if z_sign > 0 else 'NEGATIVE (OpenGL/Artec-style)'}")

        # T_CO 적용 (theta=0)
        T_CO = mms.T_CO(theta=0.0, T_EB=T_EB)
        R = T_CO[:3, :3]; t = T_CO[:3, 3]
        print(f"\n[T_CO @ theta=0]")
        print(f"  t (m, camera origin in O): {t}")
        print(f"  R column 2 (camera +z direction in O): {R[:, 2]}")

        # 변환
        v_m = v / 1000.0
        v_O_direct = v_m @ R.T + t
        # 옵션 A: 그대로
        d_origin_A = np.linalg.norm(v_O_direct, axis=1).mean()
        # 옵션 B: vertex z 부호 flip
        v_flipZ = v_m.copy(); v_flipZ[:, 2] = -v_flipZ[:, 2]
        v_O_flipZ = v_flipZ @ R.T + t
        d_origin_B = np.linalg.norm(v_O_flipZ, axis=1).mean()
        # 옵션 C: vertex y, z 부호 flip (OpenGL → OpenCV, 180° about x)
        v_flipYZ = v_m.copy(); v_flipYZ[:, 1] = -v_flipYZ[:, 1]; v_flipYZ[:, 2] = -v_flipYZ[:, 2]
        v_O_flipYZ = v_flipYZ @ R.T + t
        d_origin_C = np.linalg.norm(v_O_flipYZ, axis=1).mean()

        print(f"\n[Vertices 의 O frame 평균거리 — 정상이면 < 200mm = 객체 위 점들]")
        print(f"  A. v 그대로:           {d_origin_A*1000:.1f} mm")
        print(f"  B. z 만 flip:          {d_origin_B*1000:.1f} mm")
        print(f"  C. y,z flip (OGL→CV):  {d_origin_C*1000:.1f} mm")
        print(f"\n→ 가장 작은 옵션이 올바른 convention.")

    robot.disconnect()


if __name__ == "__main__":
    main()
