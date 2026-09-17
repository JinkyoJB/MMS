"""set_camera_fov.py — 씬 USD 의 **스캐너 카메라 화각을 실측값으로** 맞춘다.

왜 필요한가
-----------
카메라 FOV 가 세 군데에 따로 있었다(2026-09-17 정리 전):

    ① `SensorModel.hfov_deg/vfov_deg`  21.58 / 28.58   ← 실측 K, **계획기가 쓰는 값**
    ② `isaac_world.SPIDER_HFOV_DEG`    30.0            ← sim 렌더러 (런타임 덮어쓰기)
    ③ 씬 USD 의 authored 값             91.4 / 74.8     ← GUI 로 열면 보이는 값

②는 ①에서 가져오도록 고쳤지만, ③은 여전히 엉뚱해서 **GUI 로 씬을 열면 스캐너가
전혀 다른 화각으로 보인다.** 런타임이 덮어쓰니 스캔 결과는 맞지만, 눈으로 확인할
때 속는다 — 오늘 "카메라가 어디 보나" 를 눈으로 못 봐서 오래 헤맸던 것과 같은 종류다.

**단일 출처는 `SensorModel` 이다.** 매뉴얼을 보고 값을 바꾸려면 거기만 고치고
이 스크립트를 다시 돌린다.

    env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python \
        scripts/sim/set_camera_fov.py            # 기본 = v2_real + v3 씬 둘 다

USD 카메라는 FOV 를 직접 안 받고 `focalLength` + `aperture` 로 표현한다:

    focal = hAperture / (2·tan(hfov/2))
    vAperture = 2·focal·tan(vfov/2)

`hAperture` 는 자유 스케일이라 관례값 20.5 를 유지한다(다른 값을 써도 화각은 같다).
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from mms_paths import asset                                      # noqa: E402
from utils.nbv.lookaround import SensorModel               # noqa: E402

CAMERA_PATH = "/World/xarm7/link7/tool/spider/Camera"
H_APERTURE = 20.5                      # 관례값 — 화각은 focal 과의 비로 정해진다
DEFAULT_SCENES = [
    asset("frame_xarm7_spider_turntable/v2_real_260917.usd"),
    asset("frame_xarm7_spider_turntable_v2/v3_scene.usd"),
]


def intrinsics(hfov_deg: float, vfov_deg: float):
    focal = H_APERTURE / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
    v_ap = 2.0 * focal * math.tan(math.radians(vfov_deg) / 2.0)
    return focal, H_APERTURE, v_ap


def fov_of(focal, h_ap, v_ap):
    return (2 * math.degrees(math.atan(h_ap / 2 / focal)),
            2 * math.degrees(math.atan(v_ap / 2 / focal)))


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", nargs="*", default=None,
                    help="대상 씬 USD (기본: v2_real_260917 + v3_scene)")
    ap.add_argument("--camera", default=CAMERA_PATH)
    ap.add_argument("--hfov", type=float, default=None, help="deg (기본 = SensorModel)")
    ap.add_argument("--vfov", type=float, default=None, help="deg (기본 = SensorModel)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    sm = SensorModel()
    hfov = a.hfov if a.hfov is not None else sm.hfov_deg
    vfov = a.vfov if a.vfov is not None else sm.vfov_deg
    focal, h_ap, v_ap = intrinsics(hfov, vfov)
    print(f"[fov] 목표 {hfov:.2f}deg(가로) x {vfov:.2f}deg(세로)"
          f"{'' if (a.hfov or a.vfov) else '  ← SensorModel(실측 K)'}")
    print(f"[fov]   focalLength={focal:.4f}  hAperture={h_ap:.4f}  vAperture={v_ap:.4f}")

    from pxr import Usd, UsdGeom, Sdf
    for path in (a.scenes or DEFAULT_SCENES):
        if not os.path.isfile(path):
            print(f"  ⚠ 없음(건너뜀): {path}")
            continue
        stage = Usd.Stage.Open(path)
        prim = stage.GetPrimAtPath(a.camera)
        if not (prim and prim.IsValid()):
            print(f"  ⚠ 카메라 prim 없음(건너뜀): {os.path.basename(path)} {a.camera}")
            continue
        old = tuple(float(prim.GetAttribute(n).Get() or 0.0) for n in
                    ("focalLength", "horizontalAperture", "verticalAperture"))
        ohf, ovf = fov_of(*old) if old[0] else (float("nan"),) * 2
        if a.dry_run:
            print(f"  (dry) {os.path.basename(path)}: "
                  f"{ohf:.2f}x{ovf:.2f}deg → {hfov:.2f}x{vfov:.2f}deg")
            continue
        for name, val in (("focalLength", focal), ("horizontalAperture", h_ap),
                          ("verticalAperture", v_ap)):
            at = prim.GetAttribute(name)
            if not at.IsValid():
                at = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
            at.Set(float(val))
        stage.GetRootLayer().Save()
        print(f"  ✓ {os.path.basename(path)}: "
              f"{ohf:.2f}x{ovf:.2f}deg → {hfov:.2f}x{vfov:.2f}deg")

    print("\n  ※ 런타임(`isaac_world._configure_scanner_camera`)도 같은 SensorModel 을"
          "\n    쓰므로 값이 갈라지지 않는다. 매뉴얼 값으로 바꾸려면 SensorModel 을"
          "\n    고치고 이 스크립트를 다시 돌릴 것.")


if __name__ == "__main__":
    main()
