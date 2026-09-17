"""range_profile.py — **sim 스캐너의 반환 특성을 실측**해 모사값을 검증한다.

★ 이건 **sim 전용 도구다.** real 은 안 쓴다 — 쓸 이유가 없다.

왜 real 은 안 쓰나
------------------
실물 스캐너는 **자기 작동거리 창을 스스로 안다.** SDK 가 그대로 알려준다:

    ArtecClient.scanning_range()  →  IFrameProcessor::getScanningRange()  →  (near, far) mm

두 백엔드 모두 이 값을 `sensor_from_scanning_range()` 로 받아 `SensorModel.dof` 에
넣는다(`lookaround` §preview). 스캐너가 답을 갖고 있는데 로봇을 거리마다 움직여
히스토그램을 그리는 건 실물 시간 낭비이고, 더 나쁘게는 **측정 오차가 낀 값으로
SDK 가 아는 정답을 덮어쓰는** 짓이다.

왜 sim 은 필요한가
------------------
sim 의 `IsaacArtecScanner.scanning_range()` 는 SDK 가 아니라 **내가 모사한 계약**이다 —
`self._wd_m` 에 넣어 둔 값을 그대로 돌려줄 뿐이라, 그 숫자가 Isaac 렌더러가 실제로
점을 주는 범위와 맞는지는 **아무도 보장하지 않는다.** 여기서 갈라지면
"sim 에서 맞춘 거리가 real 에서 안 맞는" 형태로 나타난다.

이 스크립트가 하는 일이 그 검증이다: 스탠드오프를 훑어 **실제 반환량**과 **깊이
히스토그램**을 재고, 모사값(`MMS_SIM_WD`)이 타당한지 본다.

⚠ **`MMS_SIM_WD_FILTER=0` 으로 돌려야 의미가 있다.** 기본값(1)이면 sim 스캐너가
  `_wd_m` 창 밖의 점을 **미리 잘라서** 주므로, 훑어봐야 넣은 값이 그대로 나온다 —
  순환논법이다. 필터를 끄면 Isaac 렌더러가 실제로 주는 범위(클립·FOV·입사각이
  정하는 것)가 나오고, 그제서야 "모사한 창이 렌더러와 맞나" 를 답할 수 있다.

    env -u PYTHONPATH MMS_BACKEND=isaac MMS_ISAAC_HEADLESS=1 \
        MMS_SIM_WD_FILTER=0 \
        ~/miniconda3/envs/env_isaacsim/bin/python scripts/artec/range_profile.py

무엇을 보나
-----------
  · 스탠드오프별 반환 점 수 — **운용 지점**(어디에 자세를 둘지)
  · 합친 깊이 히스토그램  — sim 센서 특성. `MMS_SIM_WD` 와 맞는지 대조한다.

판단 로직은 `utils/nbv/range_profile.py`(공용) 에 있다 — preview 의 거리 결정
(`standoff.distance_correction`)과 같은 `point_ranges` 를 쓰므로, 여기서 본 분포가
곧 preview 가 보는 분포다.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from utils.nbv import range_profile as rp                       # noqa: E402
from utils.nbv import lookaround as p1                    # noqa: E402
from utils.robot import xarm7_kinematics as kin                 # noqa: E402
from utils.robot import view_pose as vp                         # noqa: E402


def build(backend: str):
    """(mms, robot, 캡처 콜백 재료) — 백엔드별 준비."""
    from mms_artec.system import ArtecMMS, ArtecMMSConfig
    from mms_artec.sensor.artec_config import ArtecConfig
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    cfg = ArtecMMSConfig(
        artec=ArtecConfig(serial_number=None, capture_texture=False,
                          target_interval_s=0.0),
        turntable_frame_yaml=os.path.join(root, "config/calibration/turntable_frame.yaml"),
        sensor_frames_yaml=os.path.join(root, "config/sensor_frames.yaml"),
        T_EC_key="T_EC_artec", backend=backend,
        robot_ip=os.environ.get("MMS_ROBOT_IP", "192.168.1.210"),
        turntable_ip=os.environ.get("MMS_TT_IP", "192.168.0.10"), turntable_bd_id=0,
        isaac_headless=os.environ.get("MMS_ISAAC_HEADLESS", "1") == "1",
        isaac_usd_path=os.environ.get("MMS_SIM_USD") or None)
    return cfg


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default=os.environ.get("MMS_BACKEND", "isaac"),
                    choices=("isaac", "real"),
                    help="sim 전용 도구다. real 은 SDK 가 직접 알려준다(모듈 설명 참조)")
    ap.add_argument("--from", dest="d0", type=float, default=0.18, help="시작 스탠드오프 m")
    ap.add_argument("--to", dest="d1", type=float, default=0.42, help="끝 스탠드오프 m")
    ap.add_argument("--step", type=float, default=0.02, help="간격 m")
    ap.add_argument("--el", type=float, default=30.0, help="고도각 deg")
    ap.add_argument("--az", type=float, default=0.0, help="방위각 deg")
    ap.add_argument("--aim-above", type=float, default=0.05,
                    help="조준점을 디스크 상면 위 이만큼 (m)")
    ap.add_argument("--bin", type=float, default=0.005, help="히스토그램 bin m")
    ap.add_argument("--frac", type=float, default=0.25, help="'쓸만한 창' 기준(피크 대비)")
    a = ap.parse_args()

    from mms_artec.system import ArtecMMS
    cfg = build(a.backend)
    standoffs = np.arange(a.d0, a.d1 + a.step * 0.5, a.step)
    print(f"[range] backend={a.backend}  standoff {a.d0*1000:.0f}~{a.d1*1000:.0f}mm "
          f"step {a.step*1000:.0f}mm  el={a.el:.0f}° az={a.az:.0f}°")

    if a.backend == "isaac" and os.environ.get("MMS_SIM_WD_FILTER", "1") == "1":
        print("[range] ⚠ MMS_SIM_WD_FILTER=1 (기본) — sim 이 작동거리 창으로 점을\n"
              "        **미리 잘라서** 준다. 이 상태로 훑으면 넣은 값이 그대로 나오는\n"
              "        순환논법이다. 검증하려면 `MMS_SIM_WD_FILTER=0` 으로 다시 돌릴 것.")
    with ArtecMMS(cfg) as mms:
        if a.backend == "isaac":
            from mms_artec.backends import get_isaac_world
            from mms_artec.backends.isaac import isaac_world as IW
            w = get_isaac_world(cfg)
            from mms_artec.backends import build_hardware
            robot, _tt = build_hardware(cfg)
            robot.go_home(sensor="artec", confirm=False)
            w.step(20, render=True)
            w.ensure_camera(); w.step(20, render=True)
            axis_w, _ = w.prim_world_pose(IW.DISC_PRIM)
            axis_w = np.asarray(axis_w, float)
            T_WB = np.asarray(w.T_WB) if hasattr(w, "T_WB") else None
            if T_WB is None:
                pos, R = w.base_world_pose()
                T_WB = np.eye(4); T_WB[:3, :3] = R; T_WB[:3, 3] = pos
            up = np.array([0.0, 0.0, 1.0])
            tgt = axis_w + up * a.aim_above
            disc_top = float(axis_w[2])

            def capture_at(d):
                seed = np.asarray(robot.get_joint_angles(is_radian=True), float)
                q, _, eye = vp.solve_view_q(kin, tgt, a.el, a.az, float(d), seed,
                                            np.asarray(mms._T_EC), T_WB=T_WB,
                                            convention=vp.CAM_USD,
                                            rolls_deg=vp.DEFAULT_ROLLS_DEG, n_seed_alt=8)
                if q is None:
                    return None
                # sim·real 공용 API (IsaacXArm 의 shim 이 xArm SDK 와 같은 시그니처)
                robot.arm.set_servo_angle(angle=np.asarray(q, float).tolist(),
                                          is_radian=True, wait=True)
                w.step(20, render=True)
                pc_b = mms.sensor.capture_points_base(robot, mms._T_EC, settle=5)
                if pc_b is None or len(pc_b) == 0:
                    return (np.zeros((0, 3)), eye)
                pw = np.asarray(pc_b, float) @ T_WB[:3, :3].T + T_WB[:3, 3]
                obj = p1.crop_object_points(pw, axis_w[:2], disc_top, up_sign=+1.0)
                cam, _ = w.prim_world_pose(IW.CAMERA_PRIM)
                return (obj, np.asarray(cam, float))
        else:
            raise SystemExit(
                "[range] 이 도구는 **sim 전용**이다 — 미완성이 아니라 의도다.\n"
                "        실물 스캐너는 SDK 가 작동거리 창을 직접 알려준다:\n"
                "            ArtecClient.scanning_range() → IFrameProcessor::getScanningRange\n"
                "        파이프라인은 이미 그 값을 쓴다(sensor_from_scanning_range).\n"
                "        로봇을 거리마다 움직여 재는 것은 실물 시간 낭비이고,\n"
                "        측정 오차로 SDK 가 아는 정답을 덮어쓰게 된다.\n"
                "        창을 바꾸려면 ArtecConfig.scan_range_near_mm / _far_mm.\n"
                "        sim 은 그 계약을 **모사**할 뿐이라 검증이 필요해서 이 도구가 있다.")

        prof = rp.sweep_standoff(capture_at, standoffs, bin_m=a.bin,
                                 log=lambda m: print(f"[range]{m}"))

        # ★ 리포트는 **with 블록 안에서** 찍는다 — Isaac 종료(SimulationApp
        #   __exit__)가 segfault 로 죽는 경우가 있어 뒤 출력이 잘린다.
        print("\n" + "=" * 64)
        print("스탠드오프별 반환 (운용 지점)")
        print("=" * 64)
        mx = max([s.n_points for s in prof.samples] + [1])
        for s in prof.samples:
            bar = "█" * int(round(s.n_points / mx * 44))
            pk = "" if s.hist is None else f"  피크 {s.hist.peak_m()*1000:.0f}mm"
            print(f"  {s.standoff*1000:3.0f}mm {s.n_points:>7,} |{bar}{pk}{'  '+s.note if s.note else ''}")
        b = prof.best()
        if b:
            near, far = b.hist.window(a.frac) if b.hist else (float('nan'),) * 2
            print(f"\n  ★ 최다 반환 standoff = {b.standoff*1000:.0f}mm "
                  f"({b.n_points:,}점, 표면거리 창 {near*1000:.0f}~{far*1000:.0f}mm)")

        m = rp.merged_histogram(prof)
        if m is not None:
            print("\n" + "=" * 64)
            print("합친 깊이 히스토그램 (센서 특성 — SensorModel.dof 의 근거)")
            print("=" * 64)
            print(rp.render(m, frac=a.frac))
            near, far = m.window(a.frac)
            print(f"\n  → 측정 작동거리 창 = ({near:.3f}, {far:.3f}) m")
            print(f"     현재 SensorModel.dof = {p1.SensorModel().dof}  "
                  f"{'일치' if abs(near-p1.SensorModel().dof[0])<0.02 and abs(far-p1.SensorModel().dof[1])<0.02 else '← 다르다. 실측값으로 바꿀 것'}")


if __name__ == "__main__":
    main()
