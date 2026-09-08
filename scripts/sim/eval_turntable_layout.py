"""eval_turntable_layout.py — 턴테이블 위치를 설계변수로 놓고 레이아웃 평가.

목표
----
로봇 base 는 고정. **턴테이블 위치**를 정한다. 만족해야 할 것:
  1. 스캔 거리가 스윗스팟(Artec Spider 작동거리 0.2~0.3m)
  2. 관측 elevation 이 30~80° 로 **다양하게** 나올 것
  3. F* 그리퍼(3F Delto)로 턴테이블 위 사물을 **집을 수 있을 것**

충돌 판정 — 왜 캡슐을 안 쓰나
---------------------------
`utils/collision/robot_collision` 의 캡슐 근사는 이 팔에서 오탐이 심하다(실측):
  · 링크 캡슐이 관절원점을 잇는 **직선**이라 꺾인 link4 를 잘못 덮는다
    → 툴↔link4 축간 58mm 로 계산되지만 실제 메시 거리는 93mm 이상
  · 툴 캡슐이 플랜지~카메라를 균일 반경 95mm 로 덮어 플랜지 쪽을 38mm 과대평가
결과적으로 도달 가능한 자세가 전부 '자가충돌'로 걸러졌다.
→ 여기서는 **USD 실제 메시**로 최소거리를 잰다. 해석 FK 프레임이 USD 링크 프레임과
  위치 1.5~8.6mm·회전 1.4° 로 일치하므로(실측), 로봇을 구동하지 않고 링크 로컬 메시를
  FK 로 배치하면 정확하면서도 빠르다.

속도 — KD-tree 를 링크 **로컬** 프레임에 한 번만 짓는다. 자세마다 툴 점을 각 링크
로컬로 역변환해 질의하면 트리 재구축이 없다. (전수 pairwise 대비 100 배 이상)

사용
    env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python \
        scripts/sim/eval_turntable_layout.py
    #  --x-min -0.35 --x-max 0.45 --x-step 0.05  --base-dz 0 -0.093  --clearance 0.02
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

SCENE = asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT, LINK7 = "/World/xarm7", "/World/xarm7/link7"
CAMERA = "/World/xarm7/link7/tool/spider/Camera"
TOOL = "/World/xarm7/link7/tool"

ELS = (30, 40, 50, 60, 70, 80)
AZIS = (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0)
WORK_FOCUS = 0.25            # Spider 스윗스팟 중앙 (0.2~0.3)

# 그리퍼(F* 3F Delto) — 툴 체인 축은 link7 +Z (툴 메시 실측으로 검증한다).
# 툴 스탠드 bbox Z 0.546~0.748 → 플레이트 하면에서 202mm.
GRIP_TIP_Z = 0.212           # link7 원점 → 핑거 끝 (플레이트 10mm + 202mm)
GRIP_CENTER_Z = 0.172        # 파지 중심(핑거 끝에서 40mm 안쪽)
GRIP_R = 0.060               # 그리퍼 몸통 반경 근사


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x-min", type=float, default=-0.35)
    ap.add_argument("--x-max", type=float, default=0.45)
    ap.add_argument("--x-step", type=float, default=0.05)
    ap.add_argument("--base-dz", type=float, nargs="+", default=[0.0, -0.093],
                    help="로봇 base Z 오프셋 m (0=현재 v3, -0.093=v2 높이)")
    ap.add_argument("--clearance", type=float, default=0.02,
                    help="허용 최소 여유 m (이보다 가까우면 충돌로 본다)")
    ap.add_argument("--obj-r", type=float, default=0.063)
    ap.add_argument("--obj-h", type=float, default=0.077)
    ap.add_argument("--disc-top", type=float, default=0.665)
    ap.add_argument("--diag", type=float, default=None,
                    help="이 X 하나만 상세 진단 출력")
    ap.add_argument("--rolls", type=float, nargs="+", default=[0, 45, 90, 135, 180, 225, 270, 315],
                    help="시도할 광축 회전(도). 기여도 분리용으로 '0' 만 줄 수 있다")
    ap.add_argument("--n-seed", type=int, default=11, help="추가 IK 시드 개수")
    ap.add_argument("--csv", default="scripts/sim/log/turntable_layout.csv",
                    help="스윕 결과 CSV 저장 경로")
    ap.add_argument("--dump-poses", default="scripts/sim/log/layout_poses.json",
                    help="대표 자세(관절각) 저장 경로 — GUI 재현용")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from scipy.spatial import cKDTree
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import Usd, UsdGeom

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from utils.robot import xarm7_kinematics as kin
    from mms_artec.utils.calibration.handeye_geometry import (
        look_at_camera as _look, make_T as _mk)

    open_stage(SCENE)
    for _ in range(60):
        app.update()
    stage = get_context().get_stage()
    xc = UsdGeom.XformCache()
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)

    def W(p):
        xc.Clear()
        return np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T

    T_WB0 = W(ROBOT)
    T_EC = np.linalg.inv(W(CAMERA)) @ W(LINK7)

    def local_verts(path, ref, stride, skip_tool=False):
        Li = np.linalg.inv(W(ref))
        P = []
        for d in Usd.PrimRange(stage.GetPrimAtPath(path), pred):
            if not d.IsA(UsdGeom.Mesh):
                continue
            if skip_tool and "/tool" in str(d.GetPath()):
                continue
            A = Li @ np.array(xc.GetLocalToWorldTransform(d)).T
            q = UsdGeom.Mesh(d).GetPointsAttr().Get() or []
            if not len(q):
                continue
            V = np.asarray(q, dtype=float)[::stride]
            P.append(V @ A[:3, :3].T + A[:3, 3])
        return np.vstack(P) if P else np.zeros((0, 3))

    LINKS = [local_verts(f"/World/xarm7/link{i}", f"/World/xarm7/link{i}",
                         7, skip_tool=True) for i in range(1, 8)]
    TREES = [cKDTree(L) if len(L) else None for L in LINKS]
    # 툴은 100만 정점급이라 균일 솎기. 6천점이면 표면 간격 수 mm 로, 여유 20mm
    # 판정에는 충분하다(질의 비용은 링크 트리 5개 × 점수에 비례).
    _tv = local_verts(TOOL, LINK7, 11)            # Spider 포함 (스캔용 툴)
    TOOLV = _tv[:: max(1, len(_tv) // 6000)]

    # 툴 축 방향 검증 — 툴 체인이 link7 의 +Z 로 뻗는가?
    zmin, zmax = TOOLV[:, 2].min(), TOOLV[:, 2].max()
    print(f"로봇 base(world) {np.round(T_WB0[:3,3],3)}")
    print(f"툴 메시 z 범위(link7 프레임) {zmin*1000:+.0f} ~ {zmax*1000:+.0f} mm  "
          f"→ 툴축 = link7 {'+Z' if abs(zmax) > abs(zmin) else '-Z'}")
    print("링크 메시 샘플: " + ", ".join(f"L{i+1}={len(LINKS[i])}" for i in range(7))
          + f", tool={len(TOOLV)}")

    # 피킹용 툴 = 툴체인저(축 0.065m 이내 실제 메시) + 그리퍼 원기둥 근사
    stack = _tv[_tv[:, 2] <= 0.065][::8]
    th = np.linspace(0, 2 * np.pi, 16, endpoint=False)
    zz = np.linspace(0.065, GRIP_TIP_Z, 12)
    cyl = np.array([[GRIP_R * math.cos(t), GRIP_R * math.sin(t), z]
                    for z in zz for t in th])
    GRIPV = np.vstack([stack, cyl])

    # ── 환경(프레임 벽·상판·저울) — 턴테이블 그룹은 X 로 움직이므로 제외하고
    #    후보마다 원기둥으로 다시 붙인다. 스캔 대상 물체는 파지해야 하므로 장애물 아님.
    TT_AXIS0, TT_R, TT_ZR = 0.365, 0.16, (0.50, 0.70)
    envp = []
    for d in Usd.PrimRange(stage.GetPrimAtPath("/World/frame"), pred):
        if not d.IsA(UsdGeom.Mesh):
            continue
        A = np.array(xc.GetLocalToWorldTransform(d)).T
        q = UsdGeom.Mesh(d).GetPointsAttr().Get() or []
        if not len(q):
            continue
        V = np.asarray(q, dtype=float)[::5] @ A[:3, :3].T + A[:3, 3]
        c = V.mean(0)
        if (math.hypot(c[0] - TT_AXIS0, c[1]) < TT_R
                and TT_ZR[0] < c[2] < TT_ZR[1]):
            continue                              # 턴테이블 그룹
        envp.append(V)
    ENV = np.vstack(envp)
    ENV = ENV[:: max(1, len(ENV) // 40000)]
    print(f"환경 메시 {len(ENV)}점 (턴테이블 그룹 제외)")

    def env_tree(ax_x):
        """정적 환경 + 후보 X 의 턴테이블 본체(원기둥)."""
        th = np.linspace(0, 2 * np.pi, 36, endpoint=False)
        zz = np.linspace(0.510, args.disc_top, 10)
        cyl = np.array([[ax_x + 0.119 * math.cos(t), 0.119 * math.sin(t), z]
                        for z in zz for t in th])
        return cKDTree(np.vstack([ENV, cyl]))

    def frames(q):
        return kin._fk_frames_m(q)                # (8,4,4) base 기준 m

    def env_clear(q, toolv, tree, T_WB):
        """link3~7 + 툴 ↔ 환경 최소거리(m). link1/2 는 천장 마운트에 붙어 있어 제외."""
        F = frames(q)
        pts = [LINKS[i] @ F[i + 1][:3, :3].T + F[i + 1][:3, 3] for i in range(2, 7)]
        pts.append(toolv @ F[7][:3, :3].T + F[7][:3, 3])
        P = np.vstack(pts) @ T_WB[:3, :3].T + T_WB[:3, 3]      # base → world
        return float(tree.query(P, k=1)[0].min())

    def self_clear(q, toolv):
        """툴 ↔ link1~5 실제 메시 최소거리(m). link6/7 은 인접이라 제외."""
        F = frames(q)
        T7 = F[7]
        Wt = toolv @ T7[:3, :3].T + T7[:3, 3]     # base 프레임 툴 점
        best = 1e9
        for i in range(5):
            if TREES[i] is None:
                continue
            Ti = F[i + 1]
            loc = (Wt - Ti[:3, 3]) @ Ti[:3, :3]   # 링크 로컬로 역변환
            best = min(best, float(TREES[i].query(loc, k=1)[0].min()))
        return best

    Q0 = np.radians(np.array([-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63]))
    # 해석 DLS IK 는 국소해라 시드 하나면 도달 가능한 자세를 놓친다(실측: top-down 파지가
    # X 50mm 차이로 0/8↔8/8 로 튀었다). 여러 시드를 시도해 실제 도달성을 재현한다.
    _rng = np.random.default_rng(0)
    SEEDS = [Q0] + [Q0 + _rng.uniform(-1.0, 1.0, 7)
                    for _ in range(max(0, args.n_seed - 4))] + [
        np.zeros(7),
        np.radians(np.array([0.0, -60.0, 0.0, 60.0, 0.0, 120.0, 0.0])),
        np.radians(np.array([90.0, -60.0, 0.0, 60.0, 0.0, 120.0, 0.0])),
        np.radians(np.array([-90.0, -60.0, 0.0, 60.0, 0.0, 120.0, 0.0])),
    ]

    def ik_at(T_W_tip, T_tip_link7, T_WB):
        T_EB = (np.linalg.inv(T_WB) @ T_W_tip) @ T_tip_link7
        p6 = np.concatenate([T_EB[:3, 3] * 1000.0, kin.R_to_euler_xyz(T_EB[:3, :3])])
        for s in SEEDS:
            q, ok = kin.ik(p6, seed=s)
            if ok:
                return q
        return None

    # 스캐너는 광축(roll) 회전이 자유롭다 — 축 방향으로 돌려도 같은 면을 본다.
    # look_at 은 up=(0,0,1) 로 roll 을 하나로 고정하므로, 그대로 두면 시드 문제와 똑같이
    # 도달 가능한 자세를 인위적으로 막는다. 여러 roll 을 시도해 실제 도달성을 본다.
    ROLLS = [math.radians(r) for r in args.rolls]

    def cam_world(q, T_WB):
        """IK 해 q 가 실제로 카메라를 어디에 세우는지 — FK 로 되짚는다."""
        T7 = T_WB @ frames(q)[7]
        return T7 @ np.linalg.inv(T_EC)

    def scan_pose(target_w, el, az, standoff, T_WB, acc=None):
        e, a = math.radians(el), math.radians(az)
        eye = target_w + standoff * np.array(
            [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
        T_WC0 = _mk(_look(eye, target_w, (0, 0, 1.0)), eye)
        for r in ROLLS:
            Rz = np.eye(4)
            c, s = math.cos(r), math.sin(r)
            Rz[:2, :2] = [[c, -s], [s, c]]        # 카메라 로컬 Z(광축) 둘레 회전
            q = ik_at(T_WC0 @ Rz, T_EC, T_WB)
            if q is not None:
                if acc is not None:
                    # IK 는 수렴했다고 하지만 실제로 그 거리에 섰는지는 별개다.
                    cw = cam_world(q, T_WB)
                    acc["err"].append(float(np.linalg.norm(cw[:3, 3] - eye)))
                    acc["d"].append(float(np.linalg.norm(cw[:3, 3] - target_w))
                                    - args.obj_r)
                return q
        return None

    def pick_pose(grasp_w, el, az, T_WB):
        """파지 자세를 직접 구성 — look_at 은 수직 접근에서 퇴화하므로 쓰지 않는다.

        a = 물체→그리퍼 방향. 그리퍼 축(link7 +Z)이 -a 를 향해야 물체를 향해 내려온다.
        """
        e, a_ = math.radians(el), math.radians(az)
        a = np.array([math.cos(e) * math.cos(a_), math.cos(e) * math.sin(a_), math.sin(e)])
        z = -a
        ref = np.array([0.0, 0.0, 1.0]) if abs(z[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
        x0 = np.cross(ref, z)
        x0 /= np.linalg.norm(x0)
        y0 = np.cross(z, x0)
        T_tip_link7 = np.eye(4)
        T_tip_link7[2, 3] = -GRIP_CENTER_Z        # 파지중심 → link7
        # 3핑거 그리퍼 + 대체로 축대칭인 대상 → 접근축 둘레 회전도 자유롭게 본다.
        for r in ROLLS:
            R = np.eye(4)
            R[:3, 0] = math.cos(r) * x0 + math.sin(r) * y0
            R[:3, 1] = np.cross(z, R[:3, 0])
            R[:3, 2] = z
            R[:3, 3] = grasp_w
            q = ik_at(R, T_tip_link7, T_WB)
            if q is not None:
                return q
        return None

    def eval_layout(ax_x, dz, diag=False):
        T_WB = T_WB0.copy()
        T_WB[2, 3] += dz
        tgt = np.array([ax_x, 0.0, args.disc_top + args.obj_h / 2])
        stand = WORK_FOCUS + args.obj_r
        tree = env_tree(ax_x)
        els_ok, n_scan, ws = [], 0, {"ik": 0, "self": 0, "env": 0}
        per_el, rep = {}, {}
        acc = {"err": [], "d": []}
        for el in ELS:
            ok = 0
            for az in AZIS:
                q = scan_pose(tgt, el, az, stand, T_WB, acc)
                if q is None:
                    ws["ik"] += 1
                elif self_clear(q, TOOLV) < args.clearance:
                    ws["self"] += 1
                elif env_clear(q, TOOLV, tree, T_WB) < args.clearance:
                    ws["env"] += 1
                else:
                    ok += 1
                    rep.setdefault(f"scan_el{el}", [float(v) for v in q])
            per_el[el] = ok
            if ok:
                els_ok.append(el)
            n_scan += ok
            if diag:
                print(f"      el={el:>2}  통과 az {ok}/{len(AZIS)}")
        pick, why = 0, {"ik": 0, "self": 0, "env": 0}
        per_pick = {}
        for el in (90, 75, 60, 45):
            p0 = pick
            for az in AZIS:
                q = pick_pose(tgt, el, az, T_WB)
                if q is None:
                    why["ik"] += 1
                elif self_clear(q, GRIPV) < args.clearance:
                    why["self"] += 1
                elif env_clear(q, GRIPV, tree, T_WB) < args.clearance:
                    why["env"] += 1
                else:
                    pick += 1
                    rep.setdefault(f"pick_el{el}", [float(v) for v in q])
            per_pick[el] = pick - p0
            if diag:
                print(f"      파지 el={el:>2}  통과 az {pick-p0}/{len(AZIS)}")
        if diag and acc["d"]:
            e_mm = np.array(acc["err"]) * 1000.0
            dd = np.array(acc["d"])
            print(f"    IK 위치오차  최대 {e_mm.max():.2f}mm  평균 {e_mm.mean():.2f}mm")
            print(f"    표면까지 거리 {dd.min()*100:.1f}~{dd.max()*100:.1f}cm "
                  f"(스윗스팟 20~30cm, 목표 {WORK_FOCUS*100:.0f}cm)")
        if diag:
            print(f"    스캔 실패내역: IK {ws['ik']}, 자가충돌 {ws['self']}, 환경 {ws['env']}")
            print(f"    피킹 실패내역: IK {why['ik']}, 자가충돌 {why['self']}, 환경 {why['env']}")
        return els_ok, n_scan, pick, per_el, per_pick, rep

    if args.diag is not None:
        for dz in args.base_dz:
            print(f"\n=== 진단 X={args.diag:+.2f}  base Z {T_WB0[2,3]+dz:.3f} ===")
            eval_layout(args.diag, dz, diag=True)
        app.close()
        return

    import csv
    import json
    xs = np.arange(args.x_min, args.x_max + 1e-9, args.x_step)
    summary, csv_rows, poses = [], [], {}
    for dz in args.base_dz:
        print(f"\n=== 로봇 base Z {T_WB0[2,3]+dz:.3f} m (dz={dz:+.3f}) ===")
        print(f"{'X(m)':>7}{'스캔':>6}{'피킹':>6}  el 30/40/50/60/70/80"
              f"   파지 90/75/60/45")
        rows = []
        for x in xs:
            els_ok, n_scan, pick, per_el, per_pick, rep = eval_layout(float(x), dz)
            rows.append((float(x), n_scan, pick, els_ok))
            csv_rows.append(dict(base_z=round(T_WB0[2, 3] + dz, 3), x=round(float(x), 3),
                                 scan=n_scan, pick=pick, n_el=len(els_ok),
                                 **{f"el{e}": per_el[e] for e in ELS},
                                 **{f"pick{e}": per_pick[e] for e in (90, 75, 60, 45)}))
            poses[f"z{T_WB0[2,3]+dz:.3f}_x{float(x):+.2f}"] = rep
            cov = "".join("O" if e in els_ok else "." for e in ELS)
            pk = "/".join(str(per_pick[e]) for e in (90, 75, 60, 45))
            mark = "  ← 현재 v3" if abs(x - 0.365) < 1e-6 else (
                   "  ← v4" if abs(x + 0.20) < 1e-6 else "")
            print(f"{x:>7.2f}{n_scan:>6}{pick:>6}  {cov} ({len(els_ok)}종)"
                  f"   {pk:>11}{mark}")
        # 스캔은 턴테이블 회전이 az 를 채우므로 el 대역별 1자세면 성립한다.
        # 따라서 우선순위: el 대역 수 → top-down 파지 가능 → 총 자세 여유
        good = [r for r in rows if r[2] > 0 and len(r[3]) >= 4]
        good.sort(key=lambda r: (-len(r[3]), -r[2], -r[1]))
        print("  추천: " + (", ".join(
            f"X={r[0]:+.2f}(el{len(r[3])}종/스캔{r[1]}/피킹{r[2]})" for r in good[:4])
            if good else "조건 만족 없음"))
        summary.append((dz, good[0] if good else None,
                        sum(r[1] for r in rows), sum(r[2] for r in rows)))

    print("\n=== base 높이 비교 ===")
    for dz, best, tot_s, tot_p in summary:
        b = f"최적 X={best[0]:+.2f} (el{len(best[3])}종/스캔{best[1]}/피킹{best[2]})" \
            if best else "만족 X 없음"
        print(f"  base Z {T_WB0[2,3]+dz:.3f} (dz={dz:+.3f}): {b}   "
              f"전체합 스캔 {tot_s} / 피킹 {tot_p}")

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    with open(args.csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(csv_rows[0]))
        w.writeheader()
        w.writerows(csv_rows)
    with open(args.dump_poses, "w") as f:
        json.dump(poses, f, indent=1)
    print(f"\nCSV  {args.csv}\n자세 {args.dump_poses}")
    app.close()


if __name__ == "__main__":
    main()
