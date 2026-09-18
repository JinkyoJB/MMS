#!/usr/bin/env python
"""plot_band_plan.py — 밴드 분할 계획을 그림으로 (`docs/figures/lookaround/fig4_bands.png`).

왜 스크립트로 두나
------------------
이 그림은 원래 생성기 없이 파일만 리포에 있었다. 그래서 코드가 바뀌어도 아무도 모르게
낡았다 — 2026-09-18 확인 시점에 **세 가지가 실제와 달랐다**:

    · 모든 밴드가 `el=20°`   → el 20 은 도달 자세가 없어 후보에서 뺐다(2026-09-17)
    · "스캔 순서 = safe-first"  → z 단조로 바뀌었다(2026-09-17, 밴드 = 한 IScan)
    · h=293mm · 4밴드 · band_h=136mm → 현재 코드로는 h=286mm · 5밴드+윗면 · 156mm

그림은 문서보다 더 오래 살아남고 더 잘 믿긴다. 그래서 **계획기를 실제로 돌려서** 그린다.

입력
----
`scripts/sim/extract_testset_points.py` 가 만든 점군 캐시. 없으면 먼저 한 번:

    env -u PYTHONPATH $ISAAC scripts/sim/extract_testset_points.py

사용
----
    env -u PYTHONPATH $MMS_PYTHON scripts/sim/plot_band_plan.py                 # 기본(세제)
    env -u PYTHONPATH $MMS_PYTHON scripts/sim/plot_band_plan.py --obj spray_can
    env -u PYTHONPATH $MMS_PYTHON scripts/sim/plot_band_plan.py --obj mug --out /tmp/a.png
"""
from __future__ import annotations

import argparse
import contextlib
import glob
import io
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

CACHE = "scripts/sim/log/testset_points"
OUT_DIR = "docs/figures/lookaround"


def _load(obj: str):
    """부분이름 매칭으로 점군 캐시 하나를 연다. (이름, 점군, scene.json)"""
    files = sorted(glob.glob(os.path.join(CACHE, "*.npz")))
    if not files:
        raise SystemExit(
            f"[plot] 점군 캐시가 없다: {CACHE}\n"
            f"       먼저: env -u PYTHONPATH $ISAAC "
            f"scripts/sim/extract_testset_points.py")
    hit = [f for f in files if obj.lower() in os.path.basename(f).lower()]
    if not hit:
        names = ", ".join(os.path.basename(f)[:-4] for f in files)
        raise SystemExit(f"[plot] '{obj}' 와 맞는 물체가 없다. 있는 것: {names}")
    f = hit[0]
    sc = json.load(open(os.path.join(CACHE, "scene.json")))
    return os.path.basename(f)[:-4], np.asarray(np.load(f)["pts"], float), sc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--obj", default="laundry_detergent", help="물체 부분이름")
    ap.add_argument("--out", default=None, help="출력 png (기본 fig4_bands.png)")
    ap.add_argument("--els", default="30,40,50,60,70", help="채점할 고도각 후보")
    ap.add_argument("--dpi", type=int, default=140)
    a = ap.parse_args()

    from utils.nbv import lookaround as p1

    name, pts_raw, sc = _load(a.obj)
    axis_xy = np.asarray(sc["axis_xy"], float)
    sensor = p1.SensorModel()

    pts = p1.voxel_downsample(
        p1.crop_object_points(pts_raw, axis_xy, sc["disc_top_z"]), sensor.voxel_m)
    if len(pts) < 100:
        raise SystemExit(f"[plot] 크롭 후 점 부족({len(pts)})")
    nrm = p1.estimate_outward_normals(pts, axis_xy)

    els = tuple(float(x) for x in a.els.split(",") if x.strip())
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        plan = p1.plan_lookaround_viewpoints(pts, nrm, axis_xy, sensor, els=els)
    log = buf.getvalue()
    print(log, end="")

    # ── 그림용 수치 ──────────────────────────────────────────────────────
    z = pts[:, 2]
    h_mm = (np.quantile(z, 0.995) - np.quantile(z, 0.005)) * 1000.0
    band_h_mm = None
    for line in log.splitlines():                     # 채택된 마지막 밴드 로그
        if "band_h=" in line:
            band_h_mm = float(line.split("band_h=")[1].split("mm")[0])
    r = np.linalg.norm(pts[:, :2] - axis_xy[None, :], axis=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for f in ("NanumGothic", "NanumBarunGothic", "Malgun Gothic"):
        try:
            matplotlib.rcParams["font.family"] = f
            break
        except Exception:                                       # noqa: BLE001
            continue
    matplotlib.rcParams["axes.unicode_minus"] = False

    fig, ax = plt.subplots(figsize=(10.8, 7.0))
    # 물체 **단면** — 축 기준 x 오프셋 vs z. 반경(|r|)을 미러링하면 비대칭 물체
    # (손잡이 달린 세제)가 두 개로 보인다. 실제 x 를 쓰면 실루엣이 그대로 나온다.
    x_mm = (pts[:, 0] - axis_xy[0]) * 1000.0
    ax.scatter(x_mm, z * 1000.0,
               s=1.2, c="#9ec9f0", alpha=0.45, linewidths=0, zorder=1)
    ax.axvline(0.0, ls="--", lw=1.0, c="#888", zorder=2)
    ax.text(3, float(z.max() * 1000) + 8, "턴테이블 축", color="#666", fontsize=10)

    cmap = plt.get_cmap("viridis")
    n = len(plan.poses)
    n_band = 0                       # 밴드 번호는 **뚜껑을 빼고** 센다 — 캡처
    #                                  순서(i)로 세면 뚜껑 뒤 밴드가 한 칸 밀린다
    prev_tz = None
    for i, (vp, ev) in enumerate(zip(plan.poses, plan.evals), start=1):
        col = cmap(0.12 + 0.72 * (i - 1) / max(n - 1, 1))
        tz = float(vp.target_z) * 1000.0
        so = float(vp.standoff) * 1000.0
        el = float(vp.el_deg)
        # 카메라 위치 (축 기준 수평거리·높이) — el 이 실제로 보이게 그린다
        cx = so * np.cos(np.radians(el))
        cz = tz + so * np.sin(np.radians(el)) * (1.0 if tz >= 0 else 1.0)
        ax.plot([0.0, cx], [tz, cz], ls=":", lw=1.6, color=col, zorder=3)
        ax.scatter([cx], [cz], marker="v", s=150, color=col, zorder=4)
        is_cap = bool(getattr(vp, "is_cap", False))
        if is_cap:
            tag = "윗면 보강"
        else:
            n_band += 1
            tag = f"밴드 {n_band}"
        # ⚠·★ 같은 기호는 나눔고딕에 없어 두부(□)로 렌더된다 — 글자로 쓴다
        risk = ("  (추적위험)" if float(ev.min_fill_cm2) < p1.FILL_MIN_CM2
                else "")
        ax.annotate(f"{i}. {tag}   el={el:.0f}°   "
                    f"minfill={ev.min_fill_cm2:.0f}cm²{risk}",
                    xy=(cx, cz), xytext=(11, 0), textcoords="offset points",
                    va="center", fontsize=10.5, color="#222")
        # tz 눈금 — 겹치면(뚜껑과 끝 밴드가 가깝다) 한 칸 띄운다
        dy = -4 if prev_tz is None or abs(tz - prev_tz) > 12 else 9
        ax.annotate(f"tz={tz:.0f}", xy=(0, tz), xytext=(-48, dy),
                    textcoords="offset points", fontsize=9, color=col)
        prev_tz = tz

    order = " → ".join(f"{float(v.target_z)*1000:.0f}" for v in plan.poses)
    sub = ("스캔 순서 = **z 단조** (밴드 전체가 한 IScan — 인접 자세가 겹쳐야 SLAM 이 이어진다)"
           .replace("**", ""))
    ax.set_title(f"{name}(h={h_mm:.0f}mm) — {plan.note}"
                 + (f"\n{sub}\ntz 순서: {order} mm" if n > 1 else f"\n{sub}"),
                 fontsize=12.5, loc="left")
    ax.set_xlabel("축 기준 수평거리 (mm)")
    ax.set_ylabel("z (mm)")
    ax.grid(alpha=0.25)
    ax.set_axisbelow(True)
    # 라벨이 축 오른쪽 밖으로 나가므로 여백을 준다 (라벨 길이에 비례)
    x0, x1 = ax.get_xlim()
    ax.set_xlim(x0, x1 + 0.62 * (x1 - x0))

    out = a.out or os.path.join(OUT_DIR, "fig4_bands.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=a.dpi)
    fig.savefig(os.path.splitext(out)[0] + ".svg")
    print(f"[plot] 저장: {out}  (+ .svg)")
    print(f"[plot]   {name}  h={h_mm:.0f}mm  band_h="
          f"{'?' if band_h_mm is None else f'{band_h_mm:.0f}'}mm  자세 {n}개")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
