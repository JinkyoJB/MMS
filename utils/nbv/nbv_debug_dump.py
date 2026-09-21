"""nbv_debug_dump.py — nbv 반복마다 **무엇을 겨냥해 무엇을 얻었나** 를 남긴다. sim·real 공용.

왜 있나 (2026-09-18)
-------------------
nbv 는 반복마다 gap 을 고르고(겨냥), 부분 스윕으로 찍고(수집), master 에 붙인다(정합).
이 셋 중 어디가 틀렸는지 로그 숫자만으로는 알 수 없었다 — "gap 겨냥 채택" 이 찍혀도
카메라가 정말 그 gap 을 보는지, 찍힌 점이 그 gap 을 메우는지, 정합이 옆으로 붙였는지는
그림으로 봐야 한다. Isaac 뷰포트 오버레이는 sim 에만 있고 real 엔 없다.

무엇을 남기나 — 반복마다 `output/debug/nbv/nbv_NN.npz` + `nbv_NN.png`
    master        정합 대상(누적 점군, base, m)          gaps_p/n/L  후보 전부
    chosen        고른 후보 인덱스(-1=없음), mode        ensure | frontier | fallback
    eye/target    카메라 위치·조준점 (base, m)           theta/span  턴테이블 중심각·스윕 폭
    patch_raw     찍힌 점(기구학 자리), patch_aligned   정합 후     result  ok/reason/fitness/Δt/Δr

보는 법
    python scripts/nbv/nbv_debug_view.py            # Filament 창, 디렉터리 tail, [ ] 로 반복 이동
    PNG 는 창 없이도 남는다(위에서 본 XY · 옆에서 본 XZ).

끄기: MMS_NBV_DEBUG=0.  디렉터리: MMS_NBV_DEBUG_DIR (기본 output/debug/nbv).
파이프라인을 절대 멈추지 않는다 — 모든 공개 메서드는 예외를 삼킨다.
"""
from __future__ import annotations
import json
import math
import os
import time

import numpy as np

MAX_MASTER_PTS = 200_000
MAX_PATCH_PTS = 150_000


class NbvDebugDump:
    def __init__(self, out_dir: str = None, enabled: bool = None, tag: str = ""):
        if enabled is None:
            enabled = os.environ.get("MMS_NBV_DEBUG", "1") == "1"
        self.enabled = bool(enabled)
        self.dir = out_dir or os.environ.get("MMS_NBV_DEBUG_DIR", "output/debug/nbv")
        if not os.path.isabs(self.dir):          # cwd 가 바뀌어도 리포 밑에 쌓이게
            _root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            self.dir = os.path.join(_root, self.dir)
        self.tag = tag or time.strftime("%H%M%S")
        self.k = 0
        self._pending = None
        if self.enabled:
            try:
                os.makedirs(self.dir, exist_ok=True)
            except Exception:                                   # noqa: BLE001
                self.enabled = False

    # ── 계획 시점 ───────────────────────────────────────────────────────
    def plan(self, *, gaps, chosen, mode, eye, target, theta, span_deg,
             master_pts, axis_pt, up_sign=+1.0, note=""):
        """겨냥 결정을 기록해 둔다(파일은 patch() 에서 같이 쓴다)."""
        if not self.enabled:
            return
        try:
            self.k += 1
            rec = dict(mode=str(mode), note=str(note),
                       eye=np.asarray(eye, float).reshape(3),
                       target=np.asarray(target, float).reshape(3),
                       theta=float(theta), span_deg=float(span_deg),
                       axis=np.asarray(axis_pt, float).reshape(3), up_sign=float(up_sign),
                       chosen=int(chosen if chosen is not None else -1))
            G = list(gaps or [])
            rec["gaps_p"] = np.array([np.asarray(c.p_O, float) for c in G]).reshape(-1, 3)
            rec["gaps_n"] = np.array([np.asarray(c.n_O, float) for c in G]).reshape(-1, 3)
            rec["gaps_L"] = np.array([float(c.L) for c in G], float)
            M = np.asarray(master_pts, float) if master_pts is not None else np.zeros((0, 3))
            if len(M) > MAX_MASTER_PTS:
                M = M[np.random.default_rng(0).choice(len(M), MAX_MASTER_PTS, replace=False)]
            rec["master"] = M.astype(np.float32)
            self._pending = rec
        except Exception as e:                                  # noqa: BLE001
            print(f"  [nbv-dbg] plan 기록 실패({e}) — 건너뜀")
            self._pending = None

    # ── 캡처·정합 뒤 ────────────────────────────────────────────────────
    def patch(self, *, raw_pts, aligned_pts, result=None):
        """수집·정합 결과를 붙여 파일로 쓴다. result = IcpResult 또는 None."""
        if not self.enabled or self._pending is None:
            return
        rec, self._pending = self._pending, None
        try:
            R = np.asarray(raw_pts, float) if raw_pts is not None else np.zeros((0, 3))
            A = np.asarray(aligned_pts, float) if aligned_pts is not None else R
            if len(R) > MAX_PATCH_PTS:
                idx = np.random.default_rng(1).choice(len(R), MAX_PATCH_PTS, replace=False)
                R = R[idx]; A = A[idx] if len(A) == len(np.asarray(raw_pts)) else A
            rec["patch_raw"] = R.astype(np.float32)
            rec["patch_aligned"] = A.astype(np.float32)
            res = {}
            if result is not None:
                res = dict(ok=bool(result.ok), reason=str(result.reason),
                           fitness=float(result.fitness), rmse_m=float(result.rmse),
                           dt_m=float(result.delta_translation_m),
                           dr_deg=float(result.delta_rotation_deg))
            rec["result"] = json.dumps(res)
            path = os.path.join(self.dir, f"nbv_{self.tag}_{self.k:02d}.npz")
            # ★ 임시 파일은 .npz 로 끝나면 안 된다 — 뷰어가 `nbv_*.npz` 를 tail 하므로
            #   쓰는 중인 파일을 집어 BadZipFile 로 죽었다(2026-09-18). `.part` 로 쓰고 rename.
            tmp = path + ".part"
            with open(tmp, "wb") as fh:
                np.savez_compressed(fh, **rec)
            os.replace(tmp, path)
            self._png(path[:-4] + ".png", rec, res)
            print(f"  [nbv-dbg] {os.path.basename(path)} — {rec['mode']} gap "
                  f"{rec['chosen']} · 점 {len(R):,} · "
                  f"{'정합 ' + ('ok' if res.get('ok') else res.get('reason', '-')) if res else '정합 없음'}")
        except Exception as e:                                  # noqa: BLE001
            print(f"  [nbv-dbg] 기록 실패({e}) — 건너뜀")

    # ── PNG (창 없이 보는 용) ───────────────────────────────────────────
    @staticmethod
    def _png(path, rec, res):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            return
        M, R, A = rec["master"], rec["patch_raw"], rec["patch_aligned"]
        eye, tgt, ax0, u = rec["eye"], rec["target"], rec["axis"], rec["up_sign"]
        gp, gL, ch = rec["gaps_p"], rec["gaps_L"], rec["chosen"]
        fig, axs = plt.subplots(1, 2, figsize=(13, 6))
        for k, (ia, ib, lab) in enumerate([((0, 1), None, "top (x,y)"), ((0, 2), None, "side (x,z)")]):
            a = axs[k]; i, j = ia
            def P2(P):
                return (P[:, i], P[:, j] * (u if j == 2 else 1.0))
            if len(M): a.scatter(*P2(M), s=0.3, c="0.75", label="master")
            if len(R): a.scatter(*P2(R), s=0.6, c="gold", label="patch raw")
            if len(A): a.scatter(*P2(A), s=0.6, c="limegreen", label="patch aligned")
            if len(gp):
                a.scatter(*P2(gp), s=np.clip(gL * 4000, 10, 200), facecolors="none", edgecolors="orange", label="gaps")
                if 0 <= ch < len(gp):
                    a.scatter(gp[ch, i], gp[ch, j] * (u if j == 2 else 1.0), s=180, c="red", marker="x", label="chosen")
            a.plot([eye[i], tgt[i]], [eye[j] * (u if j == 2 else 1.0), tgt[j] * (u if j == 2 else 1.0)], "b-", lw=1.5)
            a.scatter(eye[i], eye[j] * (u if j == 2 else 1.0), s=60, c="blue", marker="^", label="camera")
            a.scatter(ax0[i], ax0[j] * (u if j == 2 else 1.0), s=30, c="k", marker="+")
            a.set_aspect("equal"); a.set_title(lab); a.grid(alpha=0.2)
        axs[0].legend(loc="upper right", fontsize=8, markerscale=3)
        txt = (f"{rec['mode']}  chosen={ch}  theta={math.degrees(rec['theta']) % 360:.0f}deg "
               f"span={rec['span_deg']:.0f}deg  patch={len(R):,}pt")
        if res:
            txt += (f"\nicp ok={res['ok']} fitness={res['fitness']:.2f} rmse={res['rmse_m']*1000:.2f}mm "
                    f"dt={res['dt_m']*1000:.1f}mm dr={res['dr_deg']:.1f}deg  {res['reason']}")
        if rec.get("note"):
            txt += f"\n{rec['note']}"
        fig.suptitle(txt, fontsize=9)
        plt.tight_layout(); fig.savefig(path, dpi=100); plt.close(fig)
