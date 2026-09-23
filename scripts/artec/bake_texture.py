# scripts/artec/bake_texture.py
#
# 텍스처 **직접 굽기** — run 중 남긴 프레임(사진+uv+정점, `output/<RUN>/texture_frames/`)을
# 임의의 메시(우리 융합 결과든, Artec Studio 에서 정합을 다듬어 내보낸 메시든)에 투영해
# **정점 색**으로 입힌다. SDK 없이 돈다.
#
# 왜 — 저장된 sproj 는 프레임 uv 가 전부 NaN 이라 SDK `texturize()` 도 Studio 텍스처링도
# 실패한다(2026-09-23 실측, `docs/6_postprocess.md` §5). uv 는 live 에서만 유효하므로
# `_dump_texture_frames` 가 그때 뽑아 두고, 여기서 굽는다.
#
# 방법
#   1. 프레임마다 (정점 mm, 스캐너 프레임) ↔ (픽셀) 대응에서 **DLT 로 3×4 투영행렬**을 맞춘다
#      (캘리브·3D↔컬러 카메라 오프셋을 따로 알 필요가 없다 — 대응 자체가 그걸 담고 있다).
#   2. 메시 정점을 프레임 좌표로 옮겨(X_master = T_pre·T_frame·v 의 역) 투영하고,
#      **가시성**은 프레임 자신의 정점을 깊이 버퍼로 써서 판정(가려진 면 제외) + 법선이 카메라를 향해야.
#   3. 보이는 프레임들의 색을 입사각 가중으로 평균.
#   uv 의 v 방향(위→아래 / 아래→위)은 SDK 가 명세하지 않는다 — 두 규약으로 다 구워 **프레임 간
#   색 일관성**이 좋은 쪽을 자동 선택한다(투영행렬 잔차로는 못 가른다: y 뒤집기도 사영변환이다).
#
#   python scripts/artec/bake_texture.py --run 20260923_150000 --mesh output/20260923_150000/final.obj
#   python scripts/artec/bake_texture.py --run RUN --mesh studio_export.obj --out baked.ply
#     --mm/--m : 메시 단위 (기본 mm — SDK·Studio 내보내기)
#
# 출력: <out>.ply (정점 색) — CloudCompare/MeshLab 로 연다. 정점이 성기면 색도 성기다(v1 은
# UV 아틀라스가 아니라 정점 색이다).

from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                   # noqa: BLE001
        pass


# ── 프레임 ────────────────────────────────────────────────────────────────
class Frame:
    def __init__(self, npz_path: Path, v_flip: bool):
        import cv2
        z = np.load(npz_path, allow_pickle=True)
        self.name = npz_path.stem
        self.V = np.asarray(z["verts_mm"], float)                     # 스캐너 프레임 mm
        uv = np.asarray(z["uv"], float)
        self.H, self.W = [int(x) for x in z["image_hw"]]
        self.T_frame = np.asarray(z["T_frame_mm"], float)
        self.T_pre = np.asarray(z["T_pre_mm"], float)
        self.T_ws = self.T_pre @ self.T_frame                          # 스캐너 → master 월드 (mm)
        self.T_sw = np.linalg.inv(self.T_ws)
        jpg = npz_path.with_suffix(".jpg")
        img = cv2.imread(str(jpg), cv2.IMREAD_COLOR)
        self.img = None if img is None else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        fin = np.isfinite(uv).all(1)
        vy = (1.0 - uv[:, 1]) if v_flip else uv[:, 1]
        self.px = np.column_stack([uv[:, 0] * (self.W - 1), vy * (self.H - 1)])
        self.ok = fin & (self.px[:, 0] >= 0) & (self.px[:, 0] < self.W) & (self.px[:, 1] >= 0) & (self.px[:, 1] < self.H)
        self.P = None; self.C = None; self.rms = np.nan; self.tree = None; self.depth = None

    def fit(self, min_pts: int = 200) -> bool:
        """DLT 로 3×4 투영행렬. 잔차 큰 대응은 한 번 걸러 재적합."""
        from scipy.spatial import cKDTree
        if self.img is None or int(self.ok.sum()) < min_pts:
            return False
        X, x = self.V[self.ok], self.px[self.ok]
        sel = np.arange(len(X))
        if len(sel) > 6000:
            sel = np.random.default_rng(0).choice(len(sel), 6000, replace=False)
        P = _dlt(X[sel], x[sel])
        if P is None:
            return False
        res = np.linalg.norm(_proj(P, X) - x, axis=1)
        keep = res < max(3.0, 2.0 * np.median(res))
        if keep.sum() >= min_pts:
            sel2 = np.where(keep)[0]
            if len(sel2) > 6000:
                sel2 = np.random.default_rng(1).choice(sel2, 6000, replace=False)
            P2 = _dlt(X[sel2], x[sel2])
            if P2 is not None:
                P = P2
                res = np.linalg.norm(_proj(P, X) - x, axis=1)
        # 카메라 중심 = P 의 영공간; 깊이 부호는 실제 점이 앞에 오도록
        _, _, Vt = np.linalg.svd(P)
        C = Vt[-1]; C = C[:3] / C[3]
        w = (P @ np.c_[X, np.ones(len(X))].T)[2]
        if np.median(w) < 0:
            P = -P
        self.P, self.C, self.rms = P, C, float(np.sqrt(np.mean(res ** 2)))
        self.depth = np.linalg.norm(X - C, axis=1)
        self.tree = cKDTree(x)
        return True

    def sample(self, Xs: np.ndarray, Ns: np.ndarray, px_tol: float = 6.0, depth_tol_mm: float = 6.0):
        """메시 정점(스캐너 프레임 mm, 법선) → (색 (M,3) [0,1], 가중 (M,), 유효 mask)."""
        M = len(Xs)
        q = _proj(self.P, Xs)
        inside = (q[:, 0] >= 1) & (q[:, 0] < self.W - 1) & (q[:, 1] >= 1) & (q[:, 1] < self.H - 1)
        to_cam = self.C[None, :] - Xs
        dist = np.linalg.norm(to_cam, axis=1)
        cosi = np.einsum("ij,ij->i", Ns, to_cam / (dist[:, None] + 1e-9))
        facing = cosi > 0.15
        ok = inside & facing
        if not ok.any():
            return None, None, ok
        d_nn, i_nn = self.tree.query(q[ok], k=1, distance_upper_bound=px_tol)
        vis = np.isfinite(d_nn)
        vis[vis] &= dist[ok][vis] <= self.depth[i_nn[vis]] + depth_tol_mm
        ok_idx = np.where(ok)[0][vis]
        if len(ok_idx) == 0:
            return None, None, np.zeros(M, bool)
        qi = q[ok_idx]
        u = np.clip(np.rint(qi[:, 0]).astype(int), 0, self.W - 1)
        v = np.clip(np.rint(qi[:, 1]).astype(int), 0, self.H - 1)
        col = self.img[v, u].astype(float) / 255.0
        # 가중: 입사각² × 화면 가장자리 감쇠
        cx, cy = (self.W - 1) / 2.0, (self.H - 1) / 2.0
        edge = 1.0 - np.clip(np.maximum(np.abs(qi[:, 0] - cx) / cx, np.abs(qi[:, 1] - cy) / cy), 0, 1) ** 2
        w = (cosi[ok_idx] ** 2) * (0.3 + 0.7 * edge)
        mask = np.zeros(M, bool); mask[ok_idx] = True
        return col, w, mask


def _dlt(X, x):
    mX, sX = X.mean(0), max(X.std(0).mean(), 1e-9)
    mx, sx = x.mean(0), max(x.std(0).mean(), 1e-9)
    Xn = (X - mX) / sX; xn = (x - mx) / sx
    n = len(Xn)
    A = np.zeros((2 * n, 12))
    Xh = np.c_[Xn, np.ones(n)]
    A[0::2, 0:4] = Xh; A[0::2, 8:12] = -xn[:, [0]] * Xh
    A[1::2, 4:8] = Xh; A[1::2, 8:12] = -xn[:, [1]] * Xh
    try:
        _, _, Vt = np.linalg.svd(A, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    Pn = Vt[-1].reshape(3, 4)
    Tn = np.eye(4); Tn[:3, :3] /= sX; Tn[:3, 3] = -mX / sX          # X → Xn
    Tx = np.eye(3); Tx[:2, :2] /= sx; Tx[:2, 2] = -mx / sx          # x → xn
    P = np.linalg.inv(Tx) @ Pn @ Tn
    return P / (np.linalg.norm(P[2, :3]) + 1e-12)


def _proj(P, X):
    p = P @ np.c_[X, np.ones(len(X))].T
    return (p[:2] / (p[2] + 1e-12)).T


# ── 굽기 ──────────────────────────────────────────────────────────────────
def load_frames(frames_dir: Path, v_flip: bool, log=print):
    fr = []
    for p in sorted(frames_dir.rglob("*.npz")):
        try:
            f = Frame(p, v_flip)
        except Exception as e:                              # noqa: BLE001
            log(f"  ✘ {p.name}: {e}"); continue
        if f.fit():
            fr.append(f)
    return fr


def bake(mesh_verts_mm: np.ndarray, mesh_normals: np.ndarray, frames: list, log=print):
    """→ (색 (N,3) [0,1], 관측 수 (N,)). 색 없음 = 관측 0."""
    N = len(mesh_verts_mm)
    acc = np.zeros((N, 3)); wsum = np.zeros(N); cnt = np.zeros(N, int)
    for k, f in enumerate(frames):
        Xs = (f.T_sw[:3, :3] @ mesh_verts_mm.T).T + f.T_sw[:3, 3]
        Ns = (f.T_sw[:3, :3] @ mesh_normals.T).T
        col, w, m = f.sample(Xs, Ns)
        if col is None:
            continue
        acc[m] += col * w[:, None]; wsum[m] += w; cnt[m] += 1
    seen = wsum > 0
    rgb = np.zeros((N, 3)); rgb[seen] = acc[seen] / wsum[seen][:, None]
    return rgb, cnt


def consistency(mesh_verts_mm, mesh_normals, frames, n_sample=400, log=print) -> float:
    """프레임 간 색 일관성 = 2회 이상 관측된 정점의 색 표준편차 평균 (작을수록 좋음)."""
    N = len(mesh_verts_mm)
    idx = np.random.default_rng(0).choice(N, min(N, 4000), replace=False)
    V, Nn = mesh_verts_mm[idx], mesh_normals[idx]
    obs = [[] for _ in range(len(idx))]
    for f in frames:
        Xs = (f.T_sw[:3, :3] @ V.T).T + f.T_sw[:3, 3]
        Ns = (f.T_sw[:3, :3] @ Nn.T).T
        col, w, m = f.sample(Xs, Ns)
        if col is None:
            continue
        for j, c in zip(np.where(m)[0], col):
            obs[j].append(c)
    stds = [np.std(np.asarray(o), axis=0).mean() for o in obs if len(o) >= 2]
    return float(np.mean(stds[:n_sample])) if stds else float("nan")


def run(mesh_path: Path, frames_dir: Path, out_path: Path, mesh_in_m: bool, v_flip=None, log=print) -> int:
    import open3d as o3d
    t0 = time.time()
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if len(mesh.vertices) == 0:
        log(f"✘ 메시를 못 읽음: {mesh_path}"); return 2
    mesh.compute_vertex_normals()
    V = np.asarray(mesh.vertices, float) * (1000.0 if mesh_in_m else 1.0)
    Nn = np.asarray(mesh.vertex_normals, float)
    log(f"메시 {mesh_path.name}: 정점 {len(V):,} · 삼각형 {len(mesh.triangles):,} · 범위 "
        f"{np.ptp(V, axis=0).round(0)} mm")
    n_npz = len(list(frames_dir.rglob("*.npz")))
    if n_npz == 0:
        log(f"✘ 프레임이 없다: {frames_dir}  (run 이 `dump_texture_frames=True` 로 돌았나?)"); return 2
    # v 규약 자동 선택
    if v_flip is None:
        best = None
        for flip in (True, False):
            fr = load_frames(frames_dir, flip, log)
            if not fr:
                continue
            c = consistency(V, Nn, fr[: max(8, len(fr) // 3)])
            log(f"  v 규약 {'bottom-up(1−v)' if flip else 'top-down(v)'}: 프레임 {len(fr)} · "
                f"평균 재투영 RMS {np.mean([f.rms for f in fr]):.2f}px · 색 일관성 {c:.4f}")
            if best is None or (np.isfinite(c) and c < best[0]):
                best = (c, flip, fr)
        if best is None:
            log("✘ 투영행렬을 맞출 수 있는 프레임이 없다"); return 2
        _, v_flip, frames = best
        log(f"→ v 규약: {'bottom-up(1−v)' if v_flip else 'top-down(v)'} 채택")
    else:
        frames = load_frames(frames_dir, v_flip, log)
    log(f"프레임 {len(frames)}/{n_npz} 적합 · RMS 중앙 {np.median([f.rms for f in frames]):.2f}px")
    rgb, cnt = bake(V, Nn, frames, log)
    seen = cnt > 0
    log(f"색 입힌 정점 {seen.sum():,}/{len(V):,} ({seen.mean()*100:.1f}%) · 정점당 관측 중앙 "
        f"{np.median(cnt[seen]) if seen.any() else 0:.0f}회 · {time.time()-t0:.0f}s")
    rgb[~seen] = 0.5                                   # 못 본 정점은 회색
    mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(rgb, 0, 1))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(out_path), mesh, write_vertex_colors=True)
    log(f"→ {out_path}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="RUN_TS — output/<RUN>/texture_frames/ 를 쓴다")
    ap.add_argument("--mesh", required=True, help="색을 입힐 메시 (obj/ply). 좌표는 master 스캔월드")
    ap.add_argument("--out", default=None, help="출력 ply (기본 <mesh>_baked.ply)")
    ap.add_argument("--frames", default=None, help="프레임 폴더 (기본 output/<RUN>/texture_frames)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--mm", action="store_true", help="메시 단위 mm (기본)")
    g.add_argument("--m", action="store_true", help="메시 단위 m")
    ap.add_argument("--v-flip", choices=["auto", "yes", "no"], default="auto",
                    help="uv 의 v 규약: yes=bottom-up(1−v) · no=top-down · auto=색 일관성으로 선택")
    a = ap.parse_args()
    mesh = Path(a.mesh)
    frames = Path(a.frames) if a.frames else _ROOT / "output" / a.run / "texture_frames"
    out = Path(a.out) if a.out else mesh.with_name(mesh.stem + "_baked.ply")
    vf = None if a.v_flip == "auto" else (a.v_flip == "yes")
    return run(mesh, frames, out, mesh_in_m=bool(a.m), v_flip=vf)


if __name__ == "__main__":
    sys.exit(main())
