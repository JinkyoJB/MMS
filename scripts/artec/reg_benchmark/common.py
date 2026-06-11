"""
reg_benchmark 공통 토대.

- master.sproj 에서 3개 ScanSession 을 읽어 **세션 좌표계** 점군(미터)으로 누적.
  (각 frame 을 그 frame_transformation 으로 scan 좌표계에 매핑 — 세션 내부는
   SLAM 으로 이미 정합돼 있음. 세션 *사이* 변환이 6개 방법이 찾으려는 미지수.)
- voxel downsample + normal estimate + (옵션) per-vertex 색(텍스처 uv 샘플).
- npz/ply 캐시 → 매번 sproj 파싱(무겁다) 안 하도록.
- 평가지표: 모든 방법이 "공통 좌표계로 정렬된 세션 점군"을 내놓으면
  pairwise fitness / inlier-RMSE / chamfer 로 동일 비교.

좌표/단위: Artec native = mm. 여기선 **미터**로 통일(open3d 관례).
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# 한국어 Windows 콘솔(cp949) 은 em-dash/°/→ 등을 못 찍어 UnicodeEncodeError.
# 출력 인코딩을 UTF-8(replace) 로 재설정.
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ── 데이터/출력 경로 ───────────────────────────────────────────────────
DEFAULT_SPROJ = (PROJECT_ROOT / "output" / "scan_raw" /
                 "20260520_153357" / "master.sproj")
OUT_DIR = PROJECT_ROOT / "output" / "reg_benchmark"
CACHE_DIR = OUT_DIR / "cache"
OVERLAY_DIR = OUT_DIR / "overlays"     # 정합 결과 빨강/파랑 오버레이 PLY
FIG_DIR = OUT_DIR / "figures"          # 비교 그리드 PNG (deliverable)
MESH_DIR = OUT_DIR / "meshes"          # SDK 텍스처 OBJ
RESULTS_DIR = OUT_DIR / "results"      # CSV + MD
for _d in (OVERLAY_DIR, FIG_DIR, MESH_DIR, RESULTS_DIR):
    _d.mkdir(parents=True, exist_ok=True)

# ── 전처리 파라미터 (한 곳에서 관리) ───────────────────────────────────
VOXEL_M = 0.002            # 2mm — 전처리 downsample
NORMAL_RADIUS_M = 0.006    # normal 추정 반경
FEATURE_VOXEL_M = 0.004    # FPFH/FGR 용 coarse downsample
ICP_THRESH_M = 0.004       # 공통 ICP refine correspondence 거리
EVAL_THRESHES_M = (0.003, 0.005)   # fitness/RMSE 평가 임계값들


# ─────────────────────────────────────────────────────────────────────
# 1) sproj → 세션 점군
# ─────────────────────────────────────────────────────────────────────

def _sample_vertex_colors(verts_frame: np.ndarray, frame) -> Optional[np.ndarray]:
    """frame 의 uv + image 로 per-vertex RGB(float 0..1). 실패 시 None."""
    try:
        if not frame.is_textured() or not frame.has_image():
            return None
        uv = frame.uv()
        img = frame.image()        # (H,W,3) uint8 RGB
        if uv is None or img is None or len(uv) != len(verts_frame):
            return None
        H, W = img.shape[:2]
        # uv 원점/스케일 관례: [0,1], v 가 top-origin 이라고 가정(틀려도 색만 영향).
        u = np.clip((uv[:, 0] * (W - 1)).round().astype(int), 0, W - 1)
        v = np.clip((uv[:, 1] * (H - 1)).round().astype(int), 0, H - 1)
        col = img[v, u].astype(np.float64) / 255.0
        return col
    except Exception:
        return None


def remove_turntable_plane(pcd: o3d.geometry.PointCloud,
                           dist: float = 0.003, min_frac: float = 0.10,
                           above_margin: float = 0.003) -> o3d.geometry.PointCloud:
    """가장 큰 평면(턴테이블)을 RANSAC 으로 찾아 평면+아래쪽 점을 제거하고
    물체(평면 위)만 남긴다. 평면 inlier 가 min_frac 미만이면 그대로 둔다.

    법선은 비-inlier(물체) 다수가 있는 쪽을 + 로 정렬 → signed>above_margin 만 유지."""
    P = np.asarray(pcd.points)
    if len(P) < 200:
        return pcd
    plane, inl = pcd.segment_plane(dist, 3, 1000)
    if len(inl) < min_frac * len(P):
        return pcd
    a, b, c, d = plane
    n = np.array([a, b, c], float)
    signed = (P @ n + d) / (np.linalg.norm(n) + 1e-12)
    mask_in = np.zeros(len(P), bool); mask_in[inl] = True
    # 물체(=비평면) 점들의 평균 부호로 + 방향 결정
    obj_side = np.sign(np.mean(signed[~mask_in])) or 1.0
    s = signed * obj_side
    keep = s > above_margin                       # 평면 위(물체)만
    out = pcd.select_by_index(np.where(keep)[0])
    return out


def extract_session_cloud(scan, with_color: bool = True,
                          voxel_m: float = VOXEL_M,
                          drop_plane: bool = True) -> o3d.geometry.PointCloud:
    """ScanHandle → 세션 좌표계 점군(미터, downsampled, normals[, colors]).
    drop_plane=True 면 턴테이블 평면 제거(정합/메시 오염 방지)."""
    all_v = []
    all_c = []
    have_color = with_color
    for i in range(scan.frame_count()):
        f = scan.get_frame(i)
        v = np.asarray(f.vertices(), dtype=np.float64)
        if v.size == 0:
            continue
        T = np.asarray(scan.get_frame_transformation(i), float)
        vw = v @ T[:3, :3].T + T[:3, 3]      # frame→scan, mm
        all_v.append(vw)
        if have_color:
            c = _sample_vertex_colors(v, f)
            if c is None:
                have_color = False
                all_c = []
            else:
                all_c.append(c)
    if not all_v:
        return o3d.geometry.PointCloud()
    V = np.vstack(all_v) / 1000.0            # mm → m
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(V)
    if have_color and all_c:
        pcd.colors = o3d.utility.Vector3dVector(np.vstack(all_c))
    pcd = pcd.voxel_down_sample(voxel_m)
    if drop_plane:
        pcd = remove_turntable_plane(pcd)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=NORMAL_RADIUS_M, max_nn=30))
    pcd.normalize_normals()
    return pcd


# ─────────────────────────────────────────────────────────────────────
# 2) 캐시 (sproj 파싱 회피)
# ─────────────────────────────────────────────────────────────────────

def load_sessions(sproj: Path = DEFAULT_SPROJ, with_color: bool = True,
                  use_cache: bool = True) -> list[o3d.geometry.PointCloud]:
    """3개 세션 점군 로드. 캐시(.ply)가 있으면 그걸 사용."""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    cache_paths = sorted(CACHE_DIR.glob("session_*.ply"))
    if use_cache and len(cache_paths) >= 1:
        sess = [o3d.io.read_point_cloud(str(p)) for p in cache_paths]
        for p in sess:
            if not p.has_normals():
                p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
                    radius=NORMAL_RADIUS_M, max_nn=30))
        print(f"[common] cache 사용 — {len(sess)} sessions from {CACHE_DIR}")
        return sess

    from mms_artec.sensor.artec_client import ArtecClient
    print(f"[common] sproj 파싱: {sproj}")
    t0 = time.perf_counter()
    entries = ArtecClient.load_project(str(sproj))
    sessions = []
    for e in entries:
        scan = getattr(e, "scan", None)
        if scan is None or scan.frame_count() == 0:
            continue
        pcd = extract_session_cloud(scan, with_color=with_color)
        sessions.append(pcd)
        idx = len(sessions) - 1
        outp = CACHE_DIR / f"session_{idx}.ply"
        o3d.io.write_point_cloud(str(outp), pcd)
        print(f"  session[{idx}] pts={len(pcd.points):,} "
              f"color={pcd.has_colors()} → {outp.name}")
    print(f"[common] {len(sessions)} sessions in {time.perf_counter()-t0:.1f}s")
    return sessions


# ─────────────────────────────────────────────────────────────────────
# 3) 지표
# ─────────────────────────────────────────────────────────────────────

@dataclass
class PairMetric:
    pair: str                 # "1->0"
    fitness: dict             # {thresh_m: fitness}
    rmse: dict                # {thresh_m: inlier_rmse_m}
    chamfer_m: float
    rot_deg: float            # 추정 변환의 회전각
    expected_deg: float       # 기대 회전(90/180)
    rot_err_deg: float
    # pseudo-GT(hint) 대비 변환 편차 — 핵심 정확도 지표.
    gt_rot_err_deg: float = float("nan")   # 추정 R vs hint R
    gt_t_err_mm: float = float("nan")      # 추정 t vs hint t


def evaluate_pair(src_aligned: o3d.geometry.PointCloud,
                  ref: o3d.geometry.PointCloud,
                  pair: str, T_est: np.ndarray,
                  expected_deg: float,
                  T_gt: Optional[np.ndarray] = None) -> PairMetric:
    """이미 정렬된 src 와 ref 사이 overlap 품질 + 회전 sanity + GT 편차."""
    fitness, rmse = {}, {}
    for th in EVAL_THRESHES_M:
        ev = o3d.pipelines.registration.evaluate_registration(
            src_aligned, ref, th, np.eye(4))
        fitness[th] = ev.fitness
        rmse[th] = ev.inlier_rmse
    cham = _chamfer(src_aligned, ref)
    ang = _rot_angle_deg(T_est[:3, :3])
    gt_r = gt_t = float("nan")
    if T_gt is not None:
        dR = T_est[:3, :3] @ T_gt[:3, :3].T
        gt_r = _rot_angle_deg(dR)
        gt_t = float(np.linalg.norm(T_est[:3, 3] - T_gt[:3, 3]) * 1000.0)
    return PairMetric(
        pair=pair, fitness=fitness, rmse=rmse, chamfer_m=cham,
        rot_deg=ang, expected_deg=expected_deg,
        rot_err_deg=abs(_ang_diff(ang, expected_deg)),
        gt_rot_err_deg=gt_r, gt_t_err_mm=gt_t)


def _chamfer(a: o3d.geometry.PointCloud, b: o3d.geometry.PointCloud) -> float:
    """대칭 chamfer 거리(미터) — 평균 nearest-neighbor."""
    da = np.asarray(a.compute_point_cloud_distance(b))
    db = np.asarray(b.compute_point_cloud_distance(a))
    if len(da) == 0 or len(db) == 0:
        return float("nan")
    return float((da.mean() + db.mean()) / 2.0)


def _rot_angle_deg(R: np.ndarray) -> float:
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _ang_diff(a: float, b: float) -> float:
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


# ─────────────────────────────────────────────────────────────────────
# 4) 공통 ICP refine (global 방법들 뒤에 동일 적용 → 공정 비교)
# ─────────────────────────────────────────────────────────────────────

def icp_refine(src: o3d.geometry.PointCloud, ref: o3d.geometry.PointCloud,
               T_init: np.ndarray, thresh_m: float = ICP_THRESH_M,
               colored: bool = False) -> np.ndarray:
    if colored and src.has_colors() and ref.has_colors():
        res = o3d.pipelines.registration.registration_colored_icp(
            src, ref, thresh_m, T_init,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=60))
    else:
        res = o3d.pipelines.registration.registration_icp(
            src, ref, thresh_m, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=60))
    return res.transformation


# ─────────────────────────────────────────────────────────────────────
# 5) 기대 회전각 (meta hint 로부터)
# ─────────────────────────────────────────────────────────────────────

def expected_rotations(sproj: Path = DEFAULT_SPROJ) -> dict[int, float]:
    """session_idx → 기대 회전각(deg). meta.npz 의 T_pres 회전성분에서.
    없으면 {1:90, 2:180} 가정."""
    meta_p = sproj.parent / "meta.npz"
    out = {1: 90.0, 2: 180.0}
    if not meta_p.exists():
        return out
    try:
        m = np.load(meta_p, allow_pickle=False)
        idxs = m["scan_indices"]; T = m["T_pres"]
        for k in range(len(idxs)):
            out[int(idxs[k])] = _rot_angle_deg(np.asarray(T[k])[:3, :3])
    except Exception:
        pass
    return out


def hint_transforms(sproj: Path = DEFAULT_SPROJ) -> dict[int, np.ndarray]:
    """session_idx → hint T_pre (4x4, translation 미터). pseudo-GT.

    meta.npz 의 T_pres 는 scan-world→master-world 매핑(= session_i→session_0),
    translation 은 mm. 여기선 미터로 변환해 반환.
    """
    out: dict[int, np.ndarray] = {}
    meta_p = sproj.parent / "meta.npz"
    if not meta_p.exists():
        return out
    try:
        m = np.load(meta_p, allow_pickle=False)
        idxs = m["scan_indices"]; T = m["T_pres"]
        for k in range(len(idxs)):
            Tk = np.asarray(T[k], float).copy()
            Tk[:3, 3] = Tk[:3, 3] / 1000.0     # mm → m
            out[int(idxs[k])] = Tk
    except Exception:
        pass
    return out


if __name__ == "__main__":
    # 캐시 생성 + 요약
    sess = load_sessions(use_cache=False)
    exp = expected_rotations()
    print(f"\n[common] sessions={len(sess)}  expected_rot={exp}")
    for i, p in enumerate(sess):
        ext = (np.asarray(p.points).max(0) - np.asarray(p.points).min(0)) * 1000
        print(f"  session[{i}] pts={len(p.points):,} color={p.has_colors()} "
              f"bbox_mm={np.round(ext,1)}")
