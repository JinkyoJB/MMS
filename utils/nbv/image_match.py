"""image_match.py — 텍스처(프레임 컬러 이미지) 특징점으로 스캔 간 rigid 정합. real 전용.

왜 이게 flip 정합의 1순위인가
-----------------------------
회전대칭에 가까운 물체(병·캔)는 기하만으로는 앞뒤·뒤집힘이 구분되지 않아 FGR/ICP 가
높은 fitness 로 **틀린 답**을 낸다(face-merging). 텍스처(라벨)는 그 모호성을 깬다 —
`scripts/artec/reg_benchmark`(2026-06-11): RootSIFT + 턴테이블 평면 키포인트 제거로
inlier 5→549, 수평축 flip(=cap 정합)을 유일하게 찾은 방법. 벤치마크 코드를 **라이브
IModel 에 그대로 쓸 수 있게** 옮긴 것이다(sproj 가 아니라 scan 핸들을 받는다).

무엇을 쓰나 — **후처리 Texturize 가 아니다.** 스캔 중 `capture_texture=ALWAYS` 로 프레임마다
저장되는 원본 사진(`frame.image()`)과 `frame.uv()`(정점↔픽셀)만 쓴다. 텍스처링 전에
바로 돌아간다.

흐름
    각 스캔: 프레임(stride) 마다 SIFT → uv 로 3D(세션 좌표) 부여 → (descriptor, xyz) DB
    → 턴테이블 평면 위 키포인트 제거 → sub↔master ratio+mutual 매칭 → 3D-3D RANSAC
    + Umeyama → T(sub 세션 → master 세션, m). inlier 수가 게이트.
"""
from __future__ import annotations

import time

import numpy as np

#: 채택 최소 inlier 수. 벤치마크: 정답 쌍 수백, 오답/무텍스처 쌍 한 자리.
MIN_INLIERS = 40
RANSAC_THRESH_M = 0.006


def umeyama_rigid(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """src→dst 최적 rigid(R,t), 같은 index 대응."""
    sc = src.mean(0); dc = dst.mean(0)
    S = src - sc; D = dst - dc
    U, _, Vt = np.linalg.svd(S.T @ D)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = dc - R @ sc
    return T


def _root_sift(d: np.ndarray) -> np.ndarray:
    d = np.asarray(d, np.float32)
    d /= (np.abs(d).sum(1, keepdims=True) + 1e-7)
    return np.sqrt(d)


def build_feature_db(scan, stride: int = 4, max_kp: int = 400):
    """scan(IScan 핸들) → (desc (M,128) f32, pts3d (M,3) m — 세션 좌표, frame_transformation 적용)."""
    import cv2
    from scipy.spatial import cKDTree
    sift = cv2.SIFT_create(nfeatures=max_kp)
    descs, pts = [], []
    for j in range(0, scan.frame_count(), max(1, stride)):
        f = scan.get_frame(j)
        try:
            if not f.has_image():
                continue
            img = f.image(); uv = f.uv()
        except Exception:                                   # noqa: BLE001
            continue
        v = np.asarray(f.vertices(), float)
        if uv is None or img is None or len(uv) != len(v) or len(v) == 0:
            continue
        H, W = img.shape[:2]
        gray = cv2.cvtColor(np.ascontiguousarray(img[..., :3]), cv2.COLOR_RGB2GRAY)
        kps, ds = sift.detectAndCompute(gray, None)
        if ds is None or len(kps) == 0:
            continue
        uv_px = np.column_stack([uv[:, 0] * (W - 1), uv[:, 1] * (H - 1)])
        tree = cKDTree(uv_px)
        T = np.asarray(scan.get_frame_transformation(j), float)
        v_sess = (v @ T[:3, :3].T + T[:3, 3]) / 1000.0     # mm → m
        kp_xy = np.array([kp.pt for kp in kps])
        dist, idx = tree.query(kp_xy, k=1)
        ok = dist < 3.0                                     # 3px 이내만
        if ok.sum() == 0:
            continue
        descs.append(_root_sift(ds[ok]))
        pts.append(v_sess[idx[ok]])
    if not descs:
        return np.zeros((0, 128), np.float32), np.zeros((0, 3))
    return np.vstack(descs).astype(np.float32), np.vstack(pts)


def drop_plane_feats(db, dist: float = 0.003, min_frac: float = 0.10):
    """턴테이블 평면 위 키포인트 제거 — 원판 무늬가 물체 텍스처보다 강해 매칭을 지배한다."""
    import open3d as o3d
    desc, pts = db
    if len(pts) < 200:
        return db
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    plane, inl = pc.segment_plane(dist, 3, 1000)
    if len(inl) < min_frac * len(pts):
        return db
    n = np.asarray(plane[:3], float)
    signed = (pts @ n + plane[3]) / (np.linalg.norm(n) + 1e-12)
    mask_in = np.zeros(len(pts), bool); mask_in[inl] = True
    side = np.sign(np.mean(signed[~mask_in])) or 1.0
    keep = (signed * side) > 0.003
    return desc[keep], pts[keep]


def dedupe_db(db, voxel_m: float = 0.002):
    """같은 3D 위치(복셀)의 키포인트는 하나만 남긴다.

    연속 프레임은 같은 물리 특징점을 거의 같은 디스크립터로 여러 번 넣는다. 그러면
    ratio test 의 '차근접' 이 진짜 대응의 복제본이 되어 비율이 1 에 가까워지고 진짜
    매칭이 탈락한다(합성 검증에서 동일 프레임 4장 → 매칭 0). 복셀당 첫 것을 남긴다.
    """
    desc, pts = db
    if len(pts) == 0:
        return db
    key = np.floor(np.asarray(pts, float) / voxel_m).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    first.sort()
    return desc[first], pts[first]


def match_desc(dA, dB):
    """A→B 매칭 (ratio 0.8 + mutual). 반환 (idxA, idxB)."""
    import cv2
    if len(dA) == 0 or len(dB) == 0:
        return np.zeros(0, int), np.zeros(0, int)
    bf = cv2.BFMatcher(cv2.NORM_L2)

    def ratio(d1, d2):
        good = {}
        for pair in bf.knnMatch(d1, d2, k=2):
            if len(pair) == 2 and pair[0].distance < 0.8 * pair[1].distance:
                good[pair[0].queryIdx] = pair[0].trainIdx
        return good
    gAB, gBA = ratio(dA, dB), ratio(dB, dA)
    ia = [q for q, t in gAB.items() if gBA.get(t) == q]
    return np.array(ia, int), np.array([gAB[q] for q in ia], int)


def ransac_rigid(src, dst, thresh: float = RANSAC_THRESH_M, iters: int = 2000, seed: int = 0):
    """3D-3D rigid RANSAC. 반환 (T src→dst, inlier_mask)."""
    rng = np.random.default_rng(seed)
    n = len(src)
    if n < 3:
        return np.eye(4), np.zeros(n, bool)
    best_T, best_in = np.eye(4), np.zeros(n, bool)
    for _ in range(iters):
        s = rng.choice(n, 3, replace=False)
        T = umeyama_rigid(src[s], dst[s])
        d = np.linalg.norm((src @ T[:3, :3].T + T[:3, 3]) - dst, axis=1)
        inl = d < thresh
        if inl.sum() > best_in.sum():
            best_in, best_T = inl, T
    if best_in.sum() >= 3:
        best_T = umeyama_rigid(src[best_in], dst[best_in])
    return best_T, best_in


def register_models(sub_model, master_model, *, stride: int = 4,
                    min_inliers: int = MIN_INLIERS, log=None):
    """sub IModel → master IModel 세션 좌표 변환 (T m, n_inliers, info). 실패면 (None, n, info).

    master 의 IScan 들은 이미 master 세션 좌표(frame_transformation 포함)이므로 전부
    한 DB 로 합친다. sub 도 마찬가지(자기 세션 좌표).
    """
    t0 = time.perf_counter()

    def _db(model):
        ds, ps = [], []
        for i in range(model.scan_count()):
            d, p = build_feature_db(model.get_scan(i), stride)
            if len(d):
                ds.append(d); ps.append(p)
        if not ds:
            return np.zeros((0, 128), np.float32), np.zeros((0, 3))
        return np.vstack(ds), np.vstack(ps)

    dS, pS = dedupe_db(drop_plane_feats(_db(sub_model)))
    dM, pM = dedupe_db(drop_plane_feats(_db(master_model)))
    info = {"feat_sub": int(len(dS)), "feat_master": int(len(dM))}
    if log:
        log(f"[img] 특징점(중복 제거 후) sub={len(dS)} master={len(dM)}")
    ia, ib = match_desc(dS, dM)
    info["matches"] = int(len(ia))
    if len(ia) < 3:
        info["reason"] = "매칭 부족"
        info["runtime"] = time.perf_counter() - t0
        return None, 0, info
    T, inl = ransac_rigid(pS[ia], pM[ib])
    n = int(inl.sum())
    info["inliers"] = n
    info["runtime"] = time.perf_counter() - t0
    if log:
        log(f"[img] 매칭 {len(ia)} → inlier {n} ({info['runtime']:.1f}s)")
    if n < min_inliers:
        info["reason"] = f"inlier {n} < {min_inliers}"
        return None, n, info
    return T, n, info
