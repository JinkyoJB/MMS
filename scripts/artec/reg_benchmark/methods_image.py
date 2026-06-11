"""
방법 6 — 텍스처(이미지) 특징점 매칭 정합 (real 트랙 전용).

합성 점군엔 이미지가 없어 real 트랙에서만. 각 세션의 프레임들에서 SIFT
키포인트를 뽑고 uv→vertex 로 3D(세션 좌표)를 부여 → "디스크립터↔3D" DB.
세션 간 디스크립터 매칭(ratio+mutual) → 3D-3D 대응 → RANSAC+Umeyama 로
rigid 변환 추정. 회전대칭 객체의 앞-뒤 ambiguity 를 텍스처로 깨는 게 목적.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from scripts.artec.reg_benchmark.methods_artec import _umeyama_rigid


def _build_feature_db(scan, stride: int, max_kp: int = 400):
    """scan → (descriptors (M,128) float32, pts3d (M,3) m, 세션좌표)."""
    import cv2
    from scipy.spatial import cKDTree
    sift = cv2.SIFT_create(nfeatures=max_kp)
    descs, pts = [], []
    for j in range(0, scan.frame_count(), stride):
        f = scan.get_frame(j)
        if not f.has_image():
            continue
        img = f.image()                       # (H,W,3) RGB uint8
        uv = f.uv()
        v = np.asarray(f.vertices(), float)
        if uv is None or img is None or len(uv) != len(v) or len(v) == 0:
            continue
        H, W = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        kps, ds = sift.detectAndCompute(gray, None)
        if ds is None or len(kps) == 0:
            continue
        # uv(px) → vertex KDtree
        uv_px = np.column_stack([uv[:, 0] * (W - 1), uv[:, 1] * (H - 1)])
        tree = cKDTree(uv_px)
        T = np.asarray(scan.get_frame_transformation(j), float)
        v_sess = (v @ T[:3, :3].T + T[:3, 3]) / 1000.0   # mm→m, 세션좌표
        kp_xy = np.array([kp.pt for kp in kps])
        dist, idx = tree.query(kp_xy, k=1)
        ok = dist < 3.0                       # 3px 이내만 신뢰
        if ok.sum() == 0:
            continue
        descs.append(ds[ok])
        pts.append(v_sess[idx[ok]])
    if not descs:
        return np.zeros((0, 128), np.float32), np.zeros((0, 3))
    return np.vstack(descs).astype(np.float32), np.vstack(pts)


def _match(dA, dB):
    """A→B 디스크립터 매칭 (ratio 0.8 + mutual). 반환 (idxA, idxB)."""
    import cv2
    if len(dA) == 0 or len(dB) == 0:
        return np.zeros(0, int), np.zeros(0, int)
    bf = cv2.BFMatcher(cv2.NORM_L2)
    def ratio(d1, d2):
        m = bf.knnMatch(d1, d2, k=2)
        good = {}
        for pair in m:
            if len(pair) < 2:
                continue
            a, b = pair
            if a.distance < 0.8 * b.distance:
                good[a.queryIdx] = a.trainIdx
        return good
    gAB = ratio(dA, dB)
    gBA = ratio(dB, dA)
    ia, ib = [], []
    for qi, ti in gAB.items():
        if gBA.get(ti) == qi:                 # mutual
            ia.append(qi); ib.append(ti)
    return np.array(ia, int), np.array(ib, int)


def _ransac_rigid(src, dst, thresh=0.006, iters=2000, seed=0):
    """3D-3D rigid RANSAC. 반환 (T, inlier_mask)."""
    rng = np.random.default_rng(seed)
    n = len(src)
    if n < 3:
        return np.eye(4), np.zeros(n, bool)
    best_T = np.eye(4); best_in = np.zeros(n, bool)
    for _ in range(iters):
        s = rng.choice(n, 3, replace=False)
        T = _umeyama_rigid(src[s], dst[s])
        res = (src @ T[:3, :3].T + T[:3, 3]) - dst
        d = np.linalg.norm(res, axis=1)
        inl = d < thresh
        if inl.sum() > best_in.sum():
            best_in = inl; best_T = T
    if best_in.sum() >= 3:
        best_T = _umeyama_rigid(src[best_in], dst[best_in])
    return best_T, best_in


def _drop_plane_feats(db, dist=0.003, min_frac=0.10):
    """디스크립터 DB(desc, pts3d) 에서 턴테이블 평면 위 키포인트 제거."""
    import open3d as o3d
    desc, pts = db
    if len(pts) < 200:
        return db
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    plane, inl = pc.segment_plane(dist, 3, 1000)
    if len(inl) < min_frac * len(pts):
        return db
    a, b, c, d = plane
    n = np.array([a, b, c], float)
    signed = (pts @ n + d) / (np.linalg.norm(n) + 1e-12)
    mask_in = np.zeros(len(pts), bool); mask_in[inl] = True
    side = np.sign(np.mean(signed[~mask_in])) or 1.0
    keep = (signed * side) > 0.003
    return desc[keep], pts[keep]


def image_match_transforms(sproj: Path, stride: int = 4) -> dict:
    """{i: (T_{i->0}, n_inliers)} + 'runtime' + 'feat_counts'."""
    from mms_artec.sensor.artec_client import ArtecClient
    entries = ArtecClient.load_project(str(sproj))
    scans = [getattr(e, "scan", None) for e in entries]
    scans = [s for s in scans if s is not None and s.frame_count() > 0]
    t0 = time.perf_counter()
    dbs = [_build_feature_db(s, stride) for s in scans]
    dbs = [_drop_plane_feats(d) for d in dbs]      # 턴테이블 평면 키포인트 제거
    out = {"feat_counts": [len(d[0]) for d in dbs]}
    dA0, p0 = dbs[0]
    for i in range(1, len(scans)):
        dAi, pi = dbs[i]
        ia, ib = _match(dAi, dA0)
        if len(ia) < 3:
            out[i] = (np.eye(4), 0)
            continue
        T, inl = _ransac_rigid(pi[ia], p0[ib])
        out[i] = (T, int(inl.sum()))
    out["runtime"] = time.perf_counter() - t0
    return out
