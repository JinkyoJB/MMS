"""
방법 3 — Artec SDK GlobalRegistration (real 트랙 전용).

SDK GR 은 IModel/IScan 위에서만 동작하고, 바인딩이 numpy→IFrameMesh 생성을
지원하지 않아 **합성 점군엔 적용 불가**. 여기선 원본 sproj(3 scan)을 로드해
SerialReg + GlobalReg 를 돌리고, 각 scan 의 GR 전/후 점군으로 rigid 변환 D_i 를
복원(Umeyama)한다. session_i→session_0 변환 = inv(D_0) @ D_i.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np


def _umeyama_rigid(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """src→dst 최적 rigid(R,t) (scale 없음). 같은 index 대응 가정."""
    sc = src.mean(0); dc = dst.mean(0)
    S = src - sc; D = dst - dc
    H = S.T @ D
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = dc - R @ sc
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def _accumulate(scan, transforms, stride: int) -> np.ndarray:
    pts = []
    for j in range(0, scan.frame_count(), stride):
        v = np.asarray(scan.get_frame(j).vertices(), float)
        if v.size == 0:
            continue
        T = transforms[j]
        pts.append(v @ T[:3, :3].T + T[:3, 3])
    return np.vstack(pts) if pts else np.zeros((0, 3))


def artec_gr_transforms(sproj: Path, stride: int = 8) -> dict:
    """{i: T_{i->0} (미터)} + 'runtime'. real 트랙 method 3."""
    from mms_artec.sensor.artec_client import ArtecClient
    from mms_artec.sensor import artec_base
    from mms_artec.sensor.artec_algorithm import (
        GlobalRegistrationSettingsDTO, GlobalRegistrationType, ScannerType,
    )

    entries = ArtecClient.load_project(str(sproj))
    scans = [getattr(e, "scan", None) for e in entries]
    scans = [s for s in scans if s is not None and s.frame_count() > 0]
    n = len(scans)

    # GR 전 transforms + 점군 스냅샷
    old_T = [[np.asarray(s.get_frame_transformation(j), float)
              for j in range(s.frame_count())] for s in scans]
    old_pts = [_accumulate(scans[i], old_T[i], stride) for i in range(n)]

    # model 구성 후 SerialReg + GlobalReg
    model = artec_base.create_model()
    for s in scans:
        model.add_scan(s)
    t0 = time.perf_counter()
    try:
        model = ArtecClient.serial_registration(model)
    except Exception as e:
        print(f"  [artec_gr] SerialReg 건너뜀: {e}")
    gr = GlobalRegistrationSettingsDTO.default(ScannerType.UNKNOWN)
    gr.registration_type = int(GlobalRegistrationType.GEOMETRY)
    model = ArtecClient.global_registration(model, gr)
    runtime = time.perf_counter() - t0

    # GR 후 transforms + 점군 → rigid 복원
    D = []
    for i in range(model.scan_count()):
        s2 = model.get_scan(i)
        new_T = [np.asarray(s2.get_frame_transformation(j), float)
                 for j in range(s2.frame_count())]
        new_pts = _accumulate(s2, new_T, stride)
        m = min(len(old_pts[i]), len(new_pts))
        D.append(_umeyama_rigid(old_pts[i][:m], new_pts[:m]))

    out = {"runtime": runtime}
    D0_inv = np.linalg.inv(D[0])
    for i in range(1, n):
        T = D0_inv @ D[i]
        T[:3, 3] = T[:3, 3] / 1000.0     # mm → m
        out[i] = T
    return out
