"""mesh_sampling.py — 메시 **표면**을 균일 밀도로 점 샘플링.

왜 정점(vertex)만으로는 안 되나
------------------------------
점군 기반 SDF/KD-tree 는 **점이 있는 곳만 장애물**로 본다. 그런데 CAD 부품, 특히
압출·판재는 **정점이 모서리에만** 있다:

    4040 알루미늄 프로파일 1m → 정점 1,694개가 전부 **양 끝 단면**에 몰려 있고
    길이 방향 1m 구간에는 정점이 하나도 없다 (2026-08-18 실측)

그래서 정점만 담으면 기둥 중간이 **텅 빈 복도**가 되고, 로봇이 그대로 통과한다.
실제로 스캐너가 `/World/frame/HFS8_4040_1000_04` 를 관통했다.

로봇 링크는 유기적 곡면이라 테셀레이션이 촘촘해 덜 드러나지만 원리는 같다.

방법
----
삼각형 **면적에 비례**해 샘플 수를 배분하고 무게중심 좌표로 균일 추출한다.
정점도 함께 넣어 모서리·꼭짓점을 보장한다. 결정적(rng 시드 고정).
"""
from __future__ import annotations

import numpy as np


def triangulate(face_counts, face_indices):
    """n-gon 을 부채꼴(fan)로 삼각분할 → (T,3) 인덱스. **완전 벡터화**.

    ⚠ 파이썬 루프로 만들면 정점 100만급 메시(Spider)에서 튜플 수백만 개가 쌓여
      **OOM 으로 죽는다**(실측: EXIT=137). numpy 만으로 구성한다.
    """
    counts = np.asarray(face_counts, np.int64)
    idx = np.asarray(face_indices, np.int64)
    if counts.size == 0 or idx.size == 0:
        return np.zeros((0, 3), np.int64)
    offs = np.concatenate(([0], np.cumsum(counts)[:-1]))
    ntri = np.maximum(counts - 2, 0)
    ok = ntri > 0
    if not ok.any():
        return np.zeros((0, 3), np.int64)
    fi = np.repeat(np.nonzero(ok)[0], ntri[ok])          # 삼각형별 원본 face 번호
    starts = np.cumsum(ntri[ok]) - ntri[ok]              # face 별 첫 삼각형 위치
    j = np.arange(len(fi)) - np.repeat(starts, ntri[ok])  # face 내 삼각형 인덱스
    o = offs[fi]
    return np.stack([idx[o], idx[o + 1 + j], idx[o + 2 + j]], axis=1)


def sample_surface(points, face_counts, face_indices, spacing_m=0.005,
                   rng=None, max_pts=400_000, include_vertices=True,
                   per_tri_cap=32768):
    """메시 표면을 `spacing_m` 간격 목표로 샘플링해 (N,3) float32 반환.

    spacing 은 SDF 복셀보다 **촘촘해야** 한다 — 점 사이가 복셀보다 벌어지면
    그 틈이 빈 공간으로 인식된다(원래 문제와 같은 실패).

    ⚠ **메모리 상한이 강제된다.** 이 함수는 예전에 상한이 없어 대형 메시에서
      OOM 을 냈고 실제로 작업 PC 를 멈췄다. 세 겹으로 막는다:
        1. `per_tri_cap` — 삼각형 하나가 만들 수 있는 점 수 상한
        2. `max_pts`     — 이 메시 전체 점 수 상한(넘으면 조기 종료 후 균일 솎기)
        3. float32       — 메모리 절반
      상한에 걸리면 조용히 자르지 않고 호출자가 알 수 있도록 그대로 솎아 반환한다.
    """
    V = np.asarray(points, float)
    if len(V) == 0:
        return np.zeros((0, 3), np.float32)
    tris = triangulate(face_counts, face_indices)
    out = [V.astype(np.float32)] if include_vertices else []
    total = len(out[0]) if out else 0
    if len(tris) and total < max_pts:
        rng = rng or np.random.default_rng(0)
        CH = 100_000                       # 삼각형 청크 — 임시배열 크기 고정
        for s0 in range(0, len(tris), CH):
            if total >= max_pts:
                break
            t = tris[s0:s0 + CH]
            A, B, C = V[t[:, 0]], V[t[:, 1]], V[t[:, 2]]
            area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
            n = np.floor(area / (spacing_m ** 2)).astype(np.int64)
            np.minimum(n, per_tri_cap, out=n)          # ① 삼각형당 상한
            budget = max_pts - total
            if n.sum() > budget:                        # ② 전체 상한
                keep_frac = budget / float(n.sum())
                n = np.floor(n * keep_frac).astype(np.int64)
            sel = n > 0
            if not sel.any():
                continue
            rep = np.repeat(np.nonzero(sel)[0], n[sel])
            u = np.sqrt(rng.random(len(rep)))
            v = rng.random(len(rep))
            P = ((1 - u)[:, None] * A[rep]
                 + (u * (1 - v))[:, None] * B[rep]
                 + (u * v)[:, None] * C[rep])
            out.append(P.astype(np.float32))            # ③ float32
            total += len(P)
    S = np.vstack(out) if out else np.zeros((0, 3), np.float32)
    if len(S) > max_pts:
        S = S[:: int(np.ceil(len(S) / max_pts))]
    return S
