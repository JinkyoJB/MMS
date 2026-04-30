# mms/nbv/frontier.py
#
# Frontier 후보 생성 (docs/2_control_layers.md §4).
#
# 누적 mesh M 의 boundary edge 를 추출하고, 연결 컴포넌트(segment)로 묶은 뒤
# 필터/재분할하여 NBV 후보 리스트 `[FrontierCandidate]` 를 반환한다.

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from typing import List

import numpy as np
import open3d as o3d


@dataclass
class FrontierCandidate:
    """
    한 개의 frontier 후보 (segment 또는 그 sub-segment).

    p_O  : (3,) — 대표점 (길이 가중 centroid), O 프레임, meter
    n_O  : (3,) — 대표 normal (세그먼트 vertex 의 mesh vertex normal 평균, 바깥 방향)
    L    : float — segment 길이 (meter)
    vids : np.ndarray (K,) int — 원본 mesh vertex 인덱스 (디버깅/시각화용)
    """

    p_O: np.ndarray
    n_O: np.ndarray
    L: float
    vids: np.ndarray


# ─────────────────────────────────────────────────────────────────────────────
# Boundary edge 추출
# ─────────────────────────────────────────────────────────────────────────────

def extract_boundary_edges(mesh: o3d.geometry.TriangleMesh) -> np.ndarray:
    """
    "삼각형 하나에만 속한 edge" 를 반환.

    Returns
    -------
    boundary_edges : (E, 2) int
        각 행 = (v0, v1) vertex 인덱스. 항상 v0 < v1 로 정규화.
    """
    tris = np.asarray(mesh.triangles, dtype=np.int64)
    if len(tris) == 0:
        return np.empty((0, 2), dtype=np.int64)

    edges = np.vstack([
        tris[:, [0, 1]],
        tris[:, [1, 2]],
        tris[:, [2, 0]],
    ])
    edges = np.sort(edges, axis=1)                               # (3T, 2)
    pairs, counts = np.unique(edges, axis=0, return_counts=True)
    boundary = pairs[counts == 1]
    return boundary.astype(np.int64)


# ─────────────────────────────────────────────────────────────────────────────
# Connected component (BFS)
# ─────────────────────────────────────────────────────────────────────────────

def boundary_segments(boundary_edges: np.ndarray) -> List[np.ndarray]:
    """
    boundary_edges 를 무방향 그래프로 보고 connected component 탐색.

    Returns
    -------
    segments : list of (K,) int  — 각 컴포넌트의 vertex 인덱스 집합
    """
    if len(boundary_edges) == 0:
        return []

    adj: dict[int, set[int]] = defaultdict(set)
    for u, v in boundary_edges:
        adj[int(u)].add(int(v))
        adj[int(v)].add(int(u))

    visited: set[int] = set()
    segments: List[np.ndarray] = []
    for start in list(adj.keys()):
        if start in visited:
            continue
        # BFS
        comp: List[int] = []
        q = deque([start])
        visited.add(start)
        while q:
            u = q.popleft()
            comp.append(u)
            for nb in adj[u]:
                if nb not in visited:
                    visited.add(nb)
                    q.append(nb)
        segments.append(np.asarray(comp, dtype=np.int64))

    return segments


# ─────────────────────────────────────────────────────────────────────────────
# Segment 길이 계산 (boundary_edges 중 해당 vertex 들로 이루어진 edge 합)
# ─────────────────────────────────────────────────────────────────────────────

def _segment_edge_list(
    seg_vids: np.ndarray,
    boundary_edges: np.ndarray,
) -> np.ndarray:
    """해당 segment 에 포함되는 boundary edge 부분집합."""
    vset = set(int(x) for x in seg_vids)
    mask = np.array(
        [(int(u) in vset) and (int(v) in vset) for u, v in boundary_edges],
        dtype=bool,
    )
    return boundary_edges[mask]


def _segment_length(
    seg_edges: np.ndarray,
    vertices: np.ndarray,
) -> float:
    if len(seg_edges) == 0:
        return 0.0
    d = vertices[seg_edges[:, 0]] - vertices[seg_edges[:, 1]]
    return float(np.linalg.norm(d, axis=1).sum())


# ─────────────────────────────────────────────────────────────────────────────
# Segment → 후보 (길이 가중 centroid + 평균 normal)
# ─────────────────────────────────────────────────────────────────────────────

def _segment_to_candidate(
    seg_vids: np.ndarray,
    seg_edges: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_normals: np.ndarray,
    outward_ref: np.ndarray,     # (3,) — 일반적으로 mesh centroid, 이 점에서 멀어지는 방향이 outward
) -> FrontierCandidate:
    # edge 길이 가중 centroid: 각 vertex 의 "인접 edge 절반 길이 합" 을 weight 로
    weight = np.zeros(len(mesh_vertices), dtype=np.float64)
    if len(seg_edges) > 0:
        d = np.linalg.norm(
            mesh_vertices[seg_edges[:, 0]] - mesh_vertices[seg_edges[:, 1]],
            axis=1,
        )
        np.add.at(weight, seg_edges[:, 0], 0.5 * d)
        np.add.at(weight, seg_edges[:, 1], 0.5 * d)

    w = weight[seg_vids]
    if w.sum() < 1e-9:
        # degenerate — simple mean
        p = mesh_vertices[seg_vids].mean(axis=0)
    else:
        p = (mesh_vertices[seg_vids] * w[:, None]).sum(axis=0) / w.sum()

    n = mesh_normals[seg_vids].mean(axis=0)
    nn = np.linalg.norm(n)
    if nn < 1e-9:
        # fallback: radial (p - ref) 방향
        n = p - outward_ref
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            n = np.array([0.0, 0.0, 1.0])
            nn = 1.0
    n = n / nn

    # outward 강제
    if (p - outward_ref) @ n < 0:
        n = -n

    L = _segment_length(seg_edges, mesh_vertices)

    return FrontierCandidate(
        p_O=p.astype(np.float64),
        n_O=n.astype(np.float64),
        L=float(L),
        vids=seg_vids.astype(np.int64),
    )


def _split_long_segment(
    seg_vids: np.ndarray,
    seg_edges: np.ndarray,
    mesh_vertices: np.ndarray,
    mesh_normals: np.ndarray,
    outward_ref: np.ndarray,
    max_seg_length: float,
) -> List[FrontierCandidate]:
    """
    세그먼트가 너무 길면 길이 기준 ceil(L/max) 등분.

    간단 구현 — vertex 를 x, y 기준으로 정렬한 뒤 chunks 로 나눔.
    (정확한 "arc length 등분" 은 추후 개선.)
    """
    L = _segment_length(seg_edges, mesh_vertices)
    if L <= max_seg_length:
        return [_segment_to_candidate(
            seg_vids, seg_edges, mesh_vertices, mesh_normals, outward_ref,
        )]

    n_parts = int(np.ceil(L / max_seg_length))
    if n_parts < 2:
        return [_segment_to_candidate(
            seg_vids, seg_edges, mesh_vertices, mesh_normals, outward_ref,
        )]

    pts = mesh_vertices[seg_vids]
    # centroid 기준 atan2 로 각도 정렬 (ring 에 유리), chain 도 일관된 순서
    c = pts.mean(axis=0)
    rel = pts - c
    theta = np.arctan2(rel[:, 1], rel[:, 0])
    order = np.argsort(theta)
    seg_sorted = seg_vids[order]

    chunks = np.array_split(seg_sorted, n_parts)
    cands: List[FrontierCandidate] = []
    for chunk in chunks:
        if len(chunk) < 2:
            continue
        sub_edges = _segment_edge_list(chunk, seg_edges)
        cands.append(_segment_to_candidate(
            chunk, sub_edges, mesh_vertices, mesh_normals, outward_ref,
        ))
    return cands


# ─────────────────────────────────────────────────────────────────────────────
# 공개 API
# ─────────────────────────────────────────────────────────────────────────────

def extract_frontier_candidates(
    mesh: o3d.geometry.TriangleMesh,
    min_seg_vertices: int = 10,
    min_seg_length: float = 0.01,
    max_seg_length: float = 0.08,
) -> List[FrontierCandidate]:
    """
    누적 mesh 에서 frontier 후보 리스트를 생성한다 (§4).

    단계
    ----
    1) boundary edge 추출
    2) connected component (segment) 탐색
    3) 필터 (min_seg_vertices, min_seg_length) 및 재분할 (max_seg_length)
    4) 각 segment → (p, n, L) 후보

    Parameters
    ----------
    mesh : open3d.geometry.TriangleMesh  (O 프레임, crop 적용된 상태 권장)
    min_seg_vertices : int
    min_seg_length   : float (m)
    max_seg_length   : float (m)

    Returns
    -------
    list[FrontierCandidate]
    """
    if len(mesh.triangles) == 0:
        return []
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

    vertices = np.asarray(mesh.vertices)
    normals = np.asarray(mesh.vertex_normals)

    if len(vertices) == 0:
        return []

    outward_ref = vertices.mean(axis=0)      # mesh 대략 중심

    boundary = extract_boundary_edges(mesh)
    segments = boundary_segments(boundary)

    candidates: List[FrontierCandidate] = []
    for seg in segments:
        if len(seg) < min_seg_vertices:
            continue
        seg_edges = _segment_edge_list(seg, boundary)
        L = _segment_length(seg_edges, vertices)
        if L < min_seg_length:
            continue
        candidates.extend(_split_long_segment(
            seg, seg_edges, vertices, normals, outward_ref, max_seg_length,
        ))
    return candidates
