#!/usr/bin/env python
# scripts/inspect_dataset_frame.py
#
# Dataset 내 한 프레임의 raw .npy 를 Open3D 로 시각화.
# docs/phoxi_raw_dataset.md 의 "raw data 가 실제로 뭘 담고 있는가" 를
# 파이프라인 연산과 분리해 눈으로 검증하기 위함.
#
# 기본 동작
# --------
# 1. `range.npy` (H,W,3) float32 mm → C 프레임 PointCloud
# 2. `intensity.npy` → grayscale vertex color (선택 auto-stretch)
# 3. Open3D 창 표시 (축 포함)
# 4. 통계 출력 (shape, XYZ 범위, valid pixel 수 등)
#
# 옵션
# ----
# - `--frame` : dataset 상대 index (0..N-1)
# - `--frame-dir` : 직접 frame_XXX 폴더 지정
# - `--in-O` : calibration + meta.json 으로 O 프레임 변환해 같이 표시
# - `--show-normals` : normals.npy 를 법선 벡터 화살표로 표시 (서브샘플)
# - `--save-preview` : PNG 로 pcd 렌더 저장
# - `--stats-only` : GUI 없이 통계만
#
# 사용
# ----
#   python scripts/inspect_dataset_frame.py \
#       --dataset datasets/phoxi_20260424_125415 --frame 0
#   python scripts/inspect_dataset_frame.py \
#       --dataset datasets/phoxi_20260424_125415 --frame 0 --in-O
#   python scripts/inspect_dataset_frame.py \
#       --frame-dir datasets/phoxi_20260424_125415/frames/frame_005 --stats-only

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import open3d as o3d

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.transforms import TurntableTransformConfig, load_transform


# ─────────────────────────────────────────────────────────────────────────────
# 경로 해석
# ─────────────────────────────────────────────────────────────────────────────

def _resolve_path(p: Path) -> Path:
    return p.resolve() if p.is_absolute() else (_PROJECT_ROOT / p).resolve()


def _resolve_frame_dir(
    dataset: Optional[str], frame: Optional[int], frame_dir: Optional[str]
) -> Tuple[Path, Path]:
    """
    Returns
    -------
    (dataset_root, frame_dir) — 둘 다 절대경로.
    frame_dir 는 frame_XXX 폴더.
    """
    if frame_dir is not None:
        fdir = _resolve_path(Path(frame_dir))
        if not fdir.exists():
            sys.exit(f"frame_dir 없음: {fdir}")
        # dataset root = 부모의 부모 (frames/frame_XXX → frames → dataset)
        dataset_root = fdir.parent.parent
        return dataset_root, fdir

    if dataset is None or frame is None:
        sys.exit("--dataset + --frame 또는 --frame-dir 지정 필요")

    ds_root = _resolve_path(Path(dataset))
    if not ds_root.exists():
        sys.exit(f"dataset 폴더 없음: {ds_root}")
    fdir = ds_root / "frames" / f"frame_{int(frame):03d}"
    if not fdir.exists():
        sys.exit(f"frame 폴더 없음: {fdir}")
    return ds_root, fdir


# ─────────────────────────────────────────────────────────────────────────────
# 데이터 로드
# ─────────────────────────────────────────────────────────────────────────────

def _load_frame(frame_dir: Path) -> dict:
    data = {}
    data["range"]     = np.load(frame_dir / "range.npy")        # (H, W, 3) f32 mm
    it = frame_dir / "intensity.npy"
    data["intensity"] = np.load(it) if it.exists() else None     # (H,W) 또는 (H,W,3) uint8
    nm = frame_dir / "normals.npy"
    data["normals"]   = np.load(nm) if nm.exists() else None     # (H, W, 3) f32
    ci = frame_dir / "color_image.npy"
    data["color_image"] = np.load(ci) if ci.exists() else None   # (H_c, W_c, 3) uint8 RGB
    mt = frame_dir / "meta.json"
    data["meta"]      = json.loads(mt.read_text(encoding="utf-8")) if mt.exists() else {}
    return data


# ─────────────────────────────────────────────────────────────────────────────
# 통계
# ─────────────────────────────────────────────────────────────────────────────

def _print_stats(d: dict, frame_dir: Path) -> None:
    rng = d["range"]
    intensity = d["intensity"]
    normals = d["normals"]
    meta = d["meta"]

    print(f"\n══════════ {frame_dir} ══════════")

    H, W, _ = rng.shape
    flat = rng.reshape(-1, 3)
    valid = ~np.all(flat == 0.0, axis=1) & np.isfinite(flat).all(axis=1)
    v = flat[valid]
    print(f"  range       : shape={rng.shape}  dtype={rng.dtype}  "
          f"size={rng.nbytes/1024/1024:.1f} MB")
    print(f"  valid pixels: {int(valid.sum()):,} / {H*W:,}  "
          f"({100*valid.mean():.1f}%)")
    if len(v) > 0:
        print(f"  X (mm, C)   : [{v[:,0].min():+8.1f}, {v[:,0].max():+8.1f}]  "
              f"mean={v[:,0].mean():+7.1f}")
        print(f"  Y (mm, C)   : [{v[:,1].min():+8.1f}, {v[:,1].max():+8.1f}]  "
              f"mean={v[:,1].mean():+7.1f}")
        print(f"  Z (mm, C)   : [{v[:,2].min():+8.1f}, {v[:,2].max():+8.1f}]  "
              f"mean={v[:,2].mean():+7.1f}")

    if intensity is not None:
        kind = "RGB per-point" if (intensity.ndim == 3 and intensity.shape[2] == 3) \
               else "grayscale"
        print(f"  intensity   : shape={intensity.shape}  dtype={intensity.dtype}  "
              f"kind={kind}  [{intensity.min()}, {intensity.max()}]  "
              f"mean={intensity.mean():.1f}")
    else:
        print("  intensity   : (없음)")

    color_image = d.get("color_image")
    if color_image is not None:
        print(f"  color_image : shape={color_image.shape}  dtype={color_image.dtype}  "
              f"[{color_image.min()}, {color_image.max()}]  "
              f"(ColorCamera 2D RGB — 별도 해상도)")

    if normals is not None:
        nf = normals.reshape(-1, 3)
        vn = nf[valid]
        nn = np.linalg.norm(vn, axis=1)
        print(f"  normals     : shape={normals.shape}  ‖n‖ ∈ [{nn.min():.3f}, {nn.max():.3f}]  "
              f"mean‖n‖={nn.mean():.3f}")
    else:
        print("  normals     : (없음)")

    if meta:
        print(f"  meta        : idx={meta.get('idx')}  "
              f"θ_target={meta.get('theta_target_deg'):.2f}°  "
              f"θ_actual={meta.get('theta_actual_deg'):.2f}°  "
              f"ts={meta.get('timestamp')}")
        if "T_EB" in meta:
            t = np.array(meta["T_EB"])[:3, 3] * 1000.0
            print(f"  T_EB trans  : [{t[0]:+7.1f}, {t[1]:+7.1f}, {t[2]:+7.1f}] mm (B frame)")


# ─────────────────────────────────────────────────────────────────────────────
# PCD 구성
# ─────────────────────────────────────────────────────────────────────────────

def _build_pcd_C_m(
    d: dict,
    min_depth_mm: float = 50.0,
    max_depth_mm: float = 2000.0,
    intensity_stretch: bool = True,
) -> o3d.geometry.PointCloud:
    """
    Range + Intensity → PointCloud (C 프레임, meters).
    auto-stretch 옵션으로 어두운 intensity 를 [0, 1] 로 확장.
    """
    rng = d["range"]
    H, W, _ = rng.shape
    flat_mm = rng.reshape(-1, 3).astype(np.float64)
    z = flat_mm[:, 2]
    valid = (
        (z > min_depth_mm) & (z < max_depth_mm) &
        np.isfinite(flat_mm).all(axis=1)
    )
    pts_C_m = flat_mm[valid] / 1000.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_C_m)

    intensity = d["intensity"]
    if intensity is not None:
        arr = np.asarray(intensity)
        # Case 1: RGB per-3D-point (TextureSource=Color) → 직접 사용
        if arr.ndim == 3 and arr.shape[2] == 3 and arr.shape[:2] == (H, W):
            rgb_flat = arr.reshape(-1, 3).astype(np.float32) / 255.0   # (H*W, 3)
            colors = rgb_flat[valid].astype(np.float64)
            pcd.colors = o3d.utility.Vector3dVector(colors)
        # Case 2: grayscale (H, W) → stack 후 선택적 auto-stretch
        elif arr.ndim == 2 and arr.shape == (H, W):
            flat_i = arr.astype(np.float32).reshape(-1)
            vi = flat_i[valid]
            if intensity_stretch and vi.size > 100:
                lo = float(np.percentile(vi, 2.0))
                hi = float(np.percentile(vi, 98.0))
                g = (np.clip((vi - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
                     if hi > lo + 1e-6 else np.clip(vi / 255.0, 0.0, 1.0))
            else:
                g = np.clip(vi / 255.0, 0.0, 1.0)
            colors = np.stack([g, g, g], axis=1).astype(np.float64)
            pcd.colors = o3d.utility.Vector3dVector(colors)
        # 그 외 shape mismatch 은 무시 (색 없이 pcd 반환)

    normals = d["normals"]
    if normals is not None:
        flat_n = normals.reshape(-1, 3).astype(np.float64)
        pcd.normals = o3d.utility.Vector3dVector(flat_n[valid])

    return pcd


# ─────────────────────────────────────────────────────────────────────────────
# O 프레임 변환
# ─────────────────────────────────────────────────────────────────────────────

def _transform_pcd_C_to_O(
    pcd_C: o3d.geometry.PointCloud,
    T_CO: np.ndarray,
) -> o3d.geometry.PointCloud:
    """pcd_C(C frame m) → pcd_O(O frame m). pcd_C 은 변경하지 않음."""
    pcd_O = o3d.geometry.PointCloud(pcd_C)
    pcd_O.transform(T_CO)
    return pcd_O


def _compute_T_CO_from_meta(
    meta: dict,
    dataset_root: Path,
) -> Optional[np.ndarray]:
    """
    meta.json + dataset/calibration/*.yaml 을 이용해 T_CO (C→O) 를 계산.
    """
    calib = dataset_root / "calibration"
    he = calib / "hand_eye_phoxi.yaml"
    tt = calib / "turntable_frame.yaml"
    if not (he.exists() and tt.exists()):
        print(f"  [!] calibration 파일 누락 ({calib}) — O 변환 불가")
        return None
    T_EC  = load_transform(str(he), "T_E_C")
    T_BF0 = load_transform(str(tt), "T_B_F0")
    tt_cfg = TurntableTransformConfig(T_BF0)

    theta = float(meta.get("theta_actual_rad", 0.0))
    T_EB = np.array(meta["T_EB"], dtype=float)
    # T_CO = T_FO · T_BF(θ) · T_EB · T_CE   (T_OF = I)
    T_FO = np.eye(4)                               # inv(T_OF) = I
    T_BF = np.linalg.inv(tt_cfg.T_FB(theta))
    T_CE = np.linalg.inv(T_EC)
    return T_FO @ T_BF @ T_EB @ T_CE


# ─────────────────────────────────────────────────────────────────────────────
# 시각화
# ─────────────────────────────────────────────────────────────────────────────

def _normal_arrows(
    pcd: o3d.geometry.PointCloud,
    every: int = 200,
    length: float = 0.005,
) -> o3d.geometry.LineSet:
    pts = np.asarray(pcd.points)
    if not pcd.has_normals() or len(pts) == 0:
        return o3d.geometry.LineSet()
    nrm = np.asarray(pcd.normals)
    idx = np.arange(0, len(pts), every)
    starts = pts[idx]
    ends = starts + nrm[idx] * length
    verts = np.vstack([starts, ends])
    lines = np.array([[i, i + len(idx)] for i in range(len(idx))])
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(verts)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(
        np.tile([0.2, 0.8, 0.2], (len(lines), 1))
    )
    return ls


def _show_texture_2d(d: dict, title: str) -> None:
    """
    intensity + color_image 를 matplotlib 으로 나란히 표시 (blocking).
    창 닫으면 다음 단계로 진행.
    """
    intensity = d.get("intensity")
    color_image = d.get("color_image")
    panels: list = []
    if intensity is not None:
        if intensity.ndim == 3 and intensity.shape[2] == 3:
            panels.append(("Intensity (RGB per-point)", intensity, None))
        else:
            panels.append(("Intensity (grayscale)", intensity, "gray"))
    if color_image is not None:
        panels.append(
            (f"ColorCamera 2D RGB {tuple(color_image.shape)}", color_image, None),
        )
    if not panels:
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[texture] matplotlib 없음 — `pip install matplotlib`")
        return

    fig, axes = plt.subplots(1, len(panels), figsize=(7 * len(panels), 7))
    if len(panels) == 1:
        axes = [axes]
    for ax, (label, img, cmap) in zip(axes, panels):
        if cmap:
            ax.imshow(img, cmap=cmap, vmin=0, vmax=255)
        else:
            ax.imshow(img)
        ax.set_title(f"{label}  shape={img.shape}")
        ax.axis("off")
    fig.suptitle(title, fontsize=12)
    fig.tight_layout()
    print("[texture] 2D 창 표시 중 — 닫으면 3D 뷰어로 진행")
    plt.show()


def _show(
    geoms: list,
    title: str,
    front: list = [-0.4, -0.4, -0.8],
    up: list = [0.0, 0.0, 1.0],
    lookat_default: list = [0.0, 0.0, 0.5],
) -> None:
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1280, height=800)
    for g in geoms:
        vis.add_geometry(g)
    try:
        ctl = vis.get_view_control()
        ctl.set_up(up)
        ctl.set_front(front)
        ctl.set_lookat(lookat_default)
        ctl.set_zoom(0.7)
    except Exception:
        pass
    vis.run()
    vis.destroy_window()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description="Dataset 한 프레임의 raw .npy 를 Open3D 로 시각화.",
    )
    p.add_argument("--dataset", default=None, help="datasets/<session> 경로")
    p.add_argument("--frame", type=int, default=None, help="frame index (0..N-1)")
    p.add_argument("--frame-dir", default=None, help="frame_XXX 폴더 직접 지정")
    p.add_argument("--in-O", action="store_true",
                   help="O 프레임으로도 변환해 같이 표시 (calibration + meta 필요)")
    p.add_argument("--show-normals", action="store_true",
                   help="노말 벡터 화살표 표시 (서브샘플)")
    p.add_argument("--normal-every", type=int, default=500)
    p.add_argument("--min-depth-mm", type=float, default=50.0)
    p.add_argument("--max-depth-mm", type=float, default=2000.0)
    p.add_argument("--no-intensity-stretch", action="store_true",
                   help="intensity auto-stretch 끔 (원본 밝기)")
    p.add_argument("--stats-only", action="store_true",
                   help="GUI 생략, 통계만")
    p.add_argument("--texture-2d", dest="texture_2d", action="store_true",
                   default=True,
                   help="Intensity / color_image 를 matplotlib 으로 같이 보기 (기본 on)")
    p.add_argument("--no-texture-2d", dest="texture_2d", action="store_false",
                   help="2D 텍스처 창 생략")
    args = p.parse_args()

    ds_root, fdir = _resolve_frame_dir(args.dataset, args.frame, args.frame_dir)
    d = _load_frame(fdir)
    _print_stats(d, fdir)

    if args.stats_only:
        return

    # 2D 텍스처 창 (intensity + color_image 있으면)
    if args.texture_2d:
        _show_texture_2d(d, title=f"{fdir.name} — 2D textures")

    # C 프레임 pcd
    pcd_C = _build_pcd_C_m(
        d,
        min_depth_mm=args.min_depth_mm,
        max_depth_mm=args.max_depth_mm,
        intensity_stretch=not args.no_intensity_stretch,
    )
    print(f"\n[viz] C 프레임 pcd: {len(pcd_C.points):,} pts  "
          f"has_colors={pcd_C.has_colors()}")

    # 축 (C frame 원점)
    axes_C = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

    geoms: list = [pcd_C, axes_C]
    if args.show_normals and pcd_C.has_normals():
        geoms.append(_normal_arrows(pcd_C, every=args.normal_every))

    # O 프레임 변환 옵션
    if args.in_O:
        T_CO = _compute_T_CO_from_meta(d["meta"], ds_root)
        if T_CO is not None:
            pcd_O = _transform_pcd_C_to_O(pcd_C, T_CO)
            # 색을 구분하기 위해 O frame pcd 는 약간 틴트
            if not pcd_O.has_colors():
                pcd_O.paint_uniform_color([0.85, 0.30, 0.25])
            print(f"[viz] O 프레임 pcd: {len(pcd_O.points):,} pts  "
                  f"(T_CO trans = {(T_CO[:3,3]*1000).round(1).tolist()} mm)")
            # 같이 보기보다, 따로 두 번 띄우는 게 직관적
            print("[viz] 창 1: C 프레임  →  Q/ESC 로 닫으면 O 프레임 창 등장")
            _show(
                geoms, f"C frame — {fdir.name}",
                lookat_default=[0.0, 0.0, float(np.mean(np.asarray(pcd_C.points)[:, 2]))]
                               if len(pcd_C.points) else [0.0, 0.0, 0.5],
            )

            axes_O = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
            # 턴테이블 z=0 평면 디스크 레퍼런스
            disc = o3d.geometry.TriangleMesh.create_cylinder(
                radius=0.125, height=0.001, resolution=64,
            )
            disc.paint_uniform_color([0.20, 0.35, 0.80])
            disc.compute_vertex_normals()
            _show(
                [pcd_O, axes_O, disc],
                f"O frame — {fdir.name}  (blue disc = turntable z=0)",
                lookat_default=[0.0, 0.0, 0.05],
            )
            return
        else:
            print("  [!] T_CO 계산 실패 — C 프레임만 표시")

    # 기본: C 프레임만
    mean_z = float(np.mean(np.asarray(pcd_C.points)[:, 2])) if len(pcd_C.points) else 0.5
    _show(geoms, f"C frame — {fdir.name}",
          lookat_default=[0.0, 0.0, mean_z])


if __name__ == "__main__":
    main()
