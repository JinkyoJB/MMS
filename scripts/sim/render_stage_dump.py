"""render_stage_dump.py — nbv 단계별 메시(MMS_SIM_STAGE_DUMP)를 같은 카메라로 렌더.

각 단계를 **고정 카메라**로 찍는다. 메시마다 bbox 에 맞춰 카메라를 새로 잡으면
크기·위치가 미묘하게 달라져 "무엇이 나빠졌나"를 눈으로 비교할 수 없다.
기준 카메라는 첫 단계(=lookaround 직후) 메시의 bbox 로 한 번만 정한다.

    python scripts/sim/render_stage_dump.py --dir scripts/sim/log/stage_dump/hand_drill
"""
from __future__ import annotations
import argparse, glob, os, re
import numpy as np
import open3d as o3d
from open3d.visualization import rendering

VIEWS = {"정면": (1.0, -0.6, 0.35), "측면": (-0.2, 1.0, 0.35), "윗면": (0.35, -0.2, 1.0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--out")
    ap.add_argument("--width", type=int, default=520)
    args = ap.parse_args()
    out = args.out or os.path.join(args.dir, "render")
    os.makedirs(out, exist_ok=True)

    files = sorted(glob.glob(os.path.join(args.dir, "*.ply")))
    if not files:
        raise SystemExit(f"PLY 없음: {args.dir}")

    ref = o3d.io.read_triangle_mesh(files[0])
    bb = ref.get_axis_aligned_bounding_box()
    c = bb.get_center()
    radius = float(np.linalg.norm(bb.get_extent())) * 0.5

    mat = rendering.MaterialRecord()
    mat.shader = "defaultLit"
    mat.base_color = (0.72, 0.74, 0.80, 1.0)
    mat.base_roughness = 0.55

    h = int(args.width * 0.86)
    r = rendering.OffscreenRenderer(args.width, h)
    r.scene.set_background([1, 1, 1, 1])
    r.scene.scene.set_sun_light([-0.4, 0.5, -0.8], [1, 1, 1], 60000)
    r.scene.scene.enable_sun_light(True)
    r.scene.scene.set_indirect_light_intensity(38000)

    for f in files:
        stem = os.path.splitext(os.path.basename(f))[0]
        m = o3d.io.read_triangle_mesh(f)
        m.vertex_colors = o3d.utility.Vector3dVector()
        m.compute_vertex_normals()
        r.scene.clear_geometry()
        r.scene.add_geometry("m", m, mat)
        for vname, d in VIEWS.items():
            d = np.asarray(d, float); d /= np.linalg.norm(d)
            eye = c + d * radius * 3.0
            r.setup_camera(45.0, c.astype(np.float32), eye.astype(np.float32),
                           np.array([0, 0, 1], np.float32))
            o3d.io.write_image(os.path.join(out, f"{stem}__{vname}.png"),
                               r.render_to_image())
        b = re.search(r"_b(\d+)_g(\d+)", stem)
        print(f"  {stem}: {len(m.vertices):7d}정점 {len(m.triangles):7d}면"
              + (f"  boundary={b.group(1)}mm gaps={b.group(2)}" if b else ""))
    print(f"\n저장: {out} ({len(files)}단계)")


if __name__ == "__main__":
    main()
