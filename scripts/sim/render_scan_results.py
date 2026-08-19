"""render_scan_results.py — 스캔 결과 OBJ 를 물체명으로 정리하고 4방향 이미지로 렌더.

파이프라인이 저장하는 OBJ 는 `artec_phase1_<타임스탬프>.obj` 라 물체를 알 수 없다.
스윕 로그에서 경로를 뽑아 `<물체명>.obj` 로 복사하고, 정면/측면/윗면/**아랫면**을
렌더한다(아랫면 = Phase 3 flip 이 바닥을 실제로 취득했는지 보는 뷰).

    python scripts/sim/render_scan_results.py --logs scripts/sim/log/testset_sweep
"""
from __future__ import annotations
import argparse, glob, json, os, re, shutil
import numpy as np
import open3d as o3d
from open3d.visualization import rendering

VIEWS = {                      # (eye 방향, up) — 물체 중심 기준 단위벡터
    "정면": ((1.0, -0.6, 0.35), (0, 0, 1)),
    "측면": ((-0.2, 1.0, 0.35), (0, 0, 1)),
    "윗면": ((0.35, -0.2, 1.0), (0, 0, 1)),
    "아랫면": ((0.35, -0.2, -1.0), (0, 0, 1)),
}


def render(mesh, w=560, h=460):
    # 스캔 메시는 정점색을 들고 있는데 대개 어두워서 형상이 새까맣게 렌더된다.
    # 색을 지우고 균일 재질 + 조명으로 굴곡(=스캔 품질)이 보이게 한다.
    mesh.vertex_colors = o3d.utility.Vector3dVector()
    mesh.compute_vertex_normals()
    bb = mesh.get_axis_aligned_bounding_box()
    c = bb.get_center()
    radius = float(np.linalg.norm(bb.get_extent())) * 0.5
    mat = rendering.MaterialRecord()
    mat.shader = "defaultLit"
    mat.base_color = (0.72, 0.74, 0.80, 1.0)
    mat.base_roughness = 0.55
    out = {}
    r = rendering.OffscreenRenderer(w, h)
    r.scene.set_background([1, 1, 1, 1])
    r.scene.add_geometry("m", mesh, mat)
    r.scene.scene.set_sun_light([-0.4, 0.5, -0.8], [1, 1, 1], 60000)
    r.scene.scene.enable_sun_light(True)
    r.scene.scene.set_indirect_light_intensity(38000)
    for name, (d, up) in VIEWS.items():
        d = np.asarray(d, float); d /= np.linalg.norm(d)
        eye = c + d * radius * 3.0
        r.setup_camera(45.0, c.astype(np.float32), eye.astype(np.float32),
                       np.asarray(up, np.float32))
        out[name] = r.render_to_image()      # o3d Image (write_image 로 저장)
    del r
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", default="scripts/sim/log/testset_sweep")
    ap.add_argument("--out", default="scripts/sim/log/testset_sweep/render")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for lg in sorted(glob.glob(os.path.join(args.logs, "*.log"))):
        name = os.path.basename(lg)[:-4]
        txt = open(lg, errors="ignore").read()
        m = re.findall(r"saved OBJ → (\S+\.obj)", txt)
        if not m or not os.path.isfile(m[-1]):
            print(f"  건너뜀 {name}: OBJ 없음")
            continue
        dst = os.path.join(args.out, f"{name}.obj")
        shutil.copy(m[-1], dst)
        mesh = o3d.io.read_triangle_mesh(dst)
        if len(mesh.triangles) == 0:
            print(f"  건너뜀 {name}: 삼각형 0")
            continue
        ext = mesh.get_axis_aligned_bounding_box().get_extent() * 1000.0
        for vname, img in render(mesh).items():
            o3d.io.write_image(os.path.join(args.out, f"{name}__{vname}.png"), img)
        b = re.findall(r"boundary=(\d+)mm cov=[\d.]+ gaps=(\d+)", txt)
        rows.append(dict(name=name, verts=len(mesh.vertices), faces=len(mesh.triangles),
                         bbox_mm=[round(float(x), 1) for x in ext],
                         p1=b[0] if b else None, p2=b[-1] if b else None,
                         watertight=bool(mesh.is_watertight()),
                         obj=os.path.basename(dst)))
        print(f"  ✔ {name}: {len(mesh.vertices)}정점 {len(mesh.triangles)}면 "
              f"bbox={np.round(ext,1).tolist()}mm watertight={mesh.is_watertight()}")
    json.dump(rows, open(os.path.join(args.out, "index.json"), "w"),
              ensure_ascii=False, indent=1)
    print(f"\n저장: {args.out}  ({len(rows)}개)")


if __name__ == "__main__":
    main()
