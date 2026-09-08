"""make_paper_fig6_results.py — 논문 Fig. 6 (파이프라인 복원 결과, 테스트셋 9종).

3행(GT / front / bottom) 블록을 두 개 쌓아 6행으로 만든다.
  블록 1: mustard, hand drill, alarm clock, spray can, mug
  블록 2: laundry detergent, drug bottle, povidone iodine, protein drink
Table 6 의 대상과 순서를 그대로 따른다.

입력  figures/gt/<key>_front.png        (render_gt_meshes.py)
      figures/results/<key>_<뷰>.jpg    (Isaac 스윕 결과 렌더)
산출  figures/fig_results.png
"""
from __future__ import annotations
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.gridspec import GridSpec

BASE = os.path.join(os.path.dirname(__file__), "..", "..", "..",
                    "8_paper", "2_논문작성", "figures")
BASE = os.path.abspath(BASE)

BLOCKS = [
    [("mustard", "mustard"), ("hand_drill", "hand\ndrill"),
     ("alarm_clock", "alarm\nclock"), ("spray_can", "spray\ncan"),
     ("mug", "mug")],
    [("laundry_detergent", "laundry\ndetergent"), ("drug_bottle", "drug\nbottle"),
     ("povidone_iodine", "povidone\niodine"), ("protein_drink", "protein\ndrink")],
]
ROWS = [("GT", None), ("front", "정면"), ("bottom", "아랫면")]
NCOL = 5
FS = 7.5


def crop(im):
    """배경(모서리 색)과 다른 화소의 경계상자로 잘라낸다."""
    g = im[..., :3].mean(-1) if im.ndim == 3 else im
    bg = float(np.median([g[0, 0], g[0, -1], g[-1, 0], g[-1, -1]]))
    m = np.abs(g - bg) > (0.02 if g.max() <= 1.0 else 5.0)
    if not m.any():
        return im
    ys, xs = np.where(m)
    pad = int(0.03 * max(im.shape[0], im.shape[1]))
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad + 1, im.shape[0])
    x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad + 1, im.shape[1])
    return im[y0:y1, x0:x1]


def draw(ax, path):
    ax.imshow(crop(mpimg.imread(path)))
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)


def main():
    fig = plt.figure(figsize=(3.1, 3.32), dpi=300)
    gs = GridSpec(2, 1, figure=fig, hspace=0.11,
                  left=0.075, right=0.995, top=0.955, bottom=0.005)

    for bi, objs in enumerate(BLOCKS):
        sub = gs[bi].subgridspec(3, NCOL, hspace=0.03, wspace=0.04)
        for ri, (rlab, view) in enumerate(ROWS):
            for ci in range(NCOL):
                ax = fig.add_subplot(sub[ri, ci])
                if ci >= len(objs):
                    ax.axis("off")
                    continue
                key, label = objs[ci]
                path = (os.path.join(BASE, "gt", f"{key}_front.png") if view is None
                        else os.path.join(BASE, "results", f"{key}_{view}.jpg"))
                draw(ax, path)
                if ri == 0:
                    ax.set_title(label, fontsize=FS, pad=1.5, linespacing=0.95)
                if ci == 0:
                    ax.set_ylabel(rlab, fontsize=FS, labelpad=1.5)

    out = os.path.join(BASE, "fig_results.png")
    fig.savefig(out, dpi=300, facecolor="white",
                bbox_inches="tight", pad_inches=0.01)
    print("saved", out)


if __name__ == "__main__":
    main()
