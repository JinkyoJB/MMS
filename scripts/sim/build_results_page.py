"""build_results_page.py — testset 스윕 결과를 HTML 페이지로 조립 (docs/testset_results.md 의 웹판).

    python scripts/sim/build_results_page.py   # → /tmp/testset_results.html (Artifact 게시용)
"""
import base64, glob, io, json, os, re
from PIL import Image

BASE_R, BASE_L = "scripts/sim/log/testset_sweep/render", "scripts/sim/log/testset_sweep"
VIEWS = ["정면", "측면", "윗면", "아랫면"]
idx = {r["name"]: r for r in json.load(open(f"{BASE_R}/index.json"))}

def src(name):
    return (BASE_R, BASE_L)

def jpg(p, w=430):
    im = Image.open(p).convert("RGB")
    im = im.resize((w, int(im.height * w / im.width)), Image.LANCZOS)
    b = io.BytesIO(); im.save(b, "JPEG", quality=86, optimize=True)
    return "data:image/jpeg;base64," + base64.b64encode(b.getvalue()).decode()

rows = []
for name in sorted(idx):
    R, LOGS = src(name)
    r0src = idx
    t = open(f"{LOGS}/{name}.log", errors="ignore").read()
    m = re.search(r"r=(\d+)mm h=(\d+)mm", t)
    bands = re.search(r"P1 플랜: (\d+) bands", t)
    b = re.findall(r"boundary=(\d+)mm cov=[\d.]+ gaps=(\d+)", t)
    pts = re.search(r"완료 — 누적 (\d+)점", t)
    failed = "이동 불가 — 이 패스 건너뜀" in t
    r0 = r0src[name]
    rows.append(dict(
        name=name, label=name.split("_", 1)[1].replace("_", " "),
        r=int(m.group(1)) if m else None, h=int(m.group(2)) if m else None,
        bands=int(bands.group(1)) if bands else 1,
        p1=(int(b[0][0]), int(b[0][1])) if b else None,
        p2=(int(b[-1][0]), int(b[-1][1])) if b else None,
        pts=int(pts.group(1)) if pts else 0,
        faces=r0["faces"], bbox=r0["bbox_mm"], failed=failed,
        fixed=False,
        flip="적용" if "flip 국소정합 적용" in t else ("hint 폴백" if "hint 그대로" in t else "—"),
        imgs={v: jpg(f"{R}/{name}__{v}.png") for v in VIEWS
              if os.path.exists(f"{R}/{name}__{v}.png")}))

ok = [r for r in rows if r["p1"]]
improved = [r for r in ok if r["p2"][0] < r["p1"][0] * 0.97]
json.dump([{k: v for k, v in r.items() if k != "imgs"} for r in rows],
          open("/tmp/rows.json", "w"), ensure_ascii=False, indent=1)

def bar(r):
    if not r["p1"]:
        return '<td class="num">—</td><td class="delta"><span class="chip fail">중단</span></td>'
    d = (r["p1"][0] - r["p2"][0]) / r["p1"][0] * 100
    w = min(abs(d), 60) / 60 * 100
    cls = "pos" if d > 3 else ("neg" if d < -3 else "flat")
    sign = "−" if d > 0 else ("+" if d < -0.5 else "±")
    return (f'<td class="num">{r["p2"][0]}<span class="u">mm</span> '
            f'<span class="g">/ {r["p2"][1]}</span></td>'
            f'<td class="delta"><span class="track"><i class="{cls}" style="width:{w:.0f}%"></i></span>'
            f'<b class="{cls}">{sign}{abs(d):.0f}%</b></td>')

trs = "\n".join(
    f'<tr class="{"bad" if r["failed"] else ""}">'
    f'<th scope="row">{r["label"]}<span class="dim">{r["h"]}×{r["r"]*2}mm'
    f'{" · "+str(r["bands"])+"밴드" if r["bands"]>1 else ""}</span></th>'
    f'<td class="num">{r["p1"][0]}<span class="u">mm</span> <span class="g">/ {r["p1"][1]}</span></td>'
    if r["p1"] else
    f'<tr class="bad"><th scope="row">{r["label"]}<span class="dim">{r["h"]}×{r["r"]*2}mm · {r["bands"]}밴드</span></th>'
    f'<td class="num">—</td>'
    for r in rows)
# 위 표현이 복잡해 재작성
trs = ""
for r in rows:
    p1 = (f'{r["p1"][0]}<span class="u">mm</span> <span class="g">/ {r["p1"][1]}</span>'
          if r["p1"] else "—")
    tag = prev = ''
    trs += (f'<tr class="{"bad" if r["failed"] else ("fix" if r["fixed"] else "")}">'
            f'<th scope="row"><span class="nm">{r["label"]} {tag}</span>'
            f'<span class="dim">{r["h"]}×{r["r"]*2}mm'
            f'{" · "+str(r["bands"])+"밴드" if r["bands"]>1 else ""} {prev}</span></th>'
            f'<td class="num">{p1}</td>{bar(r)}'
            f'<td class="num sm">{r["pts"]:,}</td>'
            f'<td class="sm">{r["flip"]}</td></tr>\n')

cards = ""
for r in rows:
    ims = "".join(f'<figure><img src="{r["imgs"][v]}" alt="{r["label"]} {v}" loading="lazy">'
                  f'<figcaption>{v}</figcaption></figure>' for v in VIEWS if v in r["imgs"])
    if r["failed"]:
        note = '<p class="note fail">밴드 도달 실패 — 해당 대역이 비어 있다.</p>'
    else:
        note = ""
    st = ('<span class="chip fail">중단</span>' if r["failed"]
          else f'<span class="chip {"fix" if r["fixed"] else "ok"}">'
               f'{r["p1"][0]}→{r["p2"][0]}mm</span>')
    cards += (f'<article class="card {"bad" if r["failed"] else ""}">'
              f'<header><h3>{r["label"]}</h3>{st}'
              f'<span class="meta">{r["faces"]:,}면 · bbox {"×".join(str(int(x)) for x in r["bbox"])}mm</span>'
              f'</header><div class="views">{ims}</div>{note}</article>\n')

# GT 대조 표 (9종 최종 메시 vs 씬 정답 표면)
gtall = json.load(open("/tmp/gt_all.json"))
gtrs = ""
for g in gtall:
    lbl = g["name"].split("_",1)[1].replace("_"," ")
    f1 = g["f@1mm"]*100
    w = min(f1,100)
    cls = "pos" if f1>=90 else ("flat" if f1>=65 else "neg")
    star = ' <span class="chip fix">flip 90°</span>' if g["name"]=="0101_spray_can" else ""
    gtrs += (f'<tr><th scope="row"><span class="nm">{lbl}{star}</span></th>'
             f'<td class="num">{g["compl@1mm"]*100:.1f}<span class="u">%</span></td>'
             f'<td class="num">{g["acc@1mm"]*100:.1f}<span class="u">%</span></td>'
             f'<td class="delta"><span class="track"><i class="{cls}" style="width:{w:.0f}%"></i></span>'
             f'<b class="{cls}">{f1:.1f}%</b></td>'
             f'<td class="num">{g["f@2mm"]*100:.1f}<span class="u">%</span></td>'
             f'<td class="num">{g["chamfer_mm"]:.2f}<span class="u">mm</span></td></tr>\n')

html = open("scripts/sim/results_page_tmpl.html").read()
html = html.replace("{{GTRS}}", gtrs)
html = (html.replace("{{TRS}}", trs).replace("{{CARDS}}", cards)
        .replace("{{NOK}}", str(len(ok))).replace("{{NIMP}}", str(len(improved)))
        .replace("{{NFAIL}}", str(len(rows) - len(ok))))
open("/tmp/testset_results.html", "w").write(html)
print("작성:", len(html) // 1024, "KB")
