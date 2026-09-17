"""isaac_recorder.py — 스캔 실행을 3인칭 시점 영상으로 녹화 (자료용).

headless 에서도 동작한다. 씬에 관찰 카메라를 만들고 replicator 렌더 프로덕트를
붙여 프레임을 PNG 로 떨어뜨린 뒤, 종료 시 ffmpeg 로 mp4 를 만든다.

    MMS_SIM_REC=1                      켜기 (기본 꺼짐 — 프레임 쓰기 비용이 있다)
    MMS_SIM_REC_DIR=<dir>              프레임·영상 출력 (기본 scripts/sim/log/rec)
    MMS_SIM_REC_EVERY=3                N 스텝마다 1프레임 (기본 3)
    MMS_SIM_REC_RES=1280,720           해상도
    MMS_SIM_REC_FPS=30                 출력 fps
    MMS_SIM_REC_ORBIT=1                턴테이블처럼 카메라를 천천히 선회

단계마다 파일이 나뉜다(`Phase_1.mp4` …). 종료 시 전체본도 만든다.

⚠ 스캐너 카메라(annotator)와 별개 자원이다. 스캐너 렌더 프로덕트를 재사용하면
  캡처 해상도가 바뀌어 스캔 결과가 달라진다.
"""
from __future__ import annotations

import math
import os
import subprocess

import numpy as np


def _envf(k, d):
    try:
        return float(os.environ.get(k, d))
    except Exception:                                    # noqa: BLE001
        return float(d)


ENABLED  = os.environ.get("MMS_SIM_REC", "0") == "1"
OUT_DIR  = os.environ.get("MMS_SIM_REC_DIR", "scripts/sim/log/rec")
EVERY    = int(_envf("MMS_SIM_REC_EVERY", 3))
FPS      = int(_envf("MMS_SIM_REC_FPS", 30))
ORBIT    = os.environ.get("MMS_SIM_REC_ORBIT", "0") == "1"
# 1 이면 단계마다 파일 분리, 0 이면 전체를 한 파일로 (자료용 기본)
SPLIT    = os.environ.get("MMS_SIM_REC_SPLIT", "0") == "1"
ORBIT_DEG_PER_FRAME = _envf("MMS_SIM_REC_ORBIT_SPEED", 0.15)
RES = tuple(int(x) for x in os.environ.get("MMS_SIM_REC_RES", "1280,720").split(",")[:2])

CAM_PATH = "/World/Environment/RecCam"
# 대상물과 스캐너가 화면을 채우는 시점. 셀 기둥이 시야를 가르지 않는 방위를 고른다.
# 타깃은 대상물 중심 높이, 거리는 작업 반경 수준으로 좁힌다. 전부 env 로 조정 가능.
TARGET   = tuple(float(x) for x in
                 os.environ.get("MMS_SIM_REC_TARGET", "0.365,0.0,0.85").split(","))
DIST     = _envf("MMS_SIM_REC_DIST", 1.7)
ELEV_DEG = _envf("MMS_SIM_REC_ELEV", 40.0)
AZIM_DEG = _envf("MMS_SIM_REC_AZIM", 250.0)
FOCAL    = _envf("MMS_SIM_REC_FOCAL", 24.0)


class Recorder:
    """스캔 세션이 매 스텝 `tick()` 을 부르면 프레임을 쌓는다."""

    def __init__(self, stage, out_dir: str = None):
        self.ok = False
        self.n = 0
        self._i = 0
        self._seg = None            # 현재 구간 이름
        self._seg_n = 0             # 현재 구간의 프레임 수
        self.parts = []             # 완성된 구간 mp4 목록
        self.dir = out_dir or OUT_DIR
        if not ENABLED:
            return
        try:
            import omni.replicator.core as rep
            from pxr import UsdGeom, Gf
            os.makedirs(self.dir, exist_ok=True)
            self._rep, self._UsdGeom, self._Gf = rep, UsdGeom, Gf
            self.stage = stage
            self._azim = AZIM_DEG
            cam = UsdGeom.Camera.Define(stage, CAM_PATH)
            cam.CreateFocalLengthAttr(FOCAL)
            self._cam = cam
            self._place(self._azim)
            self._rp = rep.create.render_product(CAM_PATH, RES)
            self._annot = rep.AnnotatorRegistry.get_annotator("rgb")
            self._annot.attach([self._rp])
            self.ok = True
            print(f"[rec] 녹화 시작 — {RES[0]}×{RES[1]}, {EVERY}스텝마다, → {self.dir}")
        except Exception as e:                           # noqa: BLE001
            print(f"[rec] ⚠ 녹화 초기화 실패({type(e).__name__}: {e}) — 녹화 없이 진행")

    def _place(self, azim_deg: float):
        Gf = self._Gf
        el, az = math.radians(ELEV_DEG), math.radians(azim_deg)
        t = np.array(TARGET, float)
        eye = t + DIST * np.array([math.cos(el) * math.cos(az),
                                   math.cos(el) * math.sin(az),
                                   math.sin(el)])
        f = t - eye; f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r)
        u = np.cross(r, f)
        M = Gf.Matrix4d(*[float(v) for row in (
            list(r) + [0.0], list(u) + [0.0], list(-f) + [0.0], list(eye) + [1.0])
            for v in row])
        x = self._UsdGeom.Xformable(self._cam.GetPrim())
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(M)

    def tick(self):
        """월드 스텝 뒤에 호출. EVERY 스텝마다 1프레임 저장."""
        if not self.ok:
            return
        self._i += 1
        if self._i % EVERY:
            return
        try:
            data = self._annot.get_data()
            if data is None or not len(data):
                return
            img = np.asarray(data)[:, :, :3]
            from PIL import Image
            seg = self._seg or "seg"
            Image.fromarray(img.astype(np.uint8)).save(
                os.path.join(self.dir, f"{seg}_{self._seg_n:06d}.png"))
            self._seg_n += 1
            self.n += 1
            if ORBIT:
                self._azim += ORBIT_DEG_PER_FRAME
                self._place(self._azim)
        except Exception as e:                           # noqa: BLE001
            print(f"[rec] ⚠ 프레임 저장 실패({type(e).__name__}: {e}) — 녹화 중단")
            self.ok = False

    # ── 구간(Phase) 관리 ────────────────────────────────────────────────
    def begin(self, label: str):
        """새 구간 시작. 직전 구간이 있으면 먼저 인코딩한다.

        단계마다 파일을 나누면 자료로 쓸 때 필요한 구간만 골라 넣을 수 있고,
        한 파일이 지나치게 길어지지 않는다.
        """
        if not self.ok:
            return
        if not SPLIT:                       # 단일 파일 모드 — 구간 경계만 로그
            if self._seg is None:
                self._seg = "scan_all"
            print(f"[rec] 구간 — {self._slug(label)} (프레임 {self._seg_n})")
            return
        if self._seg is not None:
            self._encode(self._seg)
        self._seg = self._slug(label)
        self._seg_n = 0
        print(f"[rec] 구간 시작 — {self._seg}")

    @staticmethod
    def _slug(label: str) -> str:
        keep = []
        for ch in label:
            if ch.isalnum():
                keep.append(ch)
            elif ch in " -_":
                keep.append("_")
        s = "".join(keep).strip("_")
        while "__" in s:
            s = s.replace("__", "_")
        return s or "seg"

    def _encode(self, seg: str):
        """한 구간의 PNG 시퀀스를 mp4 로 인코딩하고 프레임을 지운다."""
        pat = os.path.join(self.dir, f"{seg}_%06d.png")
        n = len([f for f in os.listdir(self.dir)
                 if f.startswith(seg + "_") and f.endswith(".png")])
        if n < 2:
            return None
        out = os.path.join(self.dir, f"{seg}.mp4")
        cmd = ["ffmpeg", "-y", "-framerate", str(FPS), "-i", pat,
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
               "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", out]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            for f in os.listdir(self.dir):
                if f.startswith(seg + "_") and f.endswith(".png"):
                    os.remove(os.path.join(self.dir, f))
            print(f"[rec] 저장 — {out} ({n}프레임, {n/FPS:.1f}초)")
            self.parts.append(out)
            return out
        except subprocess.CalledProcessError as e:
            print(f"[rec] ⚠ 인코딩 실패({seg}): {e.stderr.decode()[-300:]}")
            return None

    def finish(self, name: str = "scan_all"):
        """마지막 구간을 인코딩하고, 구간이 여럿이면 이어붙인 전체본도 만든다."""
        if not self.ok:
            return None
        if self._seg is not None:
            self._encode(self._seg)
            self._seg = None
        if len(self.parts) < 2:
            return self.parts[0] if self.parts else None
        lst = os.path.join(self.dir, "_parts.txt")
        with open(lst, "w") as f:
            for p in self.parts:
                f.write(f"file '{os.path.abspath(p)}'\n")
        out = os.path.join(self.dir, f"{name}.mp4")
        try:
            subprocess.run(["ffmpeg", "-y", "-f", "concat", "-safe", "0",
                            "-i", lst, "-c", "copy", out],
                           check=True, capture_output=True)
            os.remove(lst)
            print(f"[rec] 전체본 — {out} (구간 {len(self.parts)}개)")
            return out
        except subprocess.CalledProcessError as e:
            print(f"[rec] ⚠ 이어붙이기 실패: {e.stderr.decode()[-300:]}")
            return None
