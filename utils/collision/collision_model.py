"""collision_model.py — **단계 무관 단일 충돌 게이트**. sim·real 공용.

왜 하나로 모으나
---------------
2026-08 시점 실측: nbv 만 충돌을 검사하고 **lookaround·flip 은 무검사**였다.
게다가 장애물이 3개(턴테이블 캡슐·프레임 AABB·keepout)뿐이라 벽·상판·저울·툴스탠드가
빠져 있었다 — 이건 오탐이 아니라 **미탐**(칠 수 있는데 안전하다고 함)이라 실물 파손으로
직결된다. 그래서 모든 Phase 가 부르는 게이트를 하나로 둔다.

구성 (docs/collision.md §6 의 권장 조합)
----------------------------------------
  로봇  = 링크 로컬 **메시 점군** (해석 FK 로 배치)     ← 정확. 캡슐 오탐 없음
  환경  = **SDF(복셀 거리장)**                         ← O(1) 조회. KD-tree 대비 빠름
  자가  = 링크쌍 화이트리스트(ACM 개념)
  경로  = **적응적 스텝** (고정 12스텝은 터널링 위험)

의존성은 numpy/scipy 뿐 — hpp-fcl·pinocchio 는 sim(env_isaacsim)에만 있고 실물
환경(mms-env)에는 없다. sim 에만 있는 것을 쓰면 sim/real 격차가 다시 생긴다.

캐시: `scripts/sim/export_link_meshes.py`(링크), `scripts/sim/export_env_mesh.py`(셀)
"""
from __future__ import annotations

import math
import os

import numpy as np

_HERE = os.path.dirname(__file__)
LINKS_NPZ = os.path.join(_HERE, "data", "xarm7_spider_links.npz")
ENV_NPZ = os.path.join(_HERE, "data", "cell_env.npz")

# 툴이 스칠 수 있는 링크. link6/7 은 툴이 붙은 인접 링크 → 항상 붙어 있으므로 제외.
SELF_PAIRS = (("tool", (1, 2, 3, 4, 5)),
              ("link6", (1, 2, 3)))
# 환경 검사 대상. link1/2 는 천장 마운트에 상시 근접 → 제외.
ENV_LINKS = (3, 4, 5, 6, 7)


class _Sdf:
    """점군 → 점유 복셀 → 거리변환. 조회는 trilinear 보간으로 **O(1)**.

    환경과 **각 링크**에 같은 구조를 쓴다. KD-tree 질의(점 수에 비례)를 격자 조회로
    바꾸면 자가충돌 검사가 111ms → 수 ms 로 떨어진다(실측). 우회 계획(RRT)은 자세
    검사를 수천 번 하므로 이 차이가 결정적이다.
    """

    def __init__(self, pts, voxel, pad):
        from scipy import ndimage
        pts = np.asarray(pts, float)
        self.voxel = float(voxel)
        # 격자는 점군 bbox + pad 를 덮는다. 격자 **밖** 점은 bbox 안의 어떤 점과도
        # 최소 pad 만큼 떨어져 있으므로, pad 를 돌려주면 **참인 하한**이다
        # (안전 판정에는 충분하고, 거짓 '안전'을 만들지 않는다).
        self.pad = float(pad)
        self.origin = pts.min(0) - pad
        hi = pts.max(0) + pad
        self.shape = np.maximum(
            np.ceil((hi - self.origin) / self.voxel).astype(int) + 1, 2)
        occ = np.zeros(self.shape, dtype=bool)
        idx = np.clip(np.floor((pts - self.origin) / self.voxel).astype(int),
                      0, self.shape - 1)
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        self.grid = ndimage.distance_transform_edt(~occ).astype(np.float32) * self.voxel

    @property
    def nbytes(self):
        return self.grid.nbytes

    def query(self, P):
        """점들 → 이 점군까지 거리(m). 격자 밖은 안전한 큰 값."""
        g = (np.asarray(P, float) - self.origin) / self.voxel
        i = np.floor(g).astype(int)
        inside = np.all((i >= 0) & (i < self.shape - 1), axis=1)
        out = np.full(len(g), self.pad)      # 격자 밖 = "pad 이상" (참인 하한)
        if not inside.any():
            return out
        gi, ii = g[inside], i[inside]
        f = gi - ii
        x, y, z = ii[:, 0], ii[:, 1], ii[:, 2]
        s = self.grid
        c00 = s[x, y, z] * (1 - f[:, 0]) + s[x + 1, y, z] * f[:, 0]
        c01 = s[x, y, z + 1] * (1 - f[:, 0]) + s[x + 1, y, z + 1] * f[:, 0]
        c10 = s[x, y + 1, z] * (1 - f[:, 0]) + s[x + 1, y + 1, z] * f[:, 0]
        c11 = s[x, y + 1, z + 1] * (1 - f[:, 0]) + s[x + 1, y + 1, z + 1] * f[:, 0]
        c0 = c00 * (1 - f[:, 1]) + c10 * f[:, 1]
        c1 = c01 * (1 - f[:, 1]) + c11 * f[:, 1]
        out[inside] = c0 * (1 - f[:, 2]) + c1 * f[:, 2]
        return out


class CollisionModel:
    """단일 게이트. `clearance` → `is_pose_safe` → `is_path_safe` 순으로 쓴다."""

    def __init__(self, links_npz=LINKS_NPZ, env_npz=ENV_NPZ, *,
                 self_margin_m=0.020, env_margin_m=0.025,
                 sdf_voxel_m=0.008, link_voxel_m=0.006,
                 sdf_pad_m=0.15, link_pad_m=0.12,
                 max_query_pts=2500, min_sigma=0.05, log=print):
        L = np.load(links_npz)
        self.links = {i: np.asarray(L[f"link{i}"], float) for i in range(1, 8)}
        self.tool = np.asarray(L["tool"], float)          # link7 로컬
        self.self_margin = float(self_margin_m)
        self.env_margin = float(env_margin_m)
        # 특이점 회피 — 충돌은 아니지만 **명령하면 안 되는 자세**라 같은 게이트에서 막는다.
        # 여기 두면 자세 선정·경로 보간·우회 계획이 모두 자동으로 적용받는다.
        # 근거 수치는 utils/robot/view_pose.DEFAULT_MIN_SIGMA 주석 참고.
        self.min_sigma = float(min_sigma)
        self.log = log

        def thin(V, n):
            return V[:: max(1, len(V) // max(1, n))] if len(V) else V

        # 질의용 점은 솎는다 — 거리 오차는 표면 간격(수 mm) 수준이고 여유 20~25mm 로 흡수된다.
        self._q_links = {i: thin(v, max_query_pts) for i, v in self.links.items()}
        self._q_tool = thin(self.tool, max_query_pts)
        # 링크 로컬 SDF (자가충돌 대상만)
        self._link_sdf = {}
        for _, targets in SELF_PAIRS:
            for j in targets:
                if j not in self._link_sdf and len(self.links[j]):
                    self._link_sdf[j] = _Sdf(self.links[j], link_voxel_m, link_pad_m)
        env = np.asarray(np.load(env_npz)["env"], float)
        self.env_sdf = _Sdf(env, sdf_voxel_m, sdf_pad_m)
        # ── 동적 장애물 (스캔 대상 등) ──────────────────────────────────────
        # 셀 CAD(env_npz)에는 **스캔 대상물이 없다** — "225mm standoff 로 접근하는
        # 대상"이라 일부러 뺐는데, 그 결과 자세 사이 이동 경로가 물체를 관통해도
        # 아무도 막지 않았다 (2026-09-16 실물: NBV 자세 이동 중 로봇이 대상을
        # 치고 지나감). `set_dynamic_obstacle` 로 등록하면 clearance →
        # is_pose_safe → is_path_safe → 우회계획까지 전부 자동 반영된다.
        self.dyn_sdf = None
        self.dyn_margin = float(env_margin_m)
        self._dyn_voxel = float(sdf_voxel_m)
        self._dyn_pad = float(sdf_pad_m)
        mb = (self.env_sdf.nbytes + sum(v.nbytes for v in self._link_sdf.values())) / 1e6
        self.log(f"[collision] SDF 준비 — 환경 {tuple(self.env_sdf.shape)} @ "
                 f"{sdf_voxel_m*1000:.0f}mm, 링크 {len(self._link_sdf)}개 @ "
                 f"{link_voxel_m*1000:.0f}mm, 총 {mb:.1f}MB")

    def env_distance(self, P):
        return self.env_sdf.query(P)

    def set_dynamic_obstacle(self, pts_B, margin_m: float = None):
        """런타임 장애물(스캔 대상) 등록. pts_B = (N,3) base 프레임 m. None = 해제.

        margin 이 env_margin 과 달라도 호출측 API(slack/is_pose_safe)가 그대로
        동작하도록, 조회 거리를 `d − margin + env_margin` 으로 정규화해 env 거리와
        같은 잣대로 합산한다 (slack = min − env_margin 이므로 실효 여유는 margin)."""
        if pts_B is None or len(pts_B) == 0:
            self.dyn_sdf = None
            return
        P = np.asarray(pts_B, float)
        if len(P) > 20000:
            P = P[:: len(P) // 20000]
        self.dyn_sdf = _Sdf(P, self._dyn_voxel, self._dyn_pad)
        if margin_m is not None:
            self.dyn_margin = float(margin_m)
        self.log(f"[collision] 동적 장애물 등록 — {len(P):,}pt "
                 f"margin {self.dyn_margin*1000:.0f}mm")

    # ── 배치 ───────────────────────────────────────────────────────────────
    def _placed(self, q, tool_pts=None):
        """FK 로 링크·툴 점을 base 프레임에 배치. {이름: (점, 프레임)}"""
        from utils.robot import xarm7_kinematics as kin
        F = kin._fk_frames_m(np.asarray(q, float))
        out = {}
        for i in range(1, 8):
            V = self._q_links[i]
            if len(V):
                T = F[i]
                out[f"link{i}"] = (V @ T[:3, :3].T + T[:3, 3], T)
        tp = self._q_tool if tool_pts is None else np.asarray(tool_pts, float)
        if len(tp):
            T = F[7]
            out["tool"] = (tp @ T[:3, :3].T + T[:3, 3], T)
        return out, F

    # ── 공개 API ───────────────────────────────────────────────────────────
    def clearance(self, q, tool_pts=None):
        """(self_min_m, env_min_m, label). 가장 가까운 쌍/부위 이름을 함께 준다."""
        placed, F = self._placed(q, tool_pts)
        # 자가: 화이트리스트 쌍만
        s_min, s_who = 1e9, ""
        for name, targets in SELF_PAIRS:
            if name not in placed:
                continue
            P = placed[name][0]
            for j in targets:
                sdf = self._link_sdf.get(j)
                if sdf is None:
                    continue
                loc = (P - F[j][:3, 3]) @ F[j][:3, :3]      # 링크 j 로컬로
                d = float(sdf.query(loc).min())
                if d < s_min:
                    s_min, s_who = d, f"{name}↔link{j}"
        # 환경: SDF 조회 (+동적 장애물 — margin 차를 정규화해 같은 잣대로)
        e_min, e_who = 1e9, ""
        _dyn_off = self.dyn_margin - self.env_margin
        for name in [f"link{i}" for i in ENV_LINKS] + ["tool"]:
            if name not in placed:
                continue
            d = float(self.env_distance(placed[name][0]).min())
            if d < e_min:
                e_min, e_who = d, name
            if self.dyn_sdf is not None:
                d2 = float(self.dyn_sdf.query(placed[name][0]).min()) - _dyn_off
                if d2 < e_min:
                    e_min, e_who = d2, f"{name}↔대상물"
        return s_min, e_min, (s_who if s_min - self.self_margin < e_min - self.env_margin
                              else e_who)

    def is_pose_safe(self, q, tool_pts=None):
        """(safe, reason). reason='' 이면 안전. 충돌 + **특이점** 둘 다 본다."""
        if self.min_sigma > 0.0:
            try:
                from utils.robot import xarm7_kinematics as kin
                sig = kin.sigma_min(q)
                if sig < self.min_sigma:
                    return False, f"singular(σ={sig:.3f}<{self.min_sigma:.3f})"
            except Exception:
                pass
        s, e, who = self.clearance(q, tool_pts)
        if s < self.self_margin:
            return False, f"self({who},{s*1000:.0f}mm)"
        if e < self.env_margin:
            return False, f"env({who},{e*1000:.0f}mm)"
        return True, ""

    def slack(self, q, tool_pts=None):
        """여유(m) — **거리만** 본다(특이점은 is_pose_safe 가 별도로 막는다). — 자가·환경 각각의 margin 을 뺀 값 중 **작은 쪽**.
        0 이하면 충돌. 이 값이 '앞으로 이 거리만큼은 움직여도 안전하다'는 보증이 된다."""
        s, e, who = self.clearance(q, tool_pts)
        return min(s - self.self_margin, e - self.env_margin), who

    def max_point_travel(self, q0, q1):
        """두 자세 사이 로봇 점의 **최대 이동거리**(m) 상한.

        직선 관절보간에서 각 점은 이 거리 이상 움직이지 않는다(회전 호는 현보다 길지만
        RRT 스텝 크기에서는 무시할 수준이라, 안전계수 1.15 를 곱해 상한으로 쓴다)."""
        p0, _ = self._placed(q0)
        p1, _ = self._placed(q1)
        d = 0.0
        for k in p0:
            if k in p1:
                d = max(d, float(np.linalg.norm(p1[k][0] - p0[k][0], axis=1).max()))
        return d * 1.15

    def is_path_safe(self, q0, q1, tool_pts=None, resolution_m=0.004, max_checks=600):
        """(safe, reason, n_checks) — **보수적 전진 + 이분법**.

        핵심: 자세 q 의 여유가 s 이고, 구간에서 어떤 점도 s 보다 멀리 움직이지 않으면
        그 구간은 **중간 검사 없이 안전이 보증**된다. 고정 N-스텝 샘플링보다
        (1) 빠르고 — 여유가 큰 구간은 검사 자체를 건너뛴다,
        (2) 엄밀하다 — 샘플 사이로 얇은 장애물을 지나치는 **터널링이 원리적으로 불가**.
        """
        q0 = np.asarray(q0, float)
        q1 = np.asarray(q1, float)
        s0, w0 = self.slack(q0, tool_pts)
        if s0 <= 0:
            return False, f"start({w0})", 1
        s1, w1 = self.slack(q1, tool_pts)
        if s1 <= 0:
            return False, f"goal({w1})", 2
        n = 2
        stack = [(q0, s0, q1, s1)]
        while stack:
            if n > max_checks:
                return False, "검사 한도 초과(경로가 너무 아슬아슬)", n
            a, sa, b, sb = stack.pop()
            L = self.max_point_travel(a, b)
            if min(sa, sb) > L:
                continue                       # ★ 보증됨 — 중간 검사 불필요
            if L <= resolution_m:
                continue                       # 충분히 촘촘 (양 끝이 안전)
            m = 0.5 * (a + b)
            sm, wm = self.slack(m, tool_pts)
            n += 1
            if sm <= 0:
                return False, f"mid({wm})", n
            stack.append((a, sa, m, sm))
            stack.append((m, sm, b, sb))
        return True, "", n


_cached = {}


def get_default(**kw):
    """캐시가 없으면 None — 호출부는 기존 방식으로 폴백."""
    key = tuple(sorted(kw.items()))
    if key not in _cached:
        try:
            _cached[key] = CollisionModel(**kw)
        except Exception as e:                       # noqa: BLE001
            print(f"[collision] 모델 로드 실패({type(e).__name__}: {e}) — "
                  f"캐시 생성: scripts/sim/export_link_meshes.py, export_env_mesh.py")
            _cached[key] = None
    return _cached[key]
