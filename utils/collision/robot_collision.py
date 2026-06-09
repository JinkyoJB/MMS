"""
robot_collision.py — xArm7 자세별 충돌 쿼리 (real/sim 공용, sensor 무관).

원리
----
이상적 키네마틱 로봇(위치제어)에선 물리 충돌 '응답'을 켜면 드라이브와 싸워 진동만
난다(막아주지 않음). 진짜 충돌 '방지'는 자세를 **실행 전/후에 기하로 검사**해 닿는
자세를 거르는 것. 로봇을 **캡슐**(선분+반경)들로 근사하고, 장애물(턴테이블/객체=캡슐,
바닥=반평면)과 캡슐거리로 판정한다.

좌표
----
모든 것은 **로봇 base 프레임(m)**. 턴테이블 축·표면은 calibration 이 base 로 주므로
`CollisionWorld.from_turntable(...)` 가 calibration 결과로 바로 월드를 만든다.

로봇 캡슐(링크 포즈)은 **실제 운동학**에서 와야 정확하다:
  - sim : USD 링크 prim 의 실제 world 포즈 → base (IsaacXArm.collision_capsules)
  - real: 공칭 xArm7 해석 FK 또는 xArm SDK FK (XArmInterface.collision_capsules)
이 모듈은 공통 **기하·월드·판정**과 캡슐 빌더를 제공한다(백엔드는 링크 원점만 공급).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from utils.collision.geometry import seg_seg_distance, seg_halfspace_min_signed

# 링크/스캐너 캡슐 기본 반경(m). 이웃 링크원점 사이 캡슐 i 의 반경.
# 순서: [link1-2, link2-3, link3-4, link4-5, link5-6, link6-7, link7-scanner]
DEFAULT_LINK_RADII = np.array([0.060, 0.060, 0.055, 0.050, 0.050, 0.050, 0.060])


@dataclass
class Capsule:
    p0: np.ndarray
    p1: np.ndarray
    r: float


@dataclass
class CollisionResult:
    collide: bool
    min_clearance: float                                  # 최소 여유(m, 음수=관통)
    contacts: List[Tuple[str, str, float]] = field(default_factory=list)  # (robot, obstacle, 침투 m)

    def __bool__(self):
        return self.collide


def capsules_from_origins(origins: Sequence[np.ndarray],
                          radii: Sequence[float] = DEFAULT_LINK_RADII,
                          names: Optional[Sequence[str]] = None
                          ) -> List[Tuple[str, Capsule]]:
    """
    링크 원점들(base m, 순서대로) → 이웃을 잇는 캡슐 리스트.

    origins (N,3) → 캡슐 N-1 개. radii 는 캡슐별 반경. (a=d=0 으로 원점이 겹친
    링크는 길이 0 캡슐=구가 되어도 무방.)
    """
    o = np.asarray(origins, float)
    m = len(o) - 1
    if names is None:
        names = [f"seg{i}" for i in range(m)]
    caps = []
    for i in range(m):
        caps.append((names[i], Capsule(o[i], o[i + 1], float(radii[i]))))
    return caps


def capsules_from_joints(q, link_radii=DEFAULT_LINK_RADII,
                         T_EC=None, scanner_len: float = 0.13
                         ) -> List[Tuple[str, Capsule]]:
    """
    관절각(rad,7) → **공칭 xArm7 해석 FK** 로 충돌 캡슐 (base m).

    real(실물) 경로용 — 공칭 DH 가 실물과 일치한다는 전제. ★ sim USD 는 공칭과
    어긋나므로(검증됨) sim 에선 이 함수 대신 실제 링크 포즈를 써라
    (IsaacXArm.collision_capsules). 가상 자세 사전질의(pre-move)에 적합.

    T_EC : (4,4, m) 손-눈(E→C). 주면 스캐너 끝 = 플랜지·T_EC; 없으면 플랜지 -Z 로
           scanner_len 만큼 연장.
    """
    from utils.robot.xarm7_kinematics import fk_link_origins, fk_T
    q = np.asarray(q, float)
    o = fk_link_origins(q)                       # (8,3) base m: [base, o1..o7(flange)]
    T = fk_T(q); flange = T[:3, 3] / 1000.0
    if T_EC is not None:
        Tm = T.copy(); Tm[:3, 3] /= 1000.0
        tip = (Tm @ np.asarray(T_EC, float))[:3, 3]
    else:
        tip = flange - T[:3, 2] * scanner_len    # 플랜지 -Z 로 연장
    origins = list(o[1:8]) + [tip]               # o1..o7(=flange) + 스캐너 끝 = 8점
    names = ["link1", "link2", "link3", "link4", "link5", "link6", "scanner"]
    return capsules_from_origins(origins, link_radii, names)


@dataclass
class CollisionWorld:
    """장애물 모음 (base 프레임, m). 캡슐 장애물 + 반평면(바닥/테이블)."""
    obstacles: List[Tuple[str, Capsule]] = field(default_factory=list)
    halfspaces: List[Tuple[str, np.ndarray, np.ndarray]] = field(default_factory=list)

    # ── 장애물 추가 ──────────────────────────────────────────────────────────
    def add_capsule(self, name, p0, p1, r):
        self.obstacles.append((name, Capsule(np.asarray(p0, float), np.asarray(p1, float), float(r))))
        return self

    def add_halfspace(self, name, point, normal):
        n = np.asarray(normal, float); n = n / (np.linalg.norm(n) + 1e-12)
        self.halfspaces.append((name, np.asarray(point, float), n))
        return self

    # ── calibration 결과로 턴테이블 월드 구성 ────────────────────────────────
    @classmethod
    def from_turntable(cls, surface_point, axis_dir, disc_radius,
                       body_height=0.20, object_radius=0.0, object_height=0.0,
                       margin=0.0, base_origin=(0.0, 0.0, 0.0)):
        """
        calibration(축+표면, base 프레임)으로 충돌 월드 생성.

        surface_point : disc 표면 위 중심(=F0 원점, base m)
        axis_dir      : 회전축 단위방향(base). 부호 자동정렬(객체/스캐너 쪽 = base 원점 쪽).
        disc_radius   : disc(+프레임) 반경(m). margin 더해 캡슐 반경으로.
        body_height   : 표면 아래 턴테이블 몸체(원판+프레임) 높이(m).
        object_radius/height : disc 위 대상물 근사 원기둥(있으면).
        """
        sp = np.asarray(surface_point, float)
        ax = np.asarray(axis_dir, float); ax = ax / (np.linalg.norm(ax) + 1e-12)
        up = ax if (ax @ (np.asarray(base_origin, float) - sp)) > 0 else -ax
        w = cls()
        w.add_capsule("turntable", sp, sp - up * body_height, disc_radius + margin)
        if object_radius > 0 and object_height > 0:
            w.add_capsule("object", sp, sp + up * object_height, object_radius + margin)
        return w

    # ── 충돌 검사 ────────────────────────────────────────────────────────────
    def check(self, capsules: List[Tuple[str, Capsule]], margin: float = 0.0,
              ignore: Sequence[str] = ()) -> CollisionResult:
        """
        로봇 캡슐들 vs 월드. margin>0 이면 그만큼 여유두고 보수적 판정.

        capsules : [(name, Capsule), …]  — 실제 링크 포즈에서 만든 로봇 캡슐.
        ignore   : 검사 제외할 로봇 캡슐 이름들(예: base 근처 링크 — 오탐 방지).
        """
        contacts = []
        min_clear = np.inf
        for rname, rc in capsules:
            if rname in ignore:
                continue
            for oname, oc in self.obstacles:
                d = seg_seg_distance(rc.p0, rc.p1, oc.p0, oc.p1)
                clear = d - (rc.r + oc.r)
                min_clear = min(min_clear, clear)
                if clear < margin:
                    contacts.append((rname, oname, float(margin - clear)))
            for hname, pt, nrm in self.halfspaces:
                sd = seg_halfspace_min_signed(rc.p0, rc.p1, pt, nrm)
                clear = sd - rc.r
                min_clear = min(min_clear, clear)
                if clear < margin:
                    contacts.append((rname, hname, float(margin - clear)))
        return CollisionResult(collide=len(contacts) > 0,
                               min_clearance=float(min_clear), contacts=contacts)
