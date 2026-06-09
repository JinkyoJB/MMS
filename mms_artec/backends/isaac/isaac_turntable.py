"""
IsaacTurntable — 턴테이블 백엔드 (Isaac Sim). [kinematic 직접 회전]

`utils.turntable.turntable_interface.Turntable` 와 덕타이핑 호환. θ(rad) 추적.

원판 회전 방식
--------------
원본 v2.usd 의 RevoluteJoint 는 앵커가 disc 의 authored 위치와 불일치해서, disc 를
dynamic(조인트 드라이브)으로 두면 reset 시 solver 가 disc 를 앵커로 스냅해 프레임에서
~0.3m 떨어뜨린다(검증됨). → IsaacWorld 가 DISC/FRAME/OBJECT 를 모두 **kinematic** 으로
고정(올바른 USD 위치 유지)하고 조인트를 끈다. 여기서는 disc·객체·fixture(구) 를 **디스크
중심을 지나는 world-Z 축으로 직접(kinematic) 회전**시킨다.

→ 결정론적이고 위치가 정확하다(스캔/calibration 의 ground-truth θ 보장). 실물의 모터
   드라이브 대신 sim 에선 명령각 = 실측각.

rider
-----
턴테이블과 함께 도는 prim 집합. 기본 rider = {원판(제자리 회전), 객체(공전)}.
calibration fixture(구) 는 `add_rider(path)` 로 등록 → 이후 회전 시 함께 돈다.
"""

from __future__ import annotations

import numpy as np

from mms_artec.backends.isaac.isaac_world import DISC_PRIM, OBJECT_PRIM

_STEP_DEG = 3.0            # move_abs/연속 회전 1 step 당 각도(보간 → 부드러운 렌더)


class IsaacTurntable:
    """Turntable 호환 백엔드 (Isaac Sim, kinematic 회전 방식). θ 단위 rad."""

    def __init__(self, world):
        self._world = world
        self._theta = 0.0           # 명령 θ (rad, unwrapped — 연속 회전 누적)
        self._target = 0.0
        self._vel = 0.0
        self._spinning = False
        self.is_connected = True

        self._Gf = world._Gf
        self._UsdGeom = world._UsdGeom
        self._Usd = world._Usd
        stage = world.stage

        # 회전 중심 = 원판 world XY (kinematic 이라 USD authored 위치에 고정)
        c, _ = world.prim_world_pose(DISC_PRIM)
        self._cx, self._cy = float(c[0]), float(c[1])

        # 턴테이블과 함께 도는 prim(rider)들.
        #   각 rider = (matrix_op, base_local, parentToWorld, parentToWorld_inv)
        #   원판 = 자기 중심에서 제자리 회전, 객체 = 중심 둘레 공전.
        self._riders = []
        self._add_rider_prim(stage.GetPrimAtPath(DISC_PRIM))
        self._add_rider_prim(stage.GetPrimAtPath(OBJECT_PRIM))

    # ── rider (턴테이블에 '부착'되어 함께 도는 prim) ───────────────────────────
    def _add_rider_prim(self, prim):
        UsdGeom, Usd = self._UsdGeom, self._Usd
        xf = UsdGeom.Xformable(prim)
        base = xf.GetLocalTransformation()
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        p2w = xc.GetLocalToWorldTransform(prim.GetParent())
        op = xf.MakeMatrixXform()
        op.Set(base)
        self._riders.append((op, base, p2w, p2w.GetInverse()))

    def add_rider(self, prim_path: str):
        """
        prim 을 턴테이블에 '부착'(rigid attach) — 이후 회전 시 함께 돈다.
        calibration fixture(구) 처럼 턴테이블 위에 올린 것을 등록할 때 사용.
        prim 은 kinematic 이어야 함(IsaacWorld 가 만든 것이거나 직접 설정).
        """
        prim = self._world.stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            self._add_rider_prim(prim)
            self._co_rotate(self._theta)

    # ── 적용 ────────────────────────────────────────────────────────────────
    def _co_rotate(self, ang_rad: float):
        """모든 rider 를 (cx,cy) 통과 world-Z 로 ang_rad 회전."""
        Gf = self._Gf
        Rz = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(np.degrees(ang_rad))))
        Tp = Gf.Matrix4d().SetTranslate(Gf.Vec3d(self._cx, self._cy, 0.0))
        Tn = Gf.Matrix4d().SetTranslate(Gf.Vec3d(-self._cx, -self._cy, 0.0))
        S = Tn * Rz * Tp
        for op, base, p2w, p2w_inv in self._riders:
            op.Set(base * (p2w * S * p2w_inv))

    # ── 연결/서보 (sim no-op) ───────────────────────────────────────────────
    def connect(self, comm_type: int = 0) -> bool:
        self.is_connected = True
        return True

    def disconnect(self):
        self.is_connected = False

    def reconnect(self, settle_s: float = 0.5) -> bool:
        self.is_connected = True
        return True

    def check_drive_info(self):
        print("[IsaacTurntable] (sim) drive info — kinematic 회전")

    def check_drive_err(self) -> bool:
        return False

    def set_servo_on(self, state: bool = True) -> bool:
        return True

    def set_acceleration(self, accel_rad_s2: float, decel_rad_s2: float = None):
        pass

    def set_accel_time(self, ms: int):
        pass

    def set_decel_time(self, ms: int):
        pass

    def set_jog_accel_time(self, ms: int):
        pass

    # ── 상태 조회 ───────────────────────────────────────────────────────────
    def getActualPos(self) -> float:
        return float(self._theta)

    def getActualVel(self) -> float:
        return float(self._vel)

    def GetAxisStatus(self):
        return {"inMotion": self._spinning or abs(self._theta - self._target) > 1e-4}

    def clearpos(self) -> bool:
        self._theta = 0.0
        self._target = 0.0
        self._co_rotate(0.0)
        return True

    # ── 모션 ────────────────────────────────────────────────────────────────
    def move_abs(self, pos_rad: float, vel_rad_s: float) -> bool:
        """절대 θ 로 회전(부드럽게 보간하며 rider 동반 회전 + sim step)."""
        self._vel = float(vel_rad_s)
        self._spinning = False
        start = self._theta
        end = float(pos_rad)
        n = max(1, int(np.ceil(abs(end - start) / np.radians(_STEP_DEG))))
        for i in range(1, n + 1):
            cur = start + (end - start) * (i / n)
            self._co_rotate(cur)
            self._world.step(render=True)
        self._theta = end
        self._target = end
        return True

    def move_inc(self, pos_rad: float, vel_rad_s: float) -> bool:
        return self.move_abs(self._theta + float(pos_rad), vel_rad_s)

    def move_velocity(self, vel_rad_s: float, direction: int) -> bool:
        """연속 회전 시작 (direction: 0=+, 1=-). 전진은 spin_step()/wait 에서."""
        sign = +1.0 if int(direction) == 0 else -1.0
        self._vel = sign * abs(float(vel_rad_s))
        self._spinning = True
        return True

    def spin_step(self, dt: float = 1.0 / 60.0):
        """move_velocity 연속 회전 시 외부 루프가 호출 — θ 전진 + rider 동반 + step."""
        if not self._spinning:
            return
        self._theta += self._vel * dt
        self._co_rotate(self._theta)
        self._world.step(render=True)

    def wait_motion_done(self, *args, **kwargs):
        # move_abs 가 내부에서 step 까지 수행하므로 추가 동작 불필요.
        return

    def stop(self):
        self._spinning = False
        self._vel = 0.0
        self._target = self._theta

    def emergency_stop(self):
        self.stop()
        return True
