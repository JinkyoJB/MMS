"""
mms_artec.backends — robot / turntable / sensor 백엔드 팩토리.

`ArtecMMSConfig.backend` 값으로 실물 하드웨어("real") 또는 Isaac Sim("isaac")
구현을 골라 생성한다.

⚠ 하드웨어/시뮬 의존 모듈은 **반드시 분기 안에서 lazy import** 한다:
  - "real" 경로는 Windows + xArm/Artec SDK 가 있어야 import 가능.
  - "isaac" 경로는 Isaac Sim python(`python.sh`) 에서만 import 가능.
둘을 모듈 최상단에서 import 하면 어느 한쪽 환경에서 import 자체가 실패한다.

실물/시뮬 공용으로, robot/turntable/sensor 는 같은 덕타이핑 인터페이스를
만족한다 (utils/robot/xarm_interface.py, utils/turntable/turntable_interface.py,
mms_artec/sensor/artec_client.py 의 메서드 시그니처).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Tuple

if TYPE_CHECKING:                       # 타입 체크 전용 — 런타임 import 아님
    from mms_artec.system import ArtecMMSConfig


# ─────────────────────────────────────────────────────────────────────────────
# Isaac 백엔드는 프로세스당 하나의 SimulationApp 만 가질 수 있으므로,
# robot/turntable/sensor 가 같은 IsaacWorld 를 공유하도록 캐시한다.
# ─────────────────────────────────────────────────────────────────────────────
_ISAAC_WORLD = None


def get_isaac_world(cfg: "ArtecMMSConfig"):
    """공유 IsaacWorld 싱글톤 반환 (없으면 생성). isaac 백엔드에서만 호출."""
    global _ISAAC_WORLD
    if _ISAAC_WORLD is None:
        from mms_artec.backends.isaac.isaac_world import IsaacWorld
        _ISAAC_WORLD = IsaacWorld(
            usd_path=getattr(cfg, "isaac_usd_path", None),
            headless=getattr(cfg, "isaac_headless", False),
            robot_collisions=getattr(cfg, "isaac_robot_collisions", False),
        )
    return _ISAAC_WORLD


def shutdown_isaac_world():
    """공유 IsaacWorld(SimulationApp) 종료. 프로세스 종료 전 호출해야 깔끔하게
    내려간다(미호출 시 atexit 단계에서 omni 프레임워크가 crash 할 수 있음)."""
    global _ISAAC_WORLD
    if _ISAAC_WORLD is not None:
        try:
            _ISAAC_WORLD.close()
        finally:
            _ISAAC_WORLD = None


def build_hardware(cfg: "ArtecMMSConfig") -> Tuple[object, object]:
    """
    (robot, turntable) 생성.

    real  → XArmInterface + Turntable (연결까지 수행)
    isaac → IsaacXArm + IsaacTurntable (공유 IsaacWorld 기반)
    """
    backend = getattr(cfg, "backend", "real")

    if backend == "isaac":
        world = get_isaac_world(cfg)
        from mms_artec.backends.isaac.isaac_xarm import IsaacXArm
        from mms_artec.backends.isaac.isaac_turntable import IsaacTurntable
        robot = IsaacXArm(world)
        turntable = IsaacTurntable(world)
        return robot, turntable

    if backend == "real":
        from utils.robot.xarm_interface import XArmInterface
        from utils.turntable import Turntable
        import numpy as np

        robot = XArmInterface(cfg.robot_ip)
        tt = Turntable(bd_id=cfg.turntable_bd_id, ip=cfg.turntable_ip,
                       pulses_per_rev=50000)
        tt.connect(comm_type=1)            # 1=UDP
        tt.check_drive_info()
        tt.check_drive_err()
        try:
            tt.stop()                      # 이전 run 잔여 회전 정지
        except Exception:
            pass
        tt.set_servo_on(True)
        tt.set_acceleration(np.radians(180), np.radians(180))
        return robot, tt

    raise ValueError(f"unknown backend {backend!r} — 'real' | 'isaac'")


def build_sensor(cfg: "ArtecMMSConfig") -> object:
    """
    sensor(스캐너 클라이언트) 생성.

    real  → ArtecClient (Artec SDK)
    isaac → IsaacArtecScanner (Isaac 카메라 기반, ArtecClient 인터페이스 호환)
    """
    backend = getattr(cfg, "backend", "real")

    if backend == "isaac":
        world = get_isaac_world(cfg)
        from mms_artec.backends.isaac.isaac_scanner import IsaacArtecScanner
        return IsaacArtecScanner(world, cfg.artec)

    if backend == "real":
        from mms_artec.sensor.artec_client import ArtecClient
        return ArtecClient(cfg.artec)

    raise ValueError(f"unknown backend {backend!r} — 'real' | 'isaac'")
