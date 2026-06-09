"""
Pluggable IK provider — 연결되면 xArm SDK IK(zero-gap), 아니면 해석 운동학 fallback.

sim/real 괴리 최소화용. 실물 xArm IK 는 컨트롤러 통신이라 오프라인 불가
(SDK `get_inverse_kinematics` → `arm_cmd.get_ik`, `@xarm_is_connected`). UFACTORY 가
컨트롤러 IK 의 오프라인 재현본을 제공하지 않으므로:
  - 로봇/시뮬레이터 연결 → xArm SDK IK 사용 (실물과 동일 = zero-gap)
  - 미연결 → 주입된 해석 운동학 모듈(`xarm7_kinematics`) 사용 (공칭 DH, USD 정합)

numpy/socket/표준 라이브러리만 의존(해석 운동학은 **주입**) → 어떤 환경에서도 안전.
좌표 규약: pose6d = [x,y,z mm, roll,pitch,yaw rad] (B 기준 플랜지).
"""

from __future__ import annotations

import socket
import sys

import numpy as np


def reachable(ip: str, port: int = 502, timeout: float = 1.0) -> bool:
    """TCP 소켓으로 컨트롤러 도달성 빠르게 확인 (블로킹 연결 회피)."""
    try:
        s = socket.create_connection((ip, port), timeout=timeout)
        s.close()
        return True
    except Exception:
        return False


class RobotIK:
    """
    pose6d → 관절각 IK provider.

    Parameters
    ----------
    kin : module | None
        해석 운동학 모듈(`xarm7_kinematics`). fallback 경로에서 `kin.ik(pose6d, seed)` 사용.
    robot_ip : str
        xArm 컨트롤러 IP.
    use_sdk : bool
        True 면 연결 가능 시 SDK IK 우선.
    xarm_sdk_path : str | None
        xArm-Python-SDK 경로(없으면 이미 import 가능 가정).
    logger : callable
        상태 메시지 출력 (기본 print).
    """

    def __init__(self, kin=None, robot_ip: str = "192.168.1.210", use_sdk: bool = True,
                 xarm_sdk_path: str | None = None, logger=print):
        self.kin = kin
        self.robot_ip = robot_ip
        self.mode = "analytic"
        self.api = None
        if use_sdk and reachable(robot_ip):
            try:
                if xarm_sdk_path and xarm_sdk_path not in sys.path:
                    sys.path.insert(0, xarm_sdk_path)
                from xarm.wrapper import XArmAPI
                self.api = XArmAPI(robot_ip, is_radian=True)
                self.mode = "sdk"
                logger(f"[RobotIK] xArm SDK ({robot_ip}) — zero-gap")
            except Exception as e:
                logger(f"[RobotIK] SDK 연결 실패 → analytic fallback: {e}")
                self.api = None
                self.mode = "analytic"
        if self.mode == "analytic":
            if self.kin is None:
                logger("[RobotIK][ERROR] 해석 운동학(kin) 미주입 — IK 불가")
            else:
                logger("[RobotIK] analytic xarm7_kinematics (로봇 오프라인 fallback)")

    def ik(self, pose6d, seed):
        """pose6d (B 기준 플랜지) → (q rad[7], ok)."""
        if self.mode == "sdk":
            seed_list = list(seed) if seed is not None else None
            try:
                code, ang = self.api.get_inverse_kinematics(
                    list(pose6d), input_is_radian=True, return_is_radian=True,
                    ref_angles=seed_list)
            except TypeError:                       # 구펌웨어: ref_angles 미지원
                code, ang = self.api.get_inverse_kinematics(
                    list(pose6d), input_is_radian=True, return_is_radian=True)
            if code == 0 and ang is not None and len(ang) >= 7:
                return np.array(ang[:7], dtype=float), True
            return None, False
        if self.kin is None:
            return None, False
        q, ok = self.kin.ik(pose6d, seed=seed)
        return q, ok
