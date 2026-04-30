# mms/control/hardware_layer.py
#
# Lower-layer (3.2 하드웨어 레이어).
#
# 상위(NBV) 레이어가 만든 `T_CO_des` 와 턴테이블 각도 θ 를 받아서
# 실제 로봇 EE 포즈 `T_EB` 와 턴테이블 각도를 하드웨어에 내린다.
#
# Notation (README.md / CLAUDE.md 준수)
# -------------------------------------
#   T_AB : A → B    x_B = T_AB @ x_A
# Frames: B=Base, F=Turntable, O=Internal global, E=End-effector, C=Camera
#
# 체인 (상위 ↔ 하위 사이 유일한 인터페이스)
#   T_CO = T_FO · T_BF(θ) · T_EB · T_CE      (C → O)
#   → T_EB = T_FB(θ) · T_OF · T_CO_des · T_EC

from __future__ import annotations

import time
from typing import Optional, Sequence, TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation as R

from utils.transforms import (
    TurntableTransformConfig,
    solve_T_EB,
)

if TYPE_CHECKING:
    from utils.robot.xarm_interface import XArmInterface
    from utils.turntable.turntable_interface import Turntable


def pose_mat_to_xarm6d(T_EB: np.ndarray) -> np.ndarray:
    """
    T_EB (4,4, translation in meters) → xArm 의 set_position 인자 형식.

    xArm 의 set_position 은 translation=mm, rotation=rad 를 받는다.
    여기서는 scipy xyz-Euler(intrinsic) 로 roll/pitch/yaw 를 추출한다
    (pose_mat_to_6d 와 동일한 규약 — pose6d_to_mat 의 역).

    Returns
    -------
    np.ndarray, shape (6,)
        [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
    """
    assert T_EB.shape == (4, 4)
    t_mm = T_EB[:3, 3] * 1000.0
    rpy = R.from_matrix(T_EB[:3, :3]).as_euler("xyz", degrees=False)
    return np.concatenate([t_mm, rpy])


def execute_camera_target(
    T_CO_des: np.ndarray,
    theta: float,
    T_OF: np.ndarray,
    T_EC: np.ndarray,
    tt: TurntableTransformConfig,
    robot: Optional["XArmInterface"] = None,
    turntable: Optional["Turntable"] = None,
    robot_speed: float = 30.0,
    turntable_vel_rad_s: float = np.pi / 6.0,
    move_turntable: bool = True,
    move_robot: bool = True,
    confirm: bool = True,
) -> dict:
    """
    상위 레이어의 `T_CO_des` + 원하는 턴테이블 각도 θ 를 실제 하드웨어로 내린다.

    절차
    ----
    1. θ (rad) 로 턴테이블 절대 이동 — `turntable.move_abs(theta, vel)` + wait
    2. `T_EB_des = solve_T_EB(theta, T_CO_des, T_OF, T_EC, tt)` 계산
    3. T_EB_des → [x(mm), y(mm), z(mm), roll, pitch, yaw] 로 변환
    4. xArm IK 로 도달 가능 여부 확인 (`robot.ik(...)`)
    5. confirm=True 면 Enter 대기 후 실제 로봇 이동

    Parameters
    ----------
    T_CO_des : (4,4) np.ndarray    NBV 레이어 출력 (C → O)
    theta    : float               턴테이블 목표 각도 (rad, 절대)
    T_OF     : (4,4) np.ndarray    O → F (`MMS._T_OF`)
    T_EC     : (4,4) np.ndarray    E → C (hand-eye)
    tt       : TurntableTransformConfig    B ↔ F 변환 제공자
    robot    : XArmInterface, optional     None 이면 실제 모션 생략
    turntable: Turntable, optional         None 이면 턴테이블 모션 생략
    robot_speed       : float, default=30 (deg/s)
    turntable_vel_rad_s: float, default=π/6 (30°/s)
    move_turntable    : bool, default=True   턴테이블 이동 수행 여부
    move_robot        : bool, default=True   로봇 이동 수행 여부
    confirm           : bool, default=True   실제 이동 전 Enter 확인

    Returns
    -------
    dict
        {
          "T_EB_des"   : (4,4) 목표 E → B
          "pose6d"     : (6,)  [x(mm), y(mm), z(mm), r, p, y(rad)]
          "ik_ok"      : bool                       — IK 해가 존재하는지
          "ik_joints"  : (7,)  [rad] or None        — IK 해 (ik_ok=False 면 None)
          "theta"      : float                      — 명령한 턴테이블 각도 (rad)
          "robot_moved": bool
          "turntable_moved": bool
        }

        IK 실패 / 사용자 abort 시 예외를 던지지 않고 해당 flag 를 False 로
        반환한다. 호출자는 `ik_ok` 를 보고 상위 레이어로 돌아가 다른
        타겟을 고를 수 있다.
    """
    assert T_CO_des.shape == (4, 4)

    T_EB_des = solve_T_EB(theta, T_CO_des, T_OF, T_EC, tt)
    pose6d = pose_mat_to_xarm6d(T_EB_des)

    print("\n" + "═" * 72)
    print("[HW] 하드웨어 레이어 — 카메라 타겟 실행")
    print("═" * 72)
    print(f"  θ (turntable)       : {np.degrees(theta):+.2f}°   ({theta:+.4f} rad)")
    print(f"  T_EB_des translation: [{pose6d[0]:+8.2f}, {pose6d[1]:+8.2f}, "
          f"{pose6d[2]:+8.2f}] mm")
    print(f"  T_EB_des RPY        : [{np.degrees(pose6d[3]):+7.2f}, "
          f"{np.degrees(pose6d[4]):+7.2f}, {np.degrees(pose6d[5]):+7.2f}] deg")

    ik_ok = True
    ik_joints: Optional[np.ndarray] = None
    if robot is not None and move_robot:
        try:
            ik_joints = robot.ik(pose6d, input_is_radian=True)
            print(f"  IK 성공 (joints deg): "
                  f"{np.round(np.degrees(ik_joints), 2).tolist()}")
        except RuntimeError as e:
            print(f"  ✘ IK 실패: {e}")
            print("  → 하드웨어 이동 생략. 다른 타겟을 선택하세요.")
            ik_ok = False
            return {
                "T_EB_des": T_EB_des,
                "pose6d": pose6d,
                "ik_ok": ik_ok,
                "ik_joints": None,
                "theta": theta,
                "robot_moved": False,
                "turntable_moved": False,
            }

    # ── 확인 ───────────────────────────────────────────────────────────
    if confirm:
        ans = input("\n  위 목표로 이동합니다. 계속? (Enter=yes, q=abort): ").strip().lower()
        if ans == "q":
            print("  중단됨.")
            return {
                "T_EB_des": T_EB_des,
                "pose6d": pose6d,
                "ik_ok": ik_ok,
                "ik_joints": ik_joints,
                "theta": theta,
                "robot_moved": False,
                "turntable_moved": False,
            }

    # ── 1) 턴테이블 먼저 (로봇 진입 전에 물체가 회전하도록) ───────────
    turntable_moved = False
    if turntable is not None and move_turntable:
        print(f"\n  [1/2] 턴테이블 → θ={np.degrees(theta):+.2f}°  "
              f"vel={np.degrees(turntable_vel_rad_s):.1f}°/s")
        turntable.move_abs(float(theta), float(turntable_vel_rad_s))
        turntable.wait_motion_done()
        actual_theta = turntable.getActualPos()
        print(f"        실제 θ = {np.degrees(actual_theta):+.2f}° "
              f"(err={np.degrees(actual_theta - theta):+.3f}°)")
        turntable_moved = True
    else:
        print("  [1/2] 턴테이블 이동 생략 (turntable=None or move_turntable=False)")

    # ── 2) 로봇 ────────────────────────────────────────────────────────
    robot_moved = False
    if robot is not None and move_robot:
        print(f"\n  [2/2] 로봇 EE → set_position  speed={robot_speed}°/s")
        robot.enable_motion()
        code = robot.arm.set_position(
            x=float(pose6d[0]), y=float(pose6d[1]), z=float(pose6d[2]),
            roll=float(pose6d[3]), pitch=float(pose6d[4]), yaw=float(pose6d[5]),
            is_radian=True, speed=float(robot_speed), wait=True,
        )
        print(f"        set_position code = {code}")
        if code != 0:
            raise RuntimeError(f"xArm set_position 실패 (code={code})")

        pose_now = robot.get_pose(is_radian=True)
        print(f"        실제 TCP (mm, rad) = "
              f"[{pose_now[0]:+.1f}, {pose_now[1]:+.1f}, {pose_now[2]:+.1f}, "
              f"{pose_now[3]:+.3f}, {pose_now[4]:+.3f}, {pose_now[5]:+.3f}]")
        robot_moved = True
    else:
        print("  [2/2] 로봇 이동 생략 (robot=None or move_robot=False)")

    print("═" * 72)

    return {
        "T_EB_des": T_EB_des,
        "pose6d": pose6d,
        "ik_ok": ik_ok,
        "ik_joints": ik_joints,
        "theta": theta,
        "robot_moved": robot_moved,
        "turntable_moved": turntable_moved,
    }


# ─────────────────────────────────────────────────────────────────────────────
# IK reachability precompute (상위 picker 색상 표시용)
# ─────────────────────────────────────────────────────────────────────────────

def compute_ik_reachability(
    points: np.ndarray,
    normals: np.ndarray,
    distance_m: float,
    theta_target: float,
    T_OF: np.ndarray,
    T_EC: np.ndarray,
    tt: TurntableTransformConfig,
    robot: "XArmInterface",
    roll_candidates: Sequence[float] = (0.0,),
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
    frame_is_O: bool = True,
    check_both_orientations: bool = True,
    verbose: bool = True,
) -> np.ndarray:
    """
    각 점의 로컬 노말 기준 카메라 타겟 포즈를 만든 뒤 xArm IK 로 도달 가능 여부 검사.

    X 프레임(O 또는 B) 기준 `points`, `normals` 에 대해:
        1) p_cam = p + distance_m * n
        2) z_cam = -n, roll 는 roll_candidates 순회
        3) frame_is_O=True  → T_EB = T_FB(θ) · T_OF · T_CX · T_EC
           frame_is_O=False → T_EB = T_CX · T_EC      (X = B)
        4) xArm `get_inverse_kinematics` 호출, code==0 이면 reachable

    하나의 roll 이라도 성공하면 mask[i]=True.

    Parameters
    ----------
    points : (N,3) np.ndarray        X 프레임 기준 좌표 (m)
    normals: (N,3) np.ndarray        X 프레임 기준 단위 노말
    distance_m : float               표면 → 카메라 거리 (m)
    theta_target : float             IK 체크에 쓸 턴테이블 각도 (rad)
    T_OF, T_EC, tt : 하드웨어 상수
    robot : XArmInterface            xArm SDK 호출용
    roll_candidates : tuple of float (rad)   여러 roll 샘플 가능
    frame_is_O : bool, default=True  points 가 O(T) 기준인지
    check_both_orientations : bool, default=True
        True 면 (n, -n) 두 방향 모두 검사 — 사용자가 picker 에서 normal 을
        뒤집더라도 reachable 을 만족하도록. 하나라도 성공하면 True.
    verbose : bool, default=True     진행 로그 출력

    Returns
    -------
    mask : (N,) bool  — True = IK 도달 가능
    """
    from utils.nbv.manual_picker import compute_camera_pose_from_normal

    pts = np.asarray(points, dtype=float)
    nrm = np.asarray(normals, dtype=float)
    if pts.shape != nrm.shape or pts.shape[1] != 3:
        raise ValueError(f"points/normals shape mismatch: {pts.shape} vs {nrm.shape}")

    N = len(pts)
    mask = np.zeros(N, dtype=bool)
    T_FB_theta = tt.T_FB(float(theta_target))
    world_up = np.asarray(world_up, dtype=float)

    normal_signs = (1.0, -1.0) if check_both_orientations else (1.0,)

    if verbose:
        print(f"  [IK] reachability 검사: N={N}  rolls={len(tuple(roll_candidates))}  "
              f"orient={len(normal_signs)}  "
              f"θ_target={np.degrees(theta_target):+.1f}°  frame={'O' if frame_is_O else 'B'}")

    t0 = time.perf_counter()
    n_calls = 0
    last_print = t0
    for i in range(N):
        p = pts[i]
        n = nrm[i]
        if np.linalg.norm(n) < 1e-8:
            continue
        found = False
        for sign in normal_signs:
            if found:
                break
            n_dir = sign * n
            for roll in roll_candidates:
                T_CX = compute_camera_pose_from_normal(
                    surface_point=p, normal=n_dir,
                    distance_m=distance_m, roll_rad=float(roll),
                    world_up=world_up,
                )
                if frame_is_O:
                    T_EB = T_FB_theta @ T_OF @ T_CX @ T_EC
                else:
                    T_EB = T_CX @ T_EC
                pose6d = pose_mat_to_xarm6d(T_EB)
                code, _ = robot.arm.get_inverse_kinematics(
                    pose=pose6d.tolist(),
                    input_is_radian=True,
                    return_is_radian=True,
                )
                n_calls += 1
                if code == 0:
                    mask[i] = True
                    found = True
                    break

        if verbose and (time.perf_counter() - last_print) > 1.0:
            done = i + 1
            pct = 100.0 * done / N
            print(f"    [IK] {done:>5d}/{N}  ({pct:5.1f}%)  reachable={int(mask[:done].sum())}")
            last_print = time.perf_counter()

    elapsed = time.perf_counter() - t0
    if verbose:
        n_ok = int(mask.sum())
        print(f"  [IK] 완료  reachable {n_ok}/{N} ({100*n_ok/max(N,1):.0f}%)  "
              f"calls={n_calls}  elapsed={elapsed:.2f}s  "
              f"({1000*elapsed/max(n_calls,1):.1f} ms/call)")

    return mask
