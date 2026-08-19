"""
xArm7 해석적 운동학 — DH FK + 수치 DLS IK (side-effect-free, scipy 비의존).

포즈 규약은 실물 XArmInterface 와 동일:
  pose6d = [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]  — B(베이스) 기준 플랜지
  euler 규약 'xyz' intrinsic (utils.transforms.pose6d_to_mat 의 scipy 'xyz' 와 일치)

Isaac 백엔드에서 fk/ik 및 planner reachability 에 사용한다. 실제 로봇 모션은
sim 아티큘레이션을 관절공간으로 구동하므로(IsaacXArm), 이 모델은 자기일관적이면
충분하다(공칭 xArm7 DH).
"""

from __future__ import annotations

import numpy as np

# ── xArm7 운동학: USD 관절체인에서 추출 (xarm7_spider/v2.usd 와 일치) ─────────
# 기존 공칭 modified-DH 는 이 USD 아티큘레이션과 어긋났다(플랜지 256mm 오차). USD 의
# 각 RevoluteJoint 의 부모쪽 프레임(localPos0, localRot0)을 그대로 써서 체인 FK 를
# 구성한다 (localPos1/Rot1 은 ≈단위라 생략). 이렇게 하면 sim USD 와 ≈3mm 일치하고,
# USD 는 공식 xArm7 모델 기반이므로 실물과도 정합한다.
#   child_frame = parent_frame · M0_i · Rz(q_i)
#   M0_i = Trans(localPos0_i, m) · Rot(localRot0_i)  (quat = [w,x,y,z])
_LOCALPOS0 = np.array([
    [5.960464477539063e-08, 1.792795956134796e-08, 0.2670001983642578],
    [-5.967448757360216e-13, -9.313225746154785e-09, -2.220446049250313e-16],
    [-6.1541620688387866e-09, -0.2930000424385071, 3.4872400522800717e-09],
    [0.05250001326203346, 1.1791971843422289e-09, -1.8532625434275474e-10],
    [0.07750005275011063, -0.3424999415874481, -4.991077062754812e-08],
    [-2.5974761896918608e-08, 1.8604853213588513e-09, -1.4611578613710208e-08],
    [0.07599999010562897, 0.09699998795986176, -7.572470650529795e-09],
])
_LOCALROT0 = np.array([   # [w, x, y, z]
    [1.0, -1.4901161193847656e-08, 1.960015972502728e-21, 3.203575033694506e-05],
    [-0.6841534376144409, 0.6841561198234558, 0.1786961406469345, 0.17869555950164795],
    [0.7071055173873901, 0.7071080803871155, -1.0264685442962218e-05, 1.0228301107417792e-05],
    [0.7071053981781006, 0.7071078419685364, -0.0005101628485135734, 0.0005102516734041274],
    [0.7071054577827454, 0.7071080803871155, -8.92863408807898e-06, 8.961141247709747e-06],
    [0.7070984244346619, 0.7071009874343872, -0.003164870198816061, 0.0031648967415094376],
    [-0.704627275466919, 0.7046297788619995, -0.059149447828531265, -0.05914930999279022],
])
# link_base → /World/xarm7(=base_world_pose) 정렬 (≈단위, 3mm). 미터.
BASE_TO_LINKBASE = np.array([
    [0.9999953891764765, 0.0015140619245947286, 0.00263234539449228, -0.003131270408630371],
    [-0.0015144126222633677, 0.9999988446648069, 0.0001312381835058259, -0.00015828362666070347],
    [-0.002632143650514306, -0.0001352240354813541, 0.9999965267611, 0.0014686584472656254],
    [0.0, 0.0, 0.0, 1.0],
])

# 관절 한계 (rad) — IK 해 클램프용 (공칭 xArm7, USD 와 정합)
JOINT_LOWER = np.array([-2*np.pi, -2.059, -3.927, -1.693, -6.283, -1.693, -6.283])
JOINT_UPPER = np.array([ 2*np.pi,  2.042,  0.192,  3.142,  6.283,  3.142,  6.283])


def _quat_wxyz_to_R(q) -> np.ndarray:
    wq, x, y, z = q
    n = np.sqrt(wq*wq + x*x + y*y + z*z) + 1e-18
    wq, x, y, z = wq/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*wq), 2*(x*z+y*wq)],
        [2*(x*y+z*wq), 1-2*(x*x+z*z), 2*(y*z-x*wq)],
        [2*(x*z-y*wq), 2*(y*z+x*wq), 1-2*(x*x+y*y)],
    ])


# 관절별 고정 변환 M0_i (부모→관절프레임, 미터). 모듈 로드 시 1회 구성.
_M0 = []
for _i in range(7):
    _m = np.eye(4)
    _m[:3, :3] = _quat_wxyz_to_R(_LOCALROT0[_i])
    _m[:3, 3] = _LOCALPOS0[_i]
    _M0.append(_m)


# ── euler 'xyz' intrinsic ↔ 회전행렬 (scipy from_euler('xyz') 와 동일) ───────
def euler_xyz_to_R(rpy: np.ndarray) -> np.ndarray:
    a, b, c = float(rpy[0]), float(rpy[1]), float(rpy[2])
    ca, sa = np.cos(a), np.sin(a)
    cb, sb = np.cos(b), np.sin(b)
    cc, sc = np.cos(c), np.sin(c)
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def R_to_euler_xyz(R: np.ndarray) -> np.ndarray:
    """R = Rx(a)Ry(b)Rz(c) 분해 → [a,b,c]."""
    sb = np.clip(R[0, 2], -1.0, 1.0)
    b = np.arcsin(sb)
    if abs(abs(sb) - 1.0) < 1e-6:        # gimbal lock
        a = np.arctan2(-R[1, 0], R[1, 1])
        c = 0.0
    else:
        a = np.arctan2(-R[1, 2], R[2, 2])
        c = np.arctan2(-R[0, 1], R[0, 0])
    return np.array([a, b, c])


def _Rz(a) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[0, 0] = c; T[0, 1] = -s; T[1, 0] = s; T[1, 1] = c
    return T


def _fk_frames_m(q: np.ndarray):
    """USD 관절체인 FK. base(=base_world_pose) 프레임의 링크 변환들(미터).
    반환 (8,4,4): [link_base, link1, …, link7(=플랜지)]."""
    q = np.asarray(q, dtype=float)
    T = BASE_TO_LINKBASE.copy()
    frames = [T.copy()]
    for i in range(7):
        T = T @ _M0[i] @ _Rz(q[i])          # child = parent · M0_i · Rz(q_i)
        frames.append(T.copy())
    return frames


def fk_T(q: np.ndarray) -> np.ndarray:
    """관절각(rad,7) → 플랜지 동차변환 T_EB (4x4), 병진 **mm** (USD 정합)."""
    T = _fk_frames_m(q)[-1].copy()
    T[:3, 3] *= 1000.0                       # m → mm (pose6d/ik 규약)
    return T


def fk_pose6d(q: np.ndarray) -> np.ndarray:
    """관절각 → pose6d [x,y,z mm, rpy rad]."""
    T = fk_T(q)
    return np.concatenate([T[:3, 3], R_to_euler_xyz(T[:3, :3])])


def fk_link_origins(q: np.ndarray) -> np.ndarray:
    """
    관절각(rad,7) → 각 링크 프레임 원점 (base 기준, **meters**).

    반환 (8,3): [link_base 원점, link1, …, link7(=플랜지)].
    충돌검사에서 이웃 원점들을 캡슐(선분)로 이어 팔 형상을 근사한다.
    USD 관절체인 기반이라 sim 실제 링크 원점과 ≈3mm 일치(공칭 DH 대비 정확).
    """
    return np.asarray([T[:3, 3] for T in _fk_frames_m(q)])   # 미터


def pose6d_to_T_mm(pose6d: np.ndarray) -> np.ndarray:
    """pose6d → T (4x4), 병진 mm."""
    pose6d = np.asarray(pose6d, dtype=float)
    T = np.eye(4)
    T[:3, :3] = euler_xyz_to_R(pose6d[3:6])
    T[:3, 3] = pose6d[:3]
    return T


def pose6d_to_T_m(pose6d: np.ndarray) -> np.ndarray:
    """pose6d → T (4x4), 병진 m (utils.transforms.pose6d_to_mat 와 동일 규약)."""
    T = pose6d_to_T_mm(pose6d)
    T[:3, 3] /= 1000.0
    return T


def _rotvec(R: np.ndarray) -> np.ndarray:
    """회전행렬 → 회전벡터(axis*angle)."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - np.pi) < 1e-6:        # 180° 근처
        # R = 2 v vᵀ - I → 가장 큰 대각 성분에서 축 추출
        k = int(np.argmax(np.diag(R)))
        v = np.sqrt(np.clip((np.diag(R) + 1.0) / 2.0, 0.0, None))
        axis = np.zeros(3); axis[k] = v[k]
        if v[k] > 1e-9:
            axis[(k+1) % 3] = R[k, (k+1) % 3] / (2*v[k])
            axis[(k+2) % 3] = R[k, (k+2) % 3] / (2*v[k])
        axis /= (np.linalg.norm(axis) + 1e-12)
        return axis * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def _pose_error(T_cur: np.ndarray, T_des: np.ndarray) -> np.ndarray:
    """6D 오차 [pos(mm,3); rot(rad,3)] (world 프레임)."""
    e_pos = T_des[:3, 3] - T_cur[:3, 3]
    e_rot = _rotvec(T_des[:3, :3] @ T_cur[:3, :3].T)
    return np.concatenate([e_pos, e_rot])


def _numeric_jacobian(q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """[pos(mm); rot] 의 관절각에 대한 (6x7) 수치 자코비안."""
    T0 = fk_T(q)
    J = np.zeros((6, 7))
    for i in range(7):
        dq = np.zeros(7); dq[i] = eps
        Ti = fk_T(q + dq)
        J[:3, i] = (Ti[:3, 3] - T0[:3, 3]) / eps
        J[3:, i] = _rotvec(Ti[:3, :3] @ T0[:3, :3].T) / eps
    return J


# ─────────────────────────────────────────────────────────────────────────────
# 특이점(singularity) 지표
# ─────────────────────────────────────────────────────────────────────────────
# ⚠ 단위 혼합 주의 — `_numeric_jacobian` 은 위치 행이 **mm/rad**, 회전 행이 rad/rad 다.
#   그대로 SVD 하면 위치 성분이 1000 배 커서 회전 특이점이 묻힌다. 아래 함수들은
#   위치를 m 로 바꾸고, 회전 행에 **특성길이**(characteristic length)를 곱해 두 성분의
#   스케일을 맞춘다. 그래야 σ_min 이 '자세 전체가 얼마나 잘 조건화됐는가'를 뜻한다.
CHAR_LENGTH_M = 0.30       # 팔 규모 대표 길이(회전 1rad ↔ 표면 30cm 이동으로 환산)


def jacobian_scaled(q: np.ndarray, char_len_m: float = CHAR_LENGTH_M) -> np.ndarray:
    """단위 정규화된 (6x7) 자코비안 — 전 행이 **m** 스케일."""
    J = _numeric_jacobian(np.asarray(q, float)).copy()
    J[:3, :] /= 1000.0                      # mm → m
    J[3:, :] *= float(char_len_m)           # rad → m 등가
    return J


def singular_values(q: np.ndarray, char_len_m: float = CHAR_LENGTH_M) -> np.ndarray:
    return np.linalg.svd(jacobian_scaled(q, char_len_m), compute_uv=False)


def manipulability(q: np.ndarray, char_len_m: float = CHAR_LENGTH_M) -> float:
    """Yoshikawa 조작성 w = sqrt(det(J·Jᵀ)) = 특이값들의 곱. 0 이면 특이점."""
    return float(np.prod(singular_values(q, char_len_m)))


def sigma_min(q: np.ndarray, char_len_m: float = CHAR_LENGTH_M) -> float:
    """최소 특이값 — **특이점까지의 거리**. w 보다 해석이 직접적이고 덜 민감하다
    (w 는 6개 곱이라 한 축만 나빠져도 급락하지만 크기 감이 잘 안 온다)."""
    return float(singular_values(q, char_len_m)[-1])


def condition_number(q: np.ndarray, char_len_m: float = CHAR_LENGTH_M) -> float:
    """σ_max/σ_min — 클수록 특정 방향 이동에 관절이 과도하게 움직인다."""
    s = singular_values(q, char_len_m)
    return float(s[0] / max(s[-1], 1e-12))


def ik(pose6d: np.ndarray,
       seed: np.ndarray | None = None,
       pos_weight: float = 1.0,
       rot_weight: float = 200.0,
       damping: float = 1.0,
       max_iter: int = 200,
       pos_tol_mm: float = 0.5,
       rot_tol_rad: float = 0.005):
    """
    수치 DLS IK. pose6d(B 기준 플랜지) → 관절각(rad,7).

    Returns
    -------
    (q, ok) : (np.ndarray(7), bool)
        ok=False 면 수렴 실패(미도달 가능성).

    rot_weight: 회전 오차(rad)를 위치 오차(mm) 스케일에 맞춰 가중(기본 200 → 1rad≈200mm).
    """
    T_des = pose6d_to_T_mm(pose6d)
    q = (np.zeros(7) if seed is None else np.asarray(seed, dtype=float).copy())
    W = np.diag([pos_weight]*3 + [rot_weight]*3)

    for _ in range(max_iter):
        T_cur = fk_T(q)
        e = _pose_error(T_cur, T_des)
        if (np.linalg.norm(e[:3]) < pos_tol_mm
                and np.linalg.norm(e[3:]) < rot_tol_rad):
            return np.clip(q, JOINT_LOWER, JOINT_UPPER), True
        J = _numeric_jacobian(q)
        Jw = W @ J
        ew = W @ e
        # DLS: dq = Jwᵀ (Jw Jwᵀ + λ²I)⁻¹ ew
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + (damping**2) * np.eye(6), ew)
        # 스텝 제한 (안정)
        max_step = 0.2
        mx = np.max(np.abs(dq))
        if mx > max_step:
            dq *= max_step / mx
        q = np.clip(q + dq, JOINT_LOWER, JOINT_UPPER)

    T_cur = fk_T(q)
    e = _pose_error(T_cur, T_des)
    ok = (np.linalg.norm(e[:3]) < pos_tol_mm * 4
          and np.linalg.norm(e[3:]) < rot_tol_rad * 4)
    return q, ok
