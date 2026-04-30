# tests/test_transforms.py

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from utils.transforms import (
    TurntableTransformConfig,
    compute_T_CB,
    compute_T_CO,
    solve_T_EB,
    rotz,
)


# ---------- helpers ----------

def _random_SE3(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    R = Rotation.from_rotvec(rng.uniform(-1, 1, 3)).as_matrix()
    t = rng.uniform(-1, 1, 3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _make_tt(seed: int = 0) -> TurntableTransformConfig:
    return TurntableTransformConfig(_random_SE3(seed))


# ---------- TurntableTransformConfig ----------

def test_T_FB_T_BF_are_inverses():
    tt = _make_tt()
    for theta in [0.0, np.pi / 4, np.pi, -np.pi / 3]:
        T_FB = tt.T_FB(theta)
        T_BF = tt.T_BF(theta)
        np.testing.assert_allclose(T_FB @ T_BF, np.eye(4), atol=1e-10)
        np.testing.assert_allclose(T_BF @ T_FB, np.eye(4), atol=1e-10)


def test_T_FB_at_zero_equals_T_FB0():
    T_BF0 = _random_SE3(1)
    tt = TurntableTransformConfig(T_BF0)
    np.testing.assert_allclose(tt.T_FB(0.0), np.linalg.inv(T_BF0), atol=1e-10)


def test_T_FB_translation_invariant_with_theta():
    """Turntable rotation changes only the rotation part, not the origin."""
    tt = _make_tt(2)
    t0 = tt.T_FB(0.0)[:3, 3]
    for theta in [0.3, 0.9, 2.1]:
        t = tt.T_FB(theta)[:3, 3]
        np.testing.assert_allclose(t, t0, atol=1e-12)


def test_T_FB_point_moves_on_circle():
    """Point at (r, 0, 0) in F traces a circle in B as theta changes."""
    # T_BF0 maps B→F. B-origin in F = (-1,0,0) → F-origin in B = (+1,0,0).
    T_BF0 = np.eye(4)
    T_BF0[:3, 3] = [-1.0, 0.0, 0.0]
    tt = TurntableTransformConfig(T_BF0)

    r = 0.5
    x_F = np.array([r, 0.0, 0.0, 1.0])

    for deg in [0, 45, 90, 180]:
        theta = np.radians(deg)
        x_B = tt.T_FB(theta) @ x_F
        # F-origin at (1,0,0) in B; point at (r,0,0) in F rotates with table.
        expected_x = 1.0 + r * np.cos(theta)
        expected_y = r * np.sin(theta)
        np.testing.assert_allclose(x_B[:2], [expected_x, expected_y], atol=1e-10)


# ---------- compute_T_CO / solve_T_EB round-trip ----------

def test_T_CO_solve_T_EB_round_trip_identity_transforms():
    """With identity transforms, T_CO and solve_T_EB are inverses."""
    tt = TurntableTransformConfig(np.eye(4))
    T_OF = np.eye(4)
    T_EC = np.eye(4)
    T_EB = np.eye(4)

    T_CO = compute_T_CO(0.0, T_EB, T_OF, T_EC, tt)
    T_EB_rec = solve_T_EB(0.0, T_CO, T_OF, T_EC, tt)
    np.testing.assert_allclose(T_EB_rec, T_EB, atol=1e-10)


@pytest.mark.parametrize("seed_tt,seed_OF,seed_EC,seed_EB,theta", [
    (0, 1, 2, 3, 0.0),
    (4, 5, 6, 7, np.pi / 4),
    (8, 9, 10, 11, -np.pi / 3),
    (12, 13, 14, 15, np.pi),
])
def test_T_CO_solve_T_EB_round_trip_random(seed_tt, seed_OF, seed_EC, seed_EB, theta):
    """solve_T_EB(theta, compute_T_CO(theta, T_EB, ...)) == T_EB."""
    tt = _make_tt(seed_tt)
    T_OF = _random_SE3(seed_OF)
    T_EC = _random_SE3(seed_EC)
    T_EB = _random_SE3(seed_EB)

    T_CO = compute_T_CO(theta, T_EB, T_OF, T_EC, tt)
    T_EB_rec = solve_T_EB(theta, T_CO, T_OF, T_EC, tt)

    np.testing.assert_allclose(T_EB_rec, T_EB, atol=1e-10)


def test_T_CO_camera_position_in_O():
    """T_CO[:3,3] gives the camera origin expressed in O."""
    # Simple case: all frames at origin except T_EB shifts camera by [1,0,0] in B
    T_BF0 = np.eye(4)  # F = B at theta=0
    tt = TurntableTransformConfig(T_BF0)
    T_OF = np.eye(4)   # O = F
    T_EC = np.eye(4)   # C = E

    T_EB = np.eye(4)
    T_EB[:3, 3] = [1.0, 0.0, 0.0]  # E (=C) at (1,0,0) in B (=O=F)

    T_CO = compute_T_CO(0.0, T_EB, T_OF, T_EC, tt)
    np.testing.assert_allclose(T_CO[:3, 3], [1.0, 0.0, 0.0], atol=1e-10)


def test_compute_T_CB_consistency():
    """compute_T_CB is consistent with the C-column of compute_T_CO when O=B."""
    T_EB = _random_SE3(42)
    T_EC = _random_SE3(43)

    T_CB = compute_T_CB(T_EB, T_EC)

    # With O=B=F (T_BF0=I, T_OF=I, theta=0), T_CO should equal T_CB
    tt = TurntableTransformConfig(np.eye(4))
    T_OF = np.eye(4)
    T_CO = compute_T_CO(0.0, T_EB, T_OF, T_EC, tt)

    # T_CB: C→B, T_CO: C→O=B — should be the same
    np.testing.assert_allclose(T_CO, T_CB, atol=1e-10)
