# tests/test_frame_preprocess.py

import numpy as np
import open3d as o3d
import pytest

from mms.core.frames import Frame


# ---------- helpers ----------

def _make_frame(n_points: int = 200, seed: int = 0) -> Frame:
    """Create a minimal Frame with random pcd in base (B) frame."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-0.5, 0.5, (n_points, 3)).astype(np.float64)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    return Frame(
        sensor_type="orbbec",
        img=None,
        depth=None,
        pcd=pcd,
        normal_map=None,
        frame_id=0,
        timestamp=0.0,
        ee_pose_mat_B=np.eye(4, dtype=np.float64),
    )


def _make_frame_with_normal_map(n_points: int = 200) -> Frame:
    frame = _make_frame(n_points)
    H, W = 4, 4
    frame.normal_map = np.ones((H, W, 3), dtype=np.float32) * 0.577
    return frame


# ---------- __post_init__ ----------

def test_post_init_accepts_float64():
    frame = _make_frame()
    assert frame.ee_pose_mat_B.dtype == np.float64


def test_post_init_upcasts_float32():
    frame = Frame(
        sensor_type="orbbec",
        img=None,
        depth=None,
        pcd=o3d.geometry.PointCloud(),
        normal_map=None,
        frame_id=0,
        timestamp=0.0,
        ee_pose_mat_B=np.eye(4, dtype=np.float32),
    )
    assert frame.ee_pose_mat_B.dtype == np.float64


def test_post_init_rejects_wrong_shape():
    with pytest.raises(ValueError, match="ee_pose_mat_B must be"):
        Frame(
            sensor_type="orbbec",
            img=None,
            depth=None,
            pcd=o3d.geometry.PointCloud(),
            normal_map=None,
            frame_id=0,
            timestamp=0.0,
            ee_pose_mat_B=np.eye(3, dtype=np.float64),
        )


# ---------- ee_pose_6d_B property ----------

def test_ee_pose_6d_B_shape():
    assert _make_frame().ee_pose_6d_B.shape == (6,)


def test_ee_pose_6d_B_identity():
    np.testing.assert_allclose(_make_frame().ee_pose_6d_B, np.zeros(6), atol=1e-10)


def test_ee_pose_6d_B_known_translation():
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [1.0, 2.0, 3.0]
    frame = Frame(
        sensor_type="orbbec",
        img=None,
        depth=None,
        pcd=o3d.geometry.PointCloud(),
        normal_map=None,
        frame_id=0,
        timestamp=0.0,
        ee_pose_mat_B=T,
    )
    np.testing.assert_allclose(frame.ee_pose_6d_B[:3], [1.0, 2.0, 3.0], atol=1e-10)


# ---------- roi_crop ----------

def test_roi_crop_reduces_points():
    frame = _make_frame(n_points=200)
    n_before = len(frame.pcd.points)
    frame.roi_crop((-0.1, 0.1, -0.1, 0.1, -0.1, 0.1))
    assert len(frame.pcd.points) < n_before


def test_roi_crop_keeps_points_inside():
    frame = _make_frame(n_points=500)
    bbox = (-0.2, 0.2, -0.2, 0.2, -0.2, 0.2)
    frame.roi_crop(bbox)
    pts = np.asarray(frame.pcd.points)
    assert np.all(pts[:, 0] >= -0.2) and np.all(pts[:, 0] <= 0.2)
    assert np.all(pts[:, 1] >= -0.2) and np.all(pts[:, 1] <= 0.2)
    assert np.all(pts[:, 2] >= -0.2) and np.all(pts[:, 2] <= 0.2)


def test_roi_crop_does_not_modify_normal_map():
    frame = _make_frame_with_normal_map()
    original = frame.normal_map.copy()
    frame.roi_crop((-0.1, 0.1, -0.1, 0.1, -0.1, 0.1))
    np.testing.assert_array_equal(frame.normal_map, original)


# ---------- denoise ----------

def test_denoise_removes_outliers():
    frame = _make_frame(n_points=200)
    pts = np.asarray(frame.pcd.points).copy()
    pts[:5] = 100.0  # extreme outliers
    frame.pcd.points = o3d.utility.Vector3dVector(pts)
    n_before = len(frame.pcd.points)
    frame.denoise(nb_neighbors=10, std_ratio=1.0)
    assert len(frame.pcd.points) < n_before


def test_denoise_noop_on_empty_pcd():
    frame = _make_frame()
    frame.pcd = o3d.geometry.PointCloud()
    frame.denoise()
    assert len(frame.pcd.points) == 0


def test_denoise_does_not_modify_normal_map():
    frame = _make_frame_with_normal_map()
    original = frame.normal_map.copy()
    frame.denoise()
    np.testing.assert_array_equal(frame.normal_map, original)


# ---------- estimate_normals ----------

def test_estimate_normals_populates_pcd_normals():
    frame = _make_frame(n_points=200)
    frame.estimate_normals()
    assert frame.pcd.has_normals()
    assert len(frame.pcd.normals) == len(frame.pcd.points)


def test_estimate_normals_unit_length():
    frame = _make_frame(n_points=200)
    frame.estimate_normals()
    norms = np.linalg.norm(np.asarray(frame.pcd.normals), axis=1)
    np.testing.assert_allclose(norms, np.ones(len(norms)), atol=1e-6)


def test_estimate_normals_noop_on_empty_pcd():
    frame = _make_frame()
    frame.pcd = o3d.geometry.PointCloud()
    frame.estimate_normals()
    assert not frame.pcd.has_normals()


# ---------- from_orbbec ----------

def _orbbec_inputs(N: int = 50, H: int = 4, W: int = 4, seed: int = 42):
    rng = np.random.default_rng(seed)
    pts_S = rng.uniform(0, 1, (N, 3)).astype(np.float32)
    points_xyzrgb = np.hstack([pts_S, np.ones((N, 3), dtype=np.float32) * 128])
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    normal_map_S = np.tile([0.0, 0.0, 1.0], (H, W, 1)).astype(np.float32)
    return points_xyzrgb, rgb, normal_map_S, pts_S


def test_from_orbbec_sensor_type():
    points_xyzrgb, rgb, _, _ = _orbbec_inputs()
    frame = Frame.from_orbbec(
        rgb=rgb,
        points_xyzrgb=points_xyzrgb,
        ee_pose_mat_B=np.eye(4, dtype=np.float64),
        T_E_S=np.eye(4, dtype=np.float64),
        frame_id=1,
        timestamp=1.0,
    )
    assert frame.sensor_type == "orbbec"
    assert frame.frame_id == 1
    assert frame.timestamp == 1.0
    assert frame.normal_map is None  # normals not provided


def test_from_orbbec_identity_transform_preserves_points():
    """Identity T_E^B and T_E^S → points in B == points in S."""
    points_xyzrgb, rgb, _, pts_S = _orbbec_inputs()
    frame = Frame.from_orbbec(
        rgb=rgb,
        points_xyzrgb=points_xyzrgb,
        ee_pose_mat_B=np.eye(4, dtype=np.float64),
        T_E_S=np.eye(4, dtype=np.float64),
        frame_id=0,
        timestamp=0.0,
    )
    np.testing.assert_allclose(
        np.asarray(frame.pcd.points), pts_S.astype(np.float64), atol=1e-5
    )


def test_from_orbbec_translation_applied():
    """T_E^B = pure translation (1,0,0), T_E^S = identity → points shifted by (1,0,0)."""
    N = 10
    pts_S = np.zeros((N, 3), dtype=np.float32)
    points_xyzrgb = np.hstack([pts_S, np.ones((N, 3), dtype=np.float32) * 128])
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)

    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [1.0, 0.0, 0.0]

    frame = Frame.from_orbbec(
        rgb=rgb,
        points_xyzrgb=points_xyzrgb,
        ee_pose_mat_B=T,
        T_E_S=np.eye(4, dtype=np.float64),
        frame_id=0,
        timestamp=0.0,
    )
    np.testing.assert_allclose(
        np.asarray(frame.pcd.points),
        np.full((N, 3), [1.0, 0.0, 0.0]),
        atol=1e-10,
    )


def test_from_orbbec_normal_map_z_invariant_under_z_rotation():
    """+z normals in S are invariant under a z-axis rotation (passed explicitly)."""
    N = 4
    pts_S = np.zeros((N, 3), dtype=np.float32)
    points_xyzrgb = np.hstack([pts_S, np.zeros((N, 3), dtype=np.float32)])
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    normal_map_S = np.tile([0.0, 0.0, 1.0], (2, 2, 1)).astype(np.float32)

    angle = np.pi / 2
    T_rot = np.eye(4, dtype=np.float64)
    T_rot[:3, :3] = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0, 0, 1],
    ])

    frame = Frame.from_orbbec(
        rgb=rgb,
        points_xyzrgb=points_xyzrgb,
        ee_pose_mat_B=T_rot,
        T_E_S=np.eye(4, dtype=np.float64),
        frame_id=0,
        timestamp=0.0,
        normal_map_S=normal_map_S,
    )
    np.testing.assert_allclose(
        frame.normal_map.reshape(-1, 3),
        np.tile([0.0, 0.0, 1.0], (4, 1)),
        atol=1e-6,
    )


# ---------- from_phoxi ----------

def test_from_phoxi_raises_not_implemented():
    with pytest.raises(NotImplementedError):
        Frame.from_phoxi(
            raw_frame=np.zeros(10, dtype=np.float32),
            ee_pose_mat_B=np.eye(4, dtype=np.float64),
            T_E_S=np.eye(4, dtype=np.float64),
            frame_id=0,
            timestamp=0.0,
        )


