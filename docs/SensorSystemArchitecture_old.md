# Sensor / System Architecture

## Overview

MMS separates sensor control from the robot-sensor frame transform logic into two distinct layers.

```
┌─────────────────────────────────────────────────────────┐
│  Sensor Layer  (mms/sensor/)                            │
│                                                         │
│  OrbbecClient  PhoxiClient  ArtecClient                 │
│       └──────────── capture() ────────────────┐         │
│                                               ▼         │
│                                         ScanResult      │
│                         (S frame, mm, raw sensor data)  │
└─────────────────────────────────────────────────────────┘
                              │
                   MMS._scan_to_frame()
                   T_S_B = T_E_B @ inv(T_E_S)
                   pts_B = T_S_B @ (pts_S / 1000)
                              │
┌─────────────────────────────────────────────────────────┐
│  System Layer  (mms/system.py)                          │
│                                               ▼         │
│                                            Frame        │
│                         (B frame, meters, with EE pose) │
└─────────────────────────────────────────────────────────┘
```

---

## Coordinate Frames

| Symbol | Name           | Description                              |
|--------|----------------|------------------------------------------|
| B      | Base frame     | xArm7 robot base. Global world frame.    |
| O      | Object frame   | Turntable center, z-up.                  |
| E      | End-Effector   | Robot flange / TCP frame.                |
| S      | Sensor frame   | Per-device camera frame (Femto, PhoXi, Artec). |

**Notation:** `T_A_B` maps coordinates from frame A to frame B:
```
x_B = T_A_B @ x_A
```

---

## Sensor Layer: `ScanResult`

Each sensor client exposes a single method:

```python
sensor.capture(frame_id, timestamp) -> Optional[ScanResult]
```

`ScanResult` is the raw sensor data contract:

```python
@dataclass
class ScanResult:
    sensor_type: str
    points: np.ndarray          # (N, 3) float32 — S frame, mm
    normals: Optional[np.ndarray]   # (N, 3) float32 — S frame
    triangles: Optional[np.ndarray] # (M, 3) int32  — mesh face indices (Artec)
    img: Optional[np.ndarray]       # (H, W, 3) uint8 RGB
    depth: Optional[np.ndarray]     # (H, W) float32, mm
    colors: Optional[np.ndarray]    # (N, 3) float32 [0, 1]
    frame_id: int
    timestamp: float
```

Sensor clients do **not** know about the robot, EE pose, or hand-eye calibration.
They only handle hardware communication and return raw data in the sensor's own frame.

---

## System Layer: `Frame`

`Frame` is the system-level data contract:

```python
@dataclass
class Frame:
    sensor_type: str
    points: np.ndarray          # (N, 3) float32 — B frame, meters
    normals: Optional[np.ndarray]   # (N, 3) float32 — B frame
    img: Optional[np.ndarray]
    depth: Optional[np.ndarray]     # (H, W) float32, meters
    colors: Optional[np.ndarray]
    frame_id: int
    timestamp: float
    ee_pose_mat_B: np.ndarray   # (4, 4) float64 — T_E_B at capture time
    mesh: Optional[o3d.geometry.TriangleMesh]
```

All downstream processing (ROI crop, voxel downsample, denoise, normals,
mesh reconstruction) operates on `Frame` objects in the base frame.

---

## Transform Pipeline: `MMS._scan_to_frame()`

```
Input:  ScanResult  (pts_S in mm, S frame)
        ee_pose_mat_B  (T_E_B, from robot FK)

Step 1: T_S_B = T_E_B @ inv(T_E_S)
        where T_E_S is the hand-eye calibration result

Step 2: pts_m  = pts_S / 1000          # mm → m
        pts_B  = T_S_B @ pts_m          # S frame → B frame

Step 3: normals_B = T_S_B[:3,:3] @ normals_S  (rotation only)
        depth_m   = depth_mm / 1000

Output: Frame  (pts_B in m, B frame)
```

`T_E_S` is loaded once at `MMS.__init__()` from the sensor frames YAML:

```python
cfg = MMSConfig(
    orbbec=OrbbecConfig(...),
    sensor_frames_yaml="config/sensor_frames.yaml",
    T_E_S_key="T_E_S_femto",
    object_frame_yaml="config/object_frame.yaml",
)
```

---

## Hand-Eye Calibration (`T_E_S`)

`T_E_S` maps the **EE frame (E)** to the **Sensor frame (S)**:

```
x_S = T_E_S @ x_E
```

It is obtained by hand-eye calibration (see `docs/Calibration.md`) and stored in
`config/sensor_frames.yaml`:

```yaml
T_E_S_femto:
  translation: [x, y, z]        # meters
  rotation_quat: [qx, qy, qz, qw]

T_E_S_phoxi:
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]

T_E_S_artec:
  translation: [0.0, 0.0, 0.0]  # placeholder until calibrated
  rotation_quat: [0.0, 0.0, 0.0, 1.0]
```

---

## Object Frame (`T_B_O`, `T_O_B`)

The turntable/object frame O is measured once during installation
and stored in `config/object_frame.yaml`:

```yaml
T_B_O0:               # B → O at turntable angle theta = 0
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]
```

At turntable angle θ:

```
T_O_B(θ) = inv(T_B_O0) @ inv(Rz(θ))    # O → B
T_B_O(θ) = inv(T_O_B(θ))               # B → O
```

Access via `mms.T_O_B(theta)` and `mms.T_B_O(theta)`.

---

## Configuration Example

```python
from mms.system import MMS, MMSConfig
from mms.sensor.orbbec import OrbbecConfig

cfg = MMSConfig(
    orbbec=OrbbecConfig(enable_color=True),
    sensor_frames_yaml="config/sensor_frames.yaml",
    T_E_S_key="T_E_S_femto",
    object_frame_yaml="config/object_frame.yaml",
)

with MMS(cfg) as mms:
    # capture_frames() calls sensor.capture() → _scan_to_frame() internally
    frames = mms.capture_frames(10, ee_pose_fn=robot.get_ee_pose_mat)
    mms.preprocess(frames, roi_bbox=(-0.5, 0.5, -0.5, 0.5, 0.0, 1.0))
    mms.visualize_with_object_frame(frames, theta=0.0)
```

---

## Sensor Module Structure

```
mms/sensor/
├── __init__.py              # exports ScanResult
├── scan_result.py           # ScanResult dataclass
├── orbbec/
│   ├── __init__.py
│   └── orbbec_client.py     # OrbbecClient, OrbbecConfig
├── phoxi/
│   ├── __init__.py
│   ├── phoxi_client.py      # PhoxiClient, PhoxiConfig
│   ├── phoxi_meshing.py     # reconstruct_open3d(points, normals)
│   └── phoxi_instant_meshing.py
└── artec/
    ├── CMakeLists.txt        # pybind11 build for artec_sdk_py.pyd
    ├── artec_binding.cpp     # C++ pybind11 wrapper for Artec Capture SDK
    └── artec_client.py       # ArtecClient, ArtecConfig
```
