## World / Robot Frames (Coordinate Frames)

### Notation

- We use **T_AB** to denote the homogeneous transform that maps points
  from frame A to frame B:

  ```
  T_AB : frame A → frame B,   x_B = T_AB @ x_A
  ```

  - Example: `x_F = T_BF(theta) @ x_B` transforms a point from base
    frame B to turntable frame F at angle θ.
  - Chain rule: `T_AC = T_AB @ T_BC` (middle frame B cancels).

- **Only use `T_AB` style** (two-letter subscript, underscores).  
  Never write `T_A^B`, `^A T_B`, or `T_A^B`.

### Coordinate frame naming

| Symbol | Name                    | Description                                               |
|--------|-------------------------|-----------------------------------------------------------|
| B      | Base frame              | xArm7 robot base. Global world frame (B ≡ W).            |
| F      | Turntable frame         | Origin at turntable rotation axis center, z-axis up.     |
| O      | Object (Internal Global)| First-scan reference frame, Artec-style internal global. |
| E      | End-Effector frame      | Robot flange / TCP frame.                                 |
| C      | Camera frame            | Per-sensor frame (e.g. C_femto, C_phoxi).                |

> **Important:**  
> In all math, "frame" means **coordinate frame** (B, F, O, E, C).  
> A captured camera image+depth+pcd data object is called a **Frame**
> (capital F, the Python dataclass), not a "sensor frame", to avoid confusion.

---

### 1. Base frame (B)

- We set **B ≡ W** (world frame is the xArm7 base).
- All EE poses, motion generator inputs, and sensor extrinsics
  (hand–eye calibration) are defined w.r.t. B.
- B is the common reference for the whole MMS pipeline.

---

### 2. Turntable frame (F)

- Turntable frame F is fixed to the turntable:
  - Origin at turntable rotation axis center.
  - z-axis points up.
  - An object sitting on the turntable is static in F; only F itself
    rotates relative to B as the turntable angle θ changes.

#### 2.1. B ↔ F transform

- During installation/calibration we measure **T_BF0** once:

  - `T_BF0`: B → F transform at turntable angle θ = 0.
  - Stored in `config/calibration/turntable_frame.yaml`:

    ```yaml
    T_B_F0:                # B → F at theta = 0
      translation: [x, y, z]
      rotation_quat: [qx, qy, qz, qw]
      matrix: [[...], ...]
    ```

- For a general turntable angle θ:

  ```
  T_FB(theta) = T_FB0 @ Rz(theta)      # F → B
  T_BF(theta) = inv(T_FB(theta))       # B → F

  x_B = T_FB(theta) @ x_F
  x_F = T_BF(theta) @ x_B
  ```

  where `T_FB0 = inv(T_BF0)` and `Rz(theta)` is the 4×4 z-rotation matrix.

- Intuition for Rz(θ):
  - As the turntable rotates by θ, the F frame rotates by θ around B's z-axis.
  - Points expressed in F are constant; their B-frame coordinates change with θ.

#### 2.2. TurntableTransformConfig helper

```python
class TurntableTransformConfig:
    """
    T_AB maps frame A → frame B:  x_B = T_AB @ x_A
    """

    def __init__(self, T_BF0: np.ndarray):
        """T_BF0: (4,4) B → F transform at theta = 0."""
        self.T_BF0 = T_BF0
        self._T_FB0 = np.linalg.inv(T_BF0)

    def T_FB(self, theta: float) -> np.ndarray:
        """F → B at turntable angle theta (rad).  x_B = T_FB(theta) @ x_F"""
        Rz = rotz(theta)
        T = self._T_FB0.copy()
        T[:3, :3] = self._T_FB0[:3, :3] @ Rz[:3, :3]
        return T

    def T_BF(self, theta: float) -> np.ndarray:
        """B → F at turntable angle theta (rad).  x_F = T_BF(theta) @ x_B"""
        return np.linalg.inv(self.T_FB(theta))
```

---

### 3. End-Effector (E) and Camera (C) frames

- E: robot EE / flange / TCP frame.
- C: camera/sensor frame, defined per device:

  - `T_EC_femto`: E → Femto Bolt camera frame.
  - `T_EC_phoxi`: E → PhoXi 3D camera frame.
  - Stored in `config/calibration/hand_eye_phoxi.yaml` (key: `T_E_C`).

- When NBV computes a **camera target pose** `T_BC_target`,
  convert to EE target pose:

  ```
  T_BE_target = T_BC_target @ inv(T_EC)
  ```

  so that robot control is always expressed in the EE frame.

- Camera → Base transform:

  ```
  T_CB = T_EB @ inv(T_EC)     # x_B = T_CB @ x_C
  ```

> Again: E, C, B, F, O here are **coordinate frames**.  
> A captured camera data object (img, depth, pcd, normals, timestamp,
> ee_pose_mat_B) is the Python **Frame** dataclass.
