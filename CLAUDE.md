## World / Robot Frames (Coordinate Frames)

### Notation

- We use **T\_A^B** to denote the homogeneous transform that maps points
  from frame A to frame B:
  
  \[
  T_A^B : \text{frame A} \rightarrow \text{frame B}, \quad x_B = T_A^B \cdot x_A
  \]
  
  - Example: \(x_O = T_B^O(\theta) \cdot x_B\) means we transform a point
    from base frame B to object frame O at angle \(\theta\).

### Coordinate frame naming

| Symbol | Name              | Description                                        |
|--------|-------------------|----------------------------------------------------|
| B      | Base frame        | xArm7 robot base. Global world frame (B ≡ W).     |
| O      | Object frame      | Turntable/object frame. Origin at table center, z-up. |
| E      | End-Effector frame| Robot flange / TCP frame.                          |
| S      | Sensor frame      | Per-sensor camera frame (e.g. S\_Femto, S\_PhoXi). |

> **Important:**  
> In all math, “frame” means **coordinate frame** (B, O, E, S).  
> Sensor data from a camera (one capture) is called a **SensorFrame**
> or **data frame**, to avoid confusion.

---

### 1. Base frame (B)

- We set **B ≡ W** (world frame is the xArm7 base).
- All EE poses, motion generator inputs, and sensor extrinsics
  (hand–eye calibration) are defined w.r.t. B.
- B is the common reference for the whole MMS pipeline.

---

### 2. Object frame (O)

- Object frame O is fixed to the turntable/object:
  - Origin at turntable center.
  - z-axis points up.
  - The object is static in O, even when the turntable rotates.

#### 2.1. B ↔ O transform

- During installation/calibration we measure **T\_B^O(0)** once:

  - \(T_B^O(0)\): base → object transform at turntable angle \(\theta = 0\).
  - Stored in `config/object_frame.yaml`:

    ```yaml
    T_B_O0:                # B → O at theta = 0
      translation: [0.5, 0.0, 0.2]
      rotation_quat: [qx, qy, qz, qw]
    ```

- For a general turntable angle \(\theta\):

  - We define the object→base transform:

    \[
    T_O^B(\theta) = \bigl(T_B^O(0)\bigr)^{-1} \cdot
    \begin{bmatrix}
      R_z(\theta) & 0 \\
      0 & 1
    \end{bmatrix}^{-1}
    \]

  - Then:

    \[
    x_B = T_O^B(\theta) \cdot x_O, \qquad
    x_O = T_B^O(\theta) \cdot x_B
    \]

    with

    \[
    T_B^O(\theta) = \bigl(T_O^B(\theta)\bigr)^{-1}
    \]

- Intuition for \(R_z(\theta)\):

  - \(R_z(\theta)\) represents the rotation of the O-frame z-axis by
    angle \(\theta\) as seen from the base frame B.
  - Points in O (\(x_O\)) are constant as \(\theta\) changes; only their
    representation in B (\(x_B\)) changes with \(\theta\).

#### 2.2. helper

```python
class WorldTransformConfig:
    """
    Static world transform configuration (base <-> object frames).

    Notation
    --------
    T_A_B maps coordinates from frame A to frame B:
        x_B = T_A_B @ x_A
    """

    def __init__(self, T_B_O0: np.ndarray):
        """
        Parameters
        ----------
        T_B_O0 : (4,4) np.ndarray
            Base → Object transform at theta = 0.
        """
        self.T_B_O0 = T_B_O0

    def T_O_B(self, theta: float) -> np.ndarray:
        """
        Object → Base transform at turntable angle theta (rad).

        x_B = T_O_B(theta) @ x_O
        """
        T_O_B0 = np.linalg.inv(self.T_B_O0)
        Rz = rotz(theta)
        T = T_O_B0.copy()
        T[:3, :3] = T_O_B0[:3, :3] @ Rz
        return T

    def T_B_O(self, theta: float) -> np.ndarray:
        """
        Base → Object transform at turntable angle theta (rad).

        x_O = T_B_O(theta) @ x_B
        """
        return np.linalg.inv(self.T_O_B(theta))
```

---

### 3. End-Effector (E) and Sensor (S) frames

- E: robot EE / flange / TCP frame.
- S: sensor frame, defined per device:

  - \(T_E^{S_{\text{Femto}}}\): EE → Femto Bolt sensor.
  - \(T_E^{S_{\text{PhoXi}}}\): EE → PhoXi 3D sensor.

- When NBV computes a **sensor target pose** \(T_B^{S,\text{target}}\),
  we convert to an EE target pose using:

  \[
  T_B^{E,\text{target}} = T_B^{S,\text{target}} \cdot \bigl(T_E^S\bigr)^{-1}
  \]

  so that robot control is always expressed in the EE frame.

> Again: E, S, B, O here are **coordinate frames**.  
> A captured camera **SensorFrame** is a data object that contains
> img, depth, pcd, normals, timestamp, and EE pose in the B frame.