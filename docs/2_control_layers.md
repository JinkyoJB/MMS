# 상위 레이어 (NBV) 설계 (2_control_layers)

`docs/0_control_layers.md` / `docs/1_control_layers.md` 의 레이어 구조에서
**상위(NBV / 재구성) 레이어** 에 해당하는 설계 문서.

> 하위(hardware) 레이어는 `1_control_layers.md` 에서 완결. 본 문서는 **위쪽**
> 만 다룬다: mesh 누적 방식, frontier 선정 정책, 다음 6DoF 타겟(`T_CO_des`) 생성.

---

## 1. 목표 & 벤치마킹 대상

Artec Studio / SDK 의 `ScanningProcedure` 파이프라인을 PhoXi + xArm7 + 턴테이블에
이식한다 (`docs/6_artec_process.md`, `docs/artec3_scanning_API.md`,
`docs/artec4_algorithm_API.md` 참조).

Artec 핵심 흐름:

```
Frame k 캡처
  → Reconstruct (IFrameMesh 생성)
  → Register  (ICP/Hybrid/Texture — 이전 프레임들과 정합)
  → Store     (IModel 에 축적)
```

우리 파이프라인도 동일한 계층 구조를 유지하되, 다음 두 가지가 다르다:

| 항목 | Artec | 본 시스템 |
|------|-------|-----------|
| 스캐너 | 손에 들고 이동 | EE 에 rigid 고정 (xArm7) + 턴테이블 |
| 다음 시점 결정 | 사람의 직관 | **NBV 자동화** (frontier + planner cost) |
| 정합 | ICP (geometry) / Hybrid | ICP geometry-only (초기) |

따라서 본 문서의 NBV 알고리즘은 다음을 만족해야 한다:

1. **첫 프레임 = Object frame O 정의** — Artec 의 "project global" 과 동일 역할.
2. **이후 프레임 = ICP 정합 가능 조건 유지** — 이전 프레임과 **충분한 overlap**.
3. **NBV = frontier-driven** — 현재 mesh 의 미스캔 경계를 메꾸는 방향으로 이동.
4. **Hardware cost 최소화** — θ 이동 + 로봇 관절 이동을 고려.

---

## 2. 데이터 계층 (Frame → Scan → Model)

Artec 계층을 그대로 차용한다 (`docs/6_artec_process.md` 부록 A).

```
Frame_k          (한 번의 PhoXi 캡처)
   ↓ reconstruct
IFrameMesh_k     (카메라 프레임 기준 mesh 조각)
   ↓ register (ICP against M_{k-1})
Patch_k in O     (O 프레임에 배치된 mesh 조각)
   ↓ fuse
Model M_k        (지금까지 누적된 전체 mesh, O 프레임)
```

- `Frame_k` 는 이미 `mms/core/frames.py` `Frame` 데이터클래스로 있음.
  (img, depth, points, normals, ee_pose_mat_B).
- `Patch_k` = O 프레임에 배치된 triangle mesh (`open3d.geometry.TriangleMesh`).
- `Model M_k` = 누적 구조. **핵심 설계 결정 (§6)** 필요.

---

## 3. 첫 프레임 (Frame 0) — O 정의

### 3.1 scan_start 고정 + 턴테이블 θ=0 리셋

`docs/1_control_layers.md §6.1` 의 `go_to_scan_start()` 를 **Frame 0 의 기준 자세**
로 고정한다.

- **Frame 0 직전에 턴테이블을 `θ=0` 으로 절대 이동** — `turntable.move_abs(0.0, vel)`
  + `wait_motion_done()`. 이전 세션 잔류값을 정규화하여 O 프레임 정의를 단순하게
  유지한다.
- 이후 `go_to_scan_start()` 로 로봇을 고정 자세로 이동.
  - 홈 TCP xyz 유지 + `SCAN_START_RPY_DEG = (178.5°, -20.5°, 0°)`.
- `T_OF = I` → **O ≡ F at θ=0** 그대로 유지. 이 자세가 **본 시스템의 "initial project view"**.
- 이후 모든 프레임은 O 프레임에서 이 자세 기준으로 등록된다.

### 3.2 턴테이블 상판 위만 crop (**mesh 단계에서** 적용)

`config/calibration/turntable_frame.yaml` 에는 `T_B_F0`(B→F, θ=0) 이 저장돼 있고,
F 의 원점은 턴테이블 회전축 상단 중심이다. `T_OF = I` 이므로 **O 원점 = 턴테이블 축,
O 의 z = 위쪽**.

**Crop 적용 위치**

TSDF 입력에는 crop 을 적용하지 **않는다**. 이유:
- Open3D TSDF 는 이미지 공간 입력(RGBD + intrinsic + extrinsic)을 요구 — O 공간
  crop 을 이미지 공간으로 역투영하는 것은 불필요한 복잡성.
- Crop 은 frontier 추출/surface area 계산에만 영향 → **M_k 추출 이후 mesh 단계**
  에서 마스킹하면 충분.

**Crop 식 (O 프레임 points `p_O` 에 대해, mesh vertex 단에서 필터)**

```
z_O > crop_z_min       (상판 먼지/테이블 엣지 제거, 기본 0.005 m)
x_O² + y_O² < R²       (원통 ROI)
```

**결정값**

- `R = 0.12 m` (Q1, 턴테이블 물리 직경 25 cm+안전율 의 이론 반경)
- `crop_z_min = 0.005 m`
- 모든 프레임의 M_k 에 동일 crop 적용 (Q10)

**구현**

`mms/nbv/tsdf_volume.py` 의 `extract_cropped_mesh(volume, z_min, R)` 메서드에서
`vol.extract_triangle_mesh()` 후 vertex 마스킹 + `remove_vertices_by_mask()` 로
한 번에 처리. `Frame.roi_crop` 은 그대로 두고 별도 유틸로 분리.

### 3.3 초기 mesh M_0 생성

Frame 0 의 crop 된 pcd → mesh. 옵션:

**결정 (Q2)** — **Scalable TSDF** 채택. Frame 0 의 depth + pose 로 TSDF 볼륨을 초기화하고
`extract_triangle_mesh()` 로 M_0 추출. 이후 프레임도 동일 볼륨에 integrate 하면 §6 누적이
자연히 해결됨. Voxel length 는 `0.002 m` (Q5).

---

## 4. Frontier 선정 — mesh boundary edge + connected segments

**결정 (Q3)** — 10×10 그리드 분할은 채택하지 않음. 이유:
- 물리 경계 (하나의 열린 홀 rim) 가 격자 셀 경계를 가로지르며 끊기는 문제.
- Dead cell (빈 셀) 이 많아 연산이 낭비됨.
- Mesh 의 실제 토폴로지 (각 열린 경계 = 한 ring/chain) 와 일치하지 않음.

대신 **boundary edge 를 직접 추출 → connected component 로 묶어 segment 단위로 처리**.

### 4.1 Boundary edge 추출

누적 mesh `M_{k-1}` 에서 "삼각형 하나에만 속한 edge" 를 찾는다. 순수 NumPy 로
구현 가능 (Open3D 에 전용 API 없음):

```python
tris = np.asarray(mesh.triangles)                         # (T, 3)
edges = np.vstack([tris[:, [0,1]], tris[:, [1,2]], tris[:, [2,0]]])
edges = np.sort(edges, axis=1)
pairs, counts = np.unique(edges, axis=0, return_counts=True)
boundary_edges = pairs[counts == 1]                       # (E, 2)
```

`boundary_edges` 에 등장하는 vertex 들이 **frontier vertex pool**.

### 4.2 Connected segment 로 묶기

`boundary_edges` 를 **무방향 그래프** 로 보고 connected component 탐색 (BFS / union-find).

```
segments = []
visited = set()
adj = defaultdict(set)     # vertex → 이웃 boundary vertex 집합
for (u, v) in boundary_edges:
    adj[u].add(v); adj[v].add(u)

for start in adj:
    if start in visited: continue
    seg = bfs(start, adj, visited)     # 하나의 ring/chain 을 이루는 vertex 집합
    segments.append(seg)
```

각 segment = **한 개의 열린 홀(ring)** 또는 **끊어진 mesh 경계 조각(chain)**.

### 4.3 Segment 필터링

너무 작은 segment (노이즈/토폴로지 이상) 는 제외:

- `len(seg) < min_seg_vertices` (기본 `10`)
- segment 총 길이 `< min_seg_length_m` (기본 `0.01 m`)

너무 큰 segment 도 한 대표점으로는 부족하므로 **길이 기반 재분할**:

- segment 총 길이 `> max_seg_length_m` (기본 `0.08 m`) 이면 길이 기준으로 2~N 등분.
- 이 경우 한 segment 가 여러 후보(sub-segment) 로 쪼개짐.

### 4.4 Segment → 후보 `(p_i, n_i)`

각 segment (또는 sub-segment) 마다:

- **대표점** `p_i` = segment vertex 좌표의 **길이 가중 centroid**
  (가까운 vertex 끼리 선분 길이로 가중 평균; 단순 mean 보다 분포 치우침이 적음).
- **평균 normal** `n_i` = segment vertex 의 mesh vertex normal 평균 후 정규화.
- 바깥 방향 강제 — `(p_i − centroid_O) · n_i < 0` 이면 부호 반전.
- **segment 길이** `L_i` — 후속 cost 계산의 δ 가중치(§7) 에 사용.

→ 후보 리스트 `{(p_i, n_i, L_i)}` (전형적으로 1 – 수십 개, grid 의 ≤100 보다 타겟팅 정확).

### 4.5 후보마다 camera 타겟 생성

각 `(p_i, n_i, L_i)` 에 대해:

1. Roll `roll_i*` — **§5 ICP 전략** 에서 결정 (전략 C).
2. `T_CO^{(i)} = compute_camera_pose_from_normal(p_i, n_i, distance_m, roll_i*)`.
3. Planner (docs/1 §4.1) 로 θ_i*, q_des_i, feasible 판정.

---

## 5. Roll 전략 — ICP 친화적

### 5.1 왜 roll 이 중요한가

하위 레이어(`docs/1`) 에선 roll 을 상수(`CAMERA_DEFAULT_ROLL_DEG = 90°`)로 고정했다.
그러나 Artec 스타일 **증분 정합** 에서는 이전 프레임과의 overlap 영역이 새 프레임
안에 들어와야 ICP 가 수렴한다. Roll 이 잘못되면 overlap 이 FoV 가장자리로 밀려 나가
정합 실패 확률이 높아진다.

### 5.2 세 가지 전략

#### (A) Up-vector 연속성

이전 프레임의 `y_cam^{(k-1)}` (OpenCV 규약의 "down") 을 새 프레임의 `y_cam^{(k)}` 과
최대한 일치시킨다.

```
y_prev = (T_C_prev_O)[:3, 1]                   # O 에서 본 이전 y_cam
# 새 카메라 기본 축 (roll=0) 에서의 y_cam_0 = f(n_ij)
# roll* = argmin_roll  ‖R_z_cam(roll) · y_cam_0 − y_prev‖
```

- 장점: 구현 단순. 직관적 (영상의 "위쪽" 이 유지됨).
- 단점: overlap 자체를 직접 최적화하진 않음.

#### (B) Overlap 가상 렌더링

각 roll 후보마다 새 카메라 FoV frustum 을 만들고, 현재 mesh 중 frustum 내부에
들어오는 surface 면적을 계산 → 최대 overlap roll 선택.

- 장점: 가장 직접적.
- 단점: frustum raycast / mesh 교차 비용 큼 (후보 100개 × roll 샘플 N → N×100 렌더링).

#### (C) **이전 카메라 방향 정렬 (추천)**

새 카메라에서 **이전 카메라 중심을 향하는 방향** 을 새 카메라 프레임의 **y 축(아래)**
에 오도록 roll 을 정한다. 이렇게 하면:

- 이전 프레임이 본 영역이 새 프레임의 "위쪽 절반" 에 자연스럽게 들어옴.
- overlap 이 frame 중앙 근처에 배치 → ICP feature matching 용이.
- 렌더링 필요 없음, 벡터 연산 1회.

알고리즘:

```
u_O      = normalize(p_cam_prev − p_cam_new)               # O 프레임, 새→이전 카메라 방향
R_CO_0   = build_camera_rot(n_ij, world_up, roll=0)        # roll=0 기준 축
u_cam_0  = R_CO_0.T @ u_O                                  # 새 카메라 frame 에 투영
roll*    = atan2(u_cam_0[0], u_cam_0[1])                   # x↔y 평면 상의 각도
# 검증: R_CO(roll*) 의 y 축을 O 로 변환했을 때 u_O 와 최대 내적
```

> **결정 (Q4)** — **(C) 를 기본** 으로 채택. (A) 는 fallback (C 의 분모가 작아 수치
> 불안정할 때 — 예: 새·이전 카메라 중심이 광축 축과 거의 평행한 경우). (B) 는 성능
> 필요하면 추후 도입.

### 5.3 첫 프레임 이후 k=1 부터 적용

Frame 0 의 roll 은 여전히 `CAMERA_DEFAULT_ROLL_DEG` (scan_start 고정 자세의 rpy 에서
유도) 을 사용. k ≥ 1 부터는 위 (C) 적용.

---

## 6. Mesh 누적 (Model M_k)

### 6.1 선택지

| 방식 | 저장 구조 | ICP 대상 | 메모리 | 구현 난이도 |
|------|-----------|----------|--------|-------------|
| **ScalableTSDF** (Open3D) | voxel + sdf | extract_mesh 결과 | 중 | 낮음 |
| Raw pcd accumulate → Poisson (배치) | pcd 리스트 | 모든 pcd | 큼 | 중 |
| Mesh patch list + per-patch T | 리스트 | 마지막 M_{k-1} | 중 | 높음 |

### 6.2 추천 — ScalableTSDF

- Frame 0: TSDF 볼륨 생성 + Frame 0 integrate → M_0 추출
- Frame k (k≥1):
  1. Frame k 의 depth+pose (T_CO^{(k)}) 로 **ICP 정합** → refined pose T_CO^{(k)'}
  2. refined pose 로 TSDF 에 integrate → M_k 추출

Open3D:
```python
vol = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=0.002, sdf_trunc=0.006,          # Q5 결정값
    color_type=TSDFVolumeColorType.NoColor,
)
vol.integrate(rgbd, intrinsic, extrinsic)          # extrinsic = T_CO (C → O)
mesh = vol.extract_triangle_mesh()
```

**결정 (Q5)** — `voxel_length = 0.002 m` (2 mm). PhoXi S 정밀도 (0.1–0.3 mm) 대비 넉넉,
메모리 부담 낮음. 필요 시 1 mm 로 낮춰 실험.

**PhoXi intrinsic 저장**

Open3D `TSDFVolume.integrate()` 는 `o3d.camera.PinholeCameraIntrinsic` 객체를
요구하므로, **PhoXi SDK 에서 받은 intrinsic 을 한 번 변환해 재사용**한다.

- `PhoxiClient` 가 `get_intrinsic() → o3d.camera.PinholeCameraIntrinsic` 을 노출
  (필요 시 신규 메서드). SDK `PhoXiControl` 프로필의 CameraMatrix + 해상도로 구성.
- `ScanSession` 초기화 시 한 번 호출 → 멤버 필드로 유지 → 모든 `integrate()` 재사용.

**입력 데이터**

- 전체 frame 을 그대로 integrate (crop 없음, §3.2 결정).
- `rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(...)` —
  color 는 없으면 `np.zeros` 대체 (`color_type = NoColor` 로 설정하면 무시됨).
- `extrinsic = T_OC` (O → C) = `inv(T_CO)` — Open3D 규약은 "world(O) 에서 camera(C)
  로 변환" 을 요구.

### 6.3 ICP 파라미터 & 실패 판정 (triple gate)

**결정 (Q7)** — `max_correspondence_distance = 0.005 m` (5 mm) 로 시작. 초기 몇 스텝에서
fitness 가 안정적 (> 0.8) 이면 `0.003 m` 로 축소해 정합 정밀도 향상.

- Point-to-plane (`registration_icp(..., TransformationEstimationPointToPlane())`)
- **Initial guess** = planner 가 준 `T_CO^{(k)}_nominal` (하드웨어가 실제로 간 포즈)
- 최대 iter 50

**Triple gate — 모두 만족해야 TSDF 에 integrate**

| Gate | 조건 | 이유 |
|------|------|------|
| G1 RMSE | `inlier_rmse < 1 mm` | 정합 오차 상한 |
| G2 Fitness | `fitness > 0.3` | inlier ratio — overlap 부족 검출 |
| G3 Drift | `‖t_refined − t_init‖ < 20 mm` **and** `∠(R_refined, R_init) < 10°` | 로컬 minima 탈주 방지 |

하나라도 실패하면:
- 같은 θ 에서 **1회 재캡처** 후 재시도.
- 두 번째도 실패하면 해당 frontier 후보를 skip, 다음 후보로 (cost 순위 2위) 넘어감.
- cost 후보를 모두 소진하면 스텝 fail → k 증가 없이 종료 조건 체크.

---

## 7. NBV 비용 함수 — O 기준 EE-pose

### 7.1 왜 joint L2 가 아니라 O-pose 인가

ICP / 재구성 관점에서는 **object(O) 좌표계에서 카메라(EE 또는 C) 가 얼마나
움직였는지** 가 직접적인 지표다. 로봇 joint 값은 같은 EE pose 여도 여러 IK branch
로 달라질 수 있고, NBV 품질과 직접 관계가 약하다.

- **NBV 상위 cost** → O 기준 EE pose 변화량 (translation + rotation).
- **Hardware planner 내부 cost** → joint L2 (docs/1 §4.1 그대로).
- 즉 NBV 는 "어느 EE pose 로 갈지" 만 결정하고, 같은 EE pose 안에서의 joint
  세부 최적화는 docs/1 planner 에 위임.

### 7.2 비용 항 정의

현재 카메라 pose 를 `T_CO_cur` (C → O), 후보 `i` 의 카메라 pose 를 `T_CO_i` 라 두고:

```
Δp_i   = (T_CO_i[:3, 3]) − (T_CO_cur[:3, 3])              # O 기준 EE 위치 차이, [m]
ΔR_i   = R_CO_cur⁻¹ · R_CO_i
ϕ_i    = rot_angle(ΔR_i)                                  # 회전각, [rad]
Δθ_i   = θ_i* − θ_cur                                     # 턴테이블 회전, [rad]
L̂_i    = L_i / max_j L_j                                  # frontier segment 길이 정규화, [0,1]
```

### 7.3 최종 cost

```
cost_i = α · ‖Δp_i‖²           # O 기준 translation,  [m²]
       + β · ϕ_i²              # O 기준 orientation,  [rad²]
       + γ · (Δθ_i)²           # 턴테이블,           [rad²]
       − δ · L̂_i               # frontier 우선순위,   [무차원 0~1]
```

세 제곱항은 각각 물리량 단위 안에서 해석 가능하고, frontier 항은 정규화되어
**δ 가 "frontier 우선순위의 최대 기여량"** 이라는 명확한 의미를 가진다.

### 7.4 가중치 (Q6 결정)

| 가중치 | 값 | 의미 |
|--------|-----|------|
| α | `1.0` | O 기준 EE translation 페널티 (per m²) |
| β | `0.5` | EE orientation 페널티 (per rad²) |
| γ | `0.0` | 턴테이블 회전 페널티 — v0 에선 무시. 과회전 관찰되면 0.1–0.3 로 상향 |
| δ | `0.3` | frontier 길이 보상 비중 (정규화된 L̂) |

> `γ = 0` 으로 시작하는 이유 — planner(docs/1)가 이미 joint 이동 최소화로 θ\* 를
> 고르므로, NBV 단에서 또 턴테이블을 패널라이즈하면 같은 축을 두 번 누르게 된다.
> 실기에서 과회전이 관찰되면 그때 올린다.

### 7.5 ICP fallback / feasibility

- Planner 가 infeasible 이면 후보 제외.
- Roll 전략 (C) 에서 degenerate → (A) fallback (§5.2 결정 Q4).
- 후보 전부 infeasible → 스텝 fail, k 증가 없이 종료 조건 체크.

선택: `i* = argmin_i cost_i` → 그 후보로 `plan_and_execute` 호출.

---

## 8. 한 스텝 사이클 (docs/1 의 `control_step` 대체)

```
ScanSession.run()  (의사코드)
├─ go_home()
├─ turntable.move_abs(0.0) + wait            # θ=0 리셋 (§3.1)
├─ go_to_scan_start()
├─ PhoXi intrinsic = phoxi.get_intrinsic() → o3d.PinholeCameraIntrinsic   (§6.2)
├─ vol = ScalableTSDFVolume(voxel_length=0.002, sdf_trunc=0.006)          # Q2, Q5
├─
├─ Frame 0
│   ├─ capture + rgbd_0 = to_rgbd(frame_0)
│   ├─ T_CO_0 = T_CO(θ=0, T_EB_0) via docs/1 compute_T_CO                # identity-ish
│   ├─ vol.integrate(rgbd_0, intrinsic, extrinsic=inv(T_CO_0))           # full, no crop
│   └─ p_cam_prev = T_CO_0[:3, 3]     (§5 전략 C 에 사용)
│
├─ for k = 1 .. K_max (=20):                                              # Q8
│   ├─ M_raw     = vol.extract_triangle_mesh()
│   ├─ M         = crop_mesh_O(M_raw, z_min=0.005, R=0.12)               # §3.2, Q1, Q10
│   ├─
│   ├─ # Frontier 추출 (§4)
│   ├─ boundary_edges = extract_boundary(M)
│   ├─ segments       = connected_components(boundary_edges)
│   ├─ candidates     = filter_and_split(segments,
│   │                        min_seg_vertices=10,
│   │                        min_seg_length=0.01,
│   │                        max_seg_length=0.08)
│   ├─
│   ├─ # 후보 평가
│   ├─ for each (p_i, n_i, L_i) in candidates:
│   │   ├─ # Roll: (C) 기본, degenerate 시 (A) fallback  (Q4)
│   │   ├─ roll_i  = pick_icp_roll(p_i, n_i, p_cam_prev, R_CO_prev,
│   │   │                          fallback_thresh=0.2)
│   │   ├─ T_CO_i  = camera_pose(p_i, n_i, distance_m=0.414, roll_i)
│   │   └─ plan_i  = plan_min_motion_theta(T_CO_i, ...)
│   │        → θ_i*, q_i*, feasible? (infeasible 후보는 제외)
│   ├─
│   ├─ # NBV cost — O 기준 EE pose (§7)                                   # Q6
│   ├─ for each feasible i:
│   │   ├─ Δp_i = T_CO_i[:3,3] − T_CO_cur[:3,3]
│   │   ├─ ϕ_i  = rot_angle(R_CO_cur.T @ R_CO_i)
│   │   ├─ Δθ_i = θ_i* − θ_cur
│   │   └─ cost_i = 1.0·‖Δp‖² + 0.5·ϕ² + 0.0·Δθ² − 0.3·(L_i / max_j L_j)
│   ├─
│   ├─ ranked = sort(candidates, by=cost ASC)
│   ├─
│   ├─ # 실행 + ICP 삼중 gate (§6.3)                                      # Q7
│   ├─ for i* in ranked:
│   │   ├─ plan_and_execute(T_CO_i*, ...)                                 # docs/1
│   │   ├─ capture → rgbd_k + T_CO_k_nominal
│   │   ├─ refined_T = ICP(pcd_k, pcd_from(M),
│   │   │                  init=T_CO_k_nominal,
│   │   │                  max_corr=0.005)
│   │   ├─ if gates_pass(refined_T):  break                               # G1,G2,G3
│   │   ├─ else: 1회 재캡처; 또 실패하면 다음 i 로
│   │   └─ if 모든 후보 소진: 스텝 fail, k 유지
│   ├─
│   ├─ vol.integrate(rgbd_k, intrinsic, extrinsic=inv(refined_T))
│   ├─ p_cam_prev, R_CO_prev = refined_T[:3,3], refined_T[:3,:3]
│   ├─ T_CO_cur = refined_T
│   └─ if 종료조건(§9): break                                             # Q9
│
└─ return final_mesh = crop_mesh_O(vol.extract_triangle_mesh(), ...),
           pose_list  = [T_CO_0, T_CO_1, ...]
```

---

## 9. 종료 조건

다음 중 하나 충족 시 종료:

- **Frontier 없음** — boundary edges 길이 합이 임계 이하 (mesh watertight 근접).
- **모든 후보 infeasible** — planner 가 100 cell 모두에 대해 실패.
- **Iter 상한** — `K_max` (기본 20).
- **Plateau** — 최근 3 스텝에서 mesh surface 면적 증가율 < 1 %.

---

## 10. 파일 레이아웃 (계획)

```
mms/nbv/
├── manual_picker.py          # docs/1 용 (그대로 유지)
├── scan_session.py           # (신규) 본 문서의 메인 오케스트레이터
├── frontier.py               # boundary edge 추출 + connected segment 묶기 (§4)
├── icp_strategy.py           # §5 roll 선정 (C), fallback (A)
└── tsdf_volume.py            # Open3D ScalableTSDF 래퍼 (§6)
```

크롭 유틸은 기존 모듈 확장:

```
mms/core/frames.py            # Frame.roi_cylinder_F(z_min, R) 추가
```

`mms/system.py` 에 `MMS.run_scan_session(...)` 메서드 추가 (내부에서 위 모듈 오케스트레이션).

---

## 11. 설계 결정 요약

| # | 질문 | 최종 결정 |
|---|------|-----------|
| Q1 | 턴테이블 상판 crop 반경 R | `R = 0.12 m` (턴테이블 직경 25 cm 의 이론 반경) |
| Q2 | Mesh 누적 방식 | Scalable TSDF 채택 |
| Q3 | Frontier 후보 생성 방식 | **Grid 제거**. Boundary edge 직접 추출 + connected segment 로 묶어 대표점/평균 normal/길이 산출 |
| Q4 | Roll 전략 | 전략 **(C) 이전 카메라 방향 정렬** 을 기본, `‖u_cam_xy‖/‖u_cam‖ < 0.2` 면 (A) up-vector 연속성으로 fallback |
| Q5 | TSDF `voxel_length` | `0.002 m` |
| Q6 | NBV cost — **O 기준 EE-pose** | `cost = 1.0·‖Δp‖² + 0.5·ϕ² + 0.0·Δθ² − 0.3·L̂`. joint L2 는 NBV 에 미사용 (하위 planner 만 사용) |
| Q7 | ICP gate | `max_corr = 0.005 m` + **triple gate** (RMSE<1mm, fitness>0.3, drift<20mm/10°). 실패 시 1회 재캡처 → 후보 skip |
| Q8 | `K_max` (iter 상한) | `20` |
| Q9 | 종료 조건 중 플래토 기준 | 최근 3 step 평균 mesh surface 증가율 < 1% |
| Q10 | Crop 적용 위치 | **Mesh 단계** (TSDF 입력에는 crop 없음). 모든 M_k 에 동일 필터 적용 |
| NEW | O 프레임 기준 θ | **scan_start 전 turntable.move_abs(0.0)** 으로 리셋. `T_OF = I` 유지 |
| NEW | PhoXi intrinsic | `PhoxiClient.get_intrinsic() → o3d.PinholeCameraIntrinsic` 를 세션 시작 시 한 번 받아 캐시, 모든 `integrate()` 재사용 |
| NEW | 고도화 (BA, overlap) | v0 에서 생략 — 단순 TSDF + greedy NBV 만. 실측 후 필요 시 V2 에서 추가 |

---

## 12. 다음 단계 (구현 순서)

1. **파일 스켈레톤** — §10 대로 `mms/nbv/{scan_session, frontier, icp_strategy, tsdf_volume}.py` 빈 모듈 생성.
2. **PhoXi intrinsic 노출** — `PhoxiClient.get_intrinsic() → o3d.camera.PinholeCameraIntrinsic`. SDK CameraMatrix + 해상도로 한 번 만들어 반환.
3. **TSDF 래퍼** — `tsdf_volume.py`: `ScalableTSDFVolume` 생성 + `integrate_frame(frame, T_CO)` + `extract_cropped_mesh(z_min, R)`. 정적 depth 이미지로 단위 테스트.
4. **Frame 0 파이프라인** — turntable θ=0 리셋 → `go_to_scan_start` → capture → full-frame integrate → `extract_cropped_mesh` 로 M_0. `p_cam_prev`, `R_CO_prev` 기록.
5. **Frontier 추출** — `frontier.py`: boundary edge → connected components → segment filter/split. 단위 테스트: 구멍 뚫린 구 mesh 에서 링 하나 검출되는지.
6. **Roll 전략 (C) + (A) fallback** — `icp_strategy.py`: `pick_icp_roll(p_i, n_i, p_cam_prev, R_CO_prev, fallback_thresh=0.2)`. 단위 테스트: degenerate 케이스 fallback 동작 확인.
7. **NBV cost (§7)** — `scan_session.py`: O-pose 기반 cost 계산 + 후보 정렬. mock planner 로 검증.
8. **ICP + triple gate** — `icp_strategy.py`: point-to-plane + G1/G2/G3 게이트 함수. 인공 변위 주어 수용/기각 테스트.
9. **Session orchestrator** — `scan_session.py` 전체 while 루프, 후보 소진 재시도, 종료 조건.
10. **실기 통합** — `MMS.run_scan_session()` 으로 노출. `K=3~5` 로 짧게 돌려 pose 누적 · mesh 품질 · 소요시간 점검.
11. **V0 튜닝** — α/β/δ, `max_seg_length`, ICP `max_corr` 를 실측 로그 기반으로 조정.
12. **V2 (필요시)** — pose graph + BA, overlap pre-filter (angular penalty).
