# Artec Phase 2 — Bottom-face Scanning + Pose Disambiguation — 2026-05-07

> **목적**: docs/7_artec_phase.md §0 의 water-tight 목표를 위해 Phase 2 (바닥면
> 캡처 + Phase 1 정합) 도입. 그 과정에서 발견된 **face-merging 문제** 와 해결책 분석.

---

## 0. 한 줄 요약

**Phase 2 = 바닥면 캡처 + Phase 1 데이터와 정합**. 객체를 뒤집어 (또는 옆으로 눕혀)
새 IScan 으로 캡처. SDK 의 GlobalRegistration 만으로는 대칭/유사한 아이템에서
**윗면(face1) 과 바닥면(face6) 을 동일면으로 오인 합병** 함. 시작 hint 없는
local-minimum 문제. 해결: **물리 회전 자세를 초기 추정으로 주입** (centroid pivot
기반 pre-rotation hint). 대안: 비대칭 마커 / 로봇 그립.

---

## 1. 문제 관측 — 2026-05-07

3-pass multi-pass 실행:
- Pass 1: 정상 자세, turntable 360° → face1 (윗면) + face2-5 (옆면) 캡처
- Pass 2: 사용자가 아이템 뒤집음, turntable 360° → face6 (바닥면) + face2-5 (옆면, 반대 방향)
- Pass 3: 추가 보강

`GlobalRegistration` 후 mesh 검사 결과:
- **face1 과 face6 가 같은 위치에 합쳐짐** — 윗면과 바닥면이 동일 평면으로 인식
- 옆면 (face2-5) 이 두 IScan 사이에서 정상 정합됐다면 face1-face6 사이엔 ~object_height 만큼의 거리가 있어야 하는데, 그걸 못 잡고 0 으로 무너뜨림

이건 SDK 의 GlobalReg 가 **local minimum 에 빠진 결과**. 두 가능한 정합 (correct flip 180° vs incorrect identity) 의 cost 차이가 크지 않을 때, 아이템 형상이 약간 대칭이면 wrong minimum 의 cost 가 더 낮게 나옴.

---

## 2. 원인 분석

### 2.1 GlobalReg 의 작동 방식

SDK 의 `GlobalRegistrationType_Geometry` 는:
1. 모든 IScan 의 frame transformation 을 바탕으로 글로벌 cost function 정의
2. cost = inter-scan correspondence error (point-to-plane 등)
3. iterative optimization (likely BFGS-class) 으로 minimize

**전제**: 초기 transformation 들이 답에 충분히 가까움 (basin of attraction 안).

Phase 1 의 single-IScan 은 SDK 가 streaming SLAM 으로 frame-by-frame 정합 했으니
초기 transformation 자연스럽게 답 근처. GlobalReg 는 미세조정 역할.

Phase 2 의 second IScan 은 **새 SLAM 시작점에서 출발** — frame transformation 이
"이 IScan 의 첫 frame 기준" 이라 첫 IScan 의 좌표계와 무관. GlobalReg 가 처음부터
두 IScan 사이 transform 을 찾아야 함. **초기 추정 없음 → identity 에서 시작 → wrong
minimum 으로 떨어짐**.

### 2.2 왜 대칭 아이템에서 더 잘 일어나나

아이템이 정확한 cube 라 가정:
- Identity 정합: 두 IScan 의 face1 와 face6 가 겹침. opp side 면들끼리 겹침. cost = 두 면 사이 작은 거리² × N points → 작음.
- Correct 180° 정합: 두 IScan 의 face1 와 face6 가 정반대 위치. 옆면 (face2-5) 이 정확히 일치. cost = 거의 0.

문제: identity 의 cost 가 "충분히 작다" 면 optimizer 가 그 basin 에 머무름.
180° 회전 basin 은 멀리 있어 도달 못 함.

대칭 ↑ → identity cost ↓ → 함정 ↑.

### 2.3 Studio 는 어떻게 해결하나

Artec Studio 의 표준 워크플로우:
- **Auto-alignment 우선 시도**: `createAutoalignAlgorithm` — feature-based 로 정합. 대칭에 약함.
- **Manual Alignment**: 사용자가 두 IScan 에 각각 3개 이상 corresponding point 픽. SDK 가 그걸 시작 transformation 으로 받아 GlobalReg.
- **Rigid registration with hint**: 사용자가 회전/이동 nudge.

우리 코드는 manual alignment 가 없음 → 자동 GlobalReg 만 → wrong minimum.

---

## 3. 해결 후보 (ranked by 가성비)

### 3.1 ⭐ Pre-rotation hint 주입 (즉시 추천)

**아이디어**: Phase 2 (flip 후) 의 IScan 의 **모든 frame transformation 에 알려진 flip
회전을 좌측 곱**해서 GlobalReg 에 넘기기.

```python
# Pass 2 끝나고 master_model 에 추가하기 전:
# 사용자 protocol: "X축 기준 정확히 180° 뒤집음"
T_flip = np.array([
    [1, 0, 0, 0],
    [0,-1, 0, 0],
    [0, 0,-1, 0],
    [0, 0, 0, 1],
], dtype=np.float64)

for f in scan.frames():
    T_old = f.get_transformation()
    f.set_transformation(T_flip @ T_old)
```

GlobalReg 가 거의 답에 도달한 상태에서 시작 → 옳은 minimum 찾음.

**전제**: 사용자가 flip 자세 약속 지킴 (X 축 기준 정확히 180°). 작은 오차는 GlobalReg
가 refine.

**장점**:
- 코드 변경 작음 — frame transformation 행렬 곱 하나
- Hardware 변경 없음
- Symmetric 아이템에도 정확히 작동 (basin 잡아주는 게 핵심)

**단점**:
- 임의 회전엔 약함 — flip axis 와 angle 을 사용자가 commit 해야 함
- N 가지 자세를 원하면 N 가지 hint 필요

**구현 위치**: `mms_artec/nbv/artec_multipass_scan_session.py` 의 `_merge_into_master`
직전. 또는 `ArtecMultiPassScanSessionSettings` 에 `pass_initial_transformations:
list[np.ndarray]` 추가.

### 3.2 비대칭 마커 부착

**아이디어**: 아이템 표면에 작은 sticker / 색점 / 비대칭 stripe 부착. Hybrid registration
(geometry+texture) 가 marker 를 unique correspondence 로 잡아 symmetry 깨뜨림.

**장점**:
- **코드 변경 0**. SDK 의 hybrid 가 자동으로 처리
- Symmetric 아이템에서도 robust
- Pre-rotation hint 와 직교 — 둘 다 쓸 수 있음

**단점**:
- 사용자가 매 아이템에 marker 부착 필요
- Mesh / texture 에 marker 흔적 남음 → 후처리 (mask 또는 inpaint) 필요
- 광택/투명/검정 아이템엔 marker 부착 어려움

### 3.3 Object pose 추적 (사용자 제안 a)

**아이디어**: ChArUco/ArUco 마커판을 아이템에 붙이거나 인접 fixture 에. 별도 카메라
(Femto Bolt 등) 가 매 시점 T_BO 측정. 각 IScan 의 frame transformation 에 baked-in.

**장점**:
- 임의 자세 변경 가능 (단순 flip 외 다양한 view)
- 정확

**단점**:
- 마커 가려지면 끊김 — Phase 1 의 옆면 캡처 중 마커가 turntable 회전으로 안 보일 때
- 마커 자체가 mesh/texture 에 들어감 (3.2 와 동일 문제)
- 별도 카메라 + tracking pipeline 필요 — 큰 인프라

### 3.4 Pose 변환 중 연속 트래킹 (사용자 제안 b)

**아이디어**: Spider 가 캡처 유지하는 상태에서 사용자가 아이템을 천천히 뒤집음.
Streaming SLAM 이 회전을 실시간 따라가니 face1 ~ face6 가 연속된 single IScan 안에서
정합.

**장점**:
- 별도 코드 거의 없음 — 기존 streaming session 그대로
- 한 번에 끝남

**단점**:
- **실측 결과 (docs/7 §15.3)**: 아이템을 손으로 들어 자세 바꿀 때 Studio 도 거의 못
  잡음. 빈 배경에 잘못 정합되거나 손으로 가려짐.
- 숙련도 / 조명 / 배경 / 아이템 형상 의존 큼
- Failure mode 가 silent — tracking lost 안 뜨고 잘못된 mesh 가 만들어짐

→ **가장 신뢰성 낮음**. 마지막 수단.

### 3.5 로봇팔 그립 (장기)

**아이디어**: Gripper 장착. 로봇이 아이템 들고 임의 자세로 회전. T_BO = T_BE @ T_EO
(T_EO 는 그립 시점에 측정) — 매 시점 정확한 pose 자동 계산.

**장점**:
- 가장 자동화. 사용자 개입 0.
- 임의 자세 정확
- 3.4 의 hand-held 시나리오를 robot motion 으로 대체 → 부드럽고 일관됨
- 사용자 (a)+(b) 의 robot 버전

**단점**:
- Gripper 하드웨어 (현재 미보유)
- 그립 플래닝 (안전 그립점 추정)
- 그립 부위는 스캔 안 됨 → 이부위 보강을 위한 re-grip + rescan

---

## 4. 추천 진행 순서

| 단계 | 솔루션 | 작업량 | 효과 |
|---|---|---|---|
| 즉시 | **3.1 Pre-rotation hint** | 1-2 시간 | Phase 2 의 face merging 직접 해결 |
| 백업 | **3.2 비대칭 마커** | 0 코드 + 마커 부착 | 대칭 아이템 robust |
| 장기 | **3.5 Gripper** | 하드웨어 + 큰 코드 | 임의 자세 자동화 |
| 고려 안 함 | 3.4 (연속 hand-flip) | — | 신뢰성 ↓ |

3.1 + 3.2 조합이 현실적인 baseline. 3.5 는 Phase 3 (NBV hole-fill) 와 함께 도입 검토.

---

## 5. 3.1 의 구체 구현 (2026-05-07 완료)

### 5.1 API

`mms_artec/nbv/artec_multipass_scan_session.py`:

```python
@dataclass
class ArtecMultiPassScanSessionSettings:
    ...
    # 사용자가 각 pose 에서 객체에 가한 물리 회전 (4x4). 길이 = pose 수.
    pose_physical_rotations: List[Optional[np.ndarray]] = field(default_factory=list)


def make_axis_physical_rotations(axis: str, angles_deg: List[float]) -> List[np.ndarray]:
    """단일 축 (x/y/z) + 각도 list → 4x4 회전 행렬 list. Right-hand convention."""
```

### 5.2 Pose vs Pass 구분

- **pose_idx** = 사용자가 객체를 둔 자세 (논리 인덱스). hint list 의 index.
- **n_pass** = 실제 IScan 개수 (포함 retry).
- Tracking lost 재시도는 `pose_idx` 유지 (= 같은 hint 재사용).
- 정상 완료 후 사용자 [Enter] 시만 `pose_idx += 1`.

### 5.3 적용 메커니즘 — centroid pivot (2026-05-14 업데이트)

**문제 (초기 버전)**: 단순 `T_new = inv(R_phys) @ T_old` 만 적용하면 회전이 scan
world 원점 (= Spider 카메라 위치) 을 pivot 으로 일어남. 카메라가 객체에서 ~30cm
떨어져 있으니 90° 회전 시 객체 중심이 42cm 이동 — 정합 완전히 깨짐.

**해결**: 객체 centroid 를 pivot 으로 회전. Pass 1 의 centroid 를 master 기준점
`c_master` 로 lock, Pass N 의 centroid `c_pass` 와 매칭:

```python
T_pre = np.eye(4)
T_pre[:3, :3] = inv(R_phys)[:3, :3]
T_pre[:3, 3]  = c_master - inv(R_phys)[:3, :3] @ c_pass

for i in range(scan.frame_count()):
    T_old = scan.get_frame_transformation(i)
    T_new = T_pre @ T_old
    scan.set_frame_transformation(i, T_new)
```

해석: `T_pre(c_pass) = c_master` (객체 중심은 자기 자리), `T_pre(c_pass + p) =
c_master + inv(R_phys) @ p` (회전은 body offset 에만 적용). 사용자가 객체를 살짝
어긋나게 놓아도 centroid 매칭으로 흡수.

**왜 inverse?** 사용자가 Ry(+90°) 로 객체를 회전시키면 Pass N 의 scan 데이터는
Pass 1 좌표계 대비 Ry(+90°) 만큼 회전돼있음. 그걸 Pass 1 좌표계로 가져오려면
Ry(-90°) = inv(Ry(+90°)) 적용.

**Centroid 계산** — `_compute_model_centroid`: scan 의 frame mesh vertices 를
frame_transformation 적용 후 평균. 성능을 위해 scan 당 frame 50개로 subsample.

**GlobalReg auto-skip** — `hints_applied=True` 면 system.py 의 artec_process 가
post-merge GlobalRegistration 자동 skip. Hint 가 authoritative 이므로 GlobalReg 가
다시 흩뜨리는 것 방지.

### 5.4 main_artec.py 의 default 3-pose

```python
POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
# Pose 0: face1 (top) up, canonical
# Pose 1: Ry(+90°) — face5 가 face1 자리로
# Pose 2: Ry(+180°) — face6 (바닥) 이 위로

MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
    pose_physical_rotations=POSE_ROTATIONS,
    max_passes=8,    # 3 pose + retry 여유
)
```

### 5.5 사용자 안내 출력

각 pass 시작 전 다음 pose 안내:
```
[pose 1] 객체 회전 ≈ 90° around axis (+0.00, +1.00, +0.00)
```

### 5.6 검증 체크리스트

- [ ] Hint 없이 (`pose_physical_rotations=[]`) vs 있이 비교
- [ ] 비대칭 머그컵: face1-face6 분리 + 옆면 일관성
- [ ] 정상 자세 → tracking lost → retry: 같은 hint 재사용 확인
- [ ] 90° / 180° 회전 부정확 (±10° 오차) 시 GlobalReg 가 refine 으로 흡수하는지

### 5.7 Failure modes

- 회전축이 다른 경우 → wrong basin. 진단: 로그의 `c_master/c_pass` translation
  값이 비정상이면 의심. 축 부호 (+90° vs -90°) 또는 회전축 (y vs x) 변경 시도.
- Hint matrix 가 invalid (det ≠ ±1, 또는 inverse 불가) → log warning + hint 무시.
- **비대칭 객체의 centroid 이동** — 길쭉한 객체를 옆으로 눕히면 surface vertex
  centroid 가 body 기준 다른 위치로 이동 (예: 세로 cylinder 의 centroid 가
  중심에서 옆면 중앙으로). Centroid = body 중심 가정이 어긋남.
  향후 보완: OBB (oriented bounding box) center 사용 — body-fixed 한 pivot 제공.

---

## 6. 미완 / 향후

- 3.1 의 hint 가 잘못 들어간 경우 detection — GlobalReg 의 final cost 가 비정상적으로
  높으면 hint 의심
- 3.2 의 marker 흔적을 mesh/texture 후처리로 제거 — UV mask 또는 vertex selection
- 3.5 의 gripper plan — Phase 3 도입 시 같이 설계

---

## 7. 관련 문서

- docs/7_artec_phase1.md — Phase 1 streaming + multi-pass 기반 설계
- docs/6_artec_process.md — 전체 SDK 파이프라인 순서
