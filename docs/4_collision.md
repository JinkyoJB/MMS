# 충돌 검사와 IK

> 로봇을 어떤 자세로 보낼 수 있는가(IK)와 가는 길에 무엇을 치지 않는가(충돌)를 다룬다.
> Phase 1 은 로봇이 고정이라 거의 안 걸리지만, **Phase 2 는 매번 자세를 새로 풀고 매번
> 경로를 검사**하므로 여기가 부실하면 그때 티가 난다.
>
> **도달 불가·충돌 판정이 나오면 하드웨어 한계보다 평가 코드를 먼저 의심한다.** 이 셀에서
> 그렇게 오인했던 사례가 부록에 넷 있다.

---

## 1. 사용법과 설정

검사는 **모든 이동이 지나는 한 곳**에 있다. sim 은 `_drive()`, real 은
`_move_robot_to_q()` 가 `CollisionModel` 을 부르므로 Phase 1·2·3 이 자동으로 덮인다.
호출부에서 따로 할 일은 없다.

```python
from utils.collision import collision_model as cmod
cm = cmod.get_default(self_margin_m=0.020, env_margin_m=0.025)
ok, why      = cm.is_pose_safe(q)          # 자세 하나
ok, why, s   = cm.is_path_safe(q0, q1)     # 경로 (보수적 전진)
```

캐시가 없으면 `get_default` 가 `None` 을 돌려주고 **검사는 통째로 skip 된다.**
`[collision] 모델 로드 실패` 가 뜨면 그 상태로 로봇을 움직이지 말 것.

### 캐시 (필수)

| 파일 | 내용 | 생성 |
|---|---|---|
| `utils/collision/data/xarm7_spider_links.npz` | 링크·툴 표면 점군 (링크 로컬) | `scripts/sim/export_link_meshes.py` |
| `utils/collision/data/cell_env.npz` | 셀 구조물 점군 (**로봇 base 프레임**) | `scripts/sim/export_env_mesh.py` |

base 프레임으로 굽기 때문에 **sim·real 공용**이다. **실물 셀을 개조하면 반드시
재생성한다** — 안 하면 없는 구조물을 피하거나 있는 구조물을 통과한다.

> ⚠ 재생성에는 **Isaac Sim 이 필요하다**(USD 씬을 열어 굽는다). 현장 PC(`mms-env`)에서는
> 못 돌린다. 실물이 CAD 와 달라졌을 때 현장에서 무엇을 하는가는 **§6** 에 있다.

### 바꿀 수 있는 값

| 설정 | 뜻 | 기본 |
|---|---|---|
| `self_margin_m` | 자가충돌 여유 | 0.020 |
| `env_margin_m` | 환경충돌 여유 | 0.025 |
| `min_sigma` | 특이점 임계 σ_min (§4) | 0.05 |
| `sdf_voxel_m` / `link_voxel_m` | 환경·링크 SDF 격자 | 0.008 / 0.006 |
| `sdf_pad_m` / `link_pad_m` | 격자 여백 | 0.15 / 0.12 |
| `max_query_pts` | 링크당 질의점 상한 | 2500 |

sim 은 앞의 둘을 환경변수로도 받는다 — `MMS_SIM_SELF_CLEAR`, `MMS_SIM_ENV_CLEAR`.
Phase 2 의 캡슐 world 치수(`nbv_turntable_radius_mm` 등)는 `3_phase2.md` T2 를 본다.

`min_sigma = 0` 으로 주면 특이점 판정만 끄고 충돌만 본다(평가 스크립트가 그렇게 쓴다).

---

## 2. 어떻게 검사하나

```
로봇  = 링크 로컬 메시 점군을 해석 FK 로 배치
환경  = SDF 거리장 8mm   ← O(1) 조회
자가  = 링크 로컬 SDF 6mm ← 속도의 원천
        clearance → is_pose_safe → is_path_safe
```

**자가충돌도 SDF 로 바꾼 것이 핵심이다.** 프로파일링에서 자세당 114.5ms 중 111ms 가
자가충돌 KD-tree 질의였고 환경 SDF 는 36k점에 6.6ms 였다. 같은 방식을 자가에 적용해
**자세당 5.75ms**(20배)가 됐다.

격자 밖 조회는 `pad` 를 돌려준다. 격자가 bbox+pad 를 덮으므로 밖의 점은 최소 pad 만큼
떨어져 있다는 **참인 하한**이라 거짓 '안전'이 나오지 않는다.

검사 대상 쌍은 툴↔link1~5 와 link6↔link1~3 이다. link6/7 은 툴이 붙은 인접 링크라 항상
닿아 있고, link1/2 는 천장 마운트에 상시 근접하므로 환경 검사에서 뺀다.

### 경로 — 보수적 전진

> 자세 q 의 여유가 s 이고 구간에서 어떤 점도 s 보다 멀리 움직이지 않으면, 그 구간은
> **중간 검사 없이 안전이 보증된다.**

|  | 검사 횟수 | 시간 |
|---|---|---|
| 동일 자세 | 2회 | 11.6ms |
| 관절1 5° | 2회 | 10.6ms |
| home→0 대이동 | 35회 | 243.8ms (고정 123스텝은 707ms) |

빠르면서 **터널링이 원리적으로 불가능**하다. 고정·적응 샘플링은 둘 다 샘플 사이를 못 본다.

### 우회

직선이 막히면 `utils/control/joint_path_planner.py` 의 RRT-Connect 가 우회 경로를 찾고
shortcut 으로 평활화한다. 실측에서 **안전 자세 12개 중 직선이 막힌 쌍이 6개**였고 시험한
3개 모두 우회에 성공했다(1.3~2.3s). "막히면 포기"가 실제로 자세를 절반 가까이 잃고 있었다.

검사한 경로를 그대로 주행하므로 **검사 = 실행**이 일치한다.

---

## 3. IK — sim 과 real 이 같은 함수를 쓴다

솔버는 `utils/robot/xarm7_kinematics.ik`(자체 해석 DLS) 하나다. **xArm SDK IK 는 쓰지
않는다**(2026-06 결정) — 컨트롤러 통신이라 하드웨어 연결이 필요하고 왕복이라 느린데,
계획 단계에서는 후보 수백 개를 오프라인으로 풀어야 한다.

자세 생성도 `utils/robot/view_pose.solve_view_q` 로 통일돼 있다. 그전에는 sim 만 roll
6방향·시드 8개를 쓰고 real 은 각각 1개였다 — **같은 솔버를 쓰면서도 sim 에서 되는 자세가
real 에서 "도달 불가"로 버려졌다.** 카메라 규약(sim=USD, real=OpenCV)은 통일하지 않고
**인자로 넘긴다.** 두 규약은 카메라 로컬 Y축 180° 회전 하나만 다르고(`diag(-1,1,-1)`,
전 el/az 편차 1.8e-12) 원점은 완전히 일치하므로, 규약을 데이터로만 넘기면
**캘리브레이션은 건드릴 필요가 없다.**

roll 을 푸는 것이 왜 중요한지는 부록 T2 를 본다.

---

## 4. 특이점 — 관절각 규칙이 아니라 σ_min

`xarm7_kinematics.sigma_min(q)` 로 자코비안 최소 특이값을 보고 `min_sigma = 0.05` 미만이면
자세를 거부한다. 충돌은 아니지만 **명령하면 안 되는 자세**라 같은 게이트에서 막는다 —
여기 두면 자세 선정·경로 보간·우회 계획이 전부 자동으로 적용받는다.

**단위 정규화가 필수다.** `_numeric_jacobian` 은 위치 행이 mm/rad 라 그대로 SVD 하면 위치가
1000배 커서 회전 특이점이 묻힌다. 위치를 m 로 바꾸고 회전에 특성길이 0.3m 를 곱한다.

무작위 1500자세 분포는 최소 0.0004 / 1% 0.0020 / 5% 0.0076 / 중앙 0.069 다. 0.05 는 하위
약 4%를 자르고 home(0.147)에 3배 여유를 남긴다.

**7축에서는 6축의 손목 특이점(q5=0)이 특이점이 아니다** — 실측 σ_min 0.16 으로 남는 축이
보상한다. 실제로 나쁜 것은 팔꿈치가 상한까지 펴진 q4≈157° 형상이고, 이건 **자코비안을
봐야만** 드러난다. 관절각 규칙은 이 팔에서 틀린 답을 낸다.

---

## 5. 원칙 — 턴테이블이 방위각을 담당한다

로봇이 방위각을 만들려고 물체 주위를 돌면 팔이 크게 감싸 위험해진다. 물체중심을 겨냥해
standoff 0.313m 로 실측한 결과다.

| az | 관절이동 | 자가여유 | 환경여유 | 판정 |
|---|---|---|---|---|
| **0°** | **58°** | **52mm** | 217mm | OK |
| 30° | 95° | 60mm | 137mm | OK |
| −30° | 362° | 12mm | 0mm | 충돌 |
| −60° | 409° | **3mm** | 18mm | 충돌 |
| −90° | 445° | **1mm** | 112mm | 충돌 |
| 180° | 345° | 68mm | 0mm | 충돌 |

16개 조합 중 9개가 충돌로 기각되고, az 를 벌릴수록 관절이동이 8배 늘며 여유가 1mm 까지
떨어진다. 그런데 **방위 커버리지는 턴테이블 전회전이 이미 공짜로 제공한다.**

→ `VIEW_AZIS_DEG` 를 `[0, 30, -30]` 으로 축소했다(`MMS_SIM_VIEW_AZIS`).
**턴테이블 = 방위각, 로봇 = 고도각·거리**가 이 셀의 기본 원칙이다.

---

## 6. ⚠ 실물 셀이 CAD 와 다를 때

> 2026-09-15 현장. **여기서 틀리면 오탐이 아니라 미탐**(칠 수 있는데 안전하다고 함)이라
> 손해가 파손이다. §1 의 "실물 셀을 개조하면 반드시 재생성한다" 를 실제로 어떻게 하는가.

### 6.1 지금 무엇을 믿고 있나 — 검증 결과

```
CAD(STEP) ──step2usd──▶ v3_scene.usd ──export_env_mesh.py──▶ cell_env.npz ──▶ 환경 SDF
                                       (표면 샘플링 5mm       (base 프레임
                                        + base 프레임 변환)     점군 130만)
```

기억대로 **CAD 기반이 맞다.** 다만 CAD 를 직접 읽는 게 아니라 **CAD→USD 씬을 한 번 구운
스냅샷**이고, 굽는 도구가 Isaac Sim 에 묶여 있다는 게 지금 문제의 뿌리다(§6.2).

캐시 2개의 성격이 다르다.

| 파일 | 내용 | 셀이 바뀌면 |
|---|---|---|
| `xarm7_spider_links.npz` | **로봇 자신** — 링크 로컬 점군 + 툴(link7 로컬) | 영향 없음. 툴(스캐너·어댑터)을 바꿔 달 때만 |
| `cell_env.npz` | **셀 구조물** — base 프레임 점군 (`env`, `names`) | **매번 틀어진다** ← 이 문서의 대상 |

실제 파일 내용 (2026-09-15 확인):

```
env    (1301122, 3) float64      범위  X -0.389~0.968 · Y ±0.380 · Z -0.055~1.571
names  ['mesh']  ← 1개
```

> **`names` 가 쓸모없다.** USD 메시 이름이 전부 `mesh` 라 부재 단위 식별이 안 된다.
> "저울만 빼고 다시 굽는다" 같은 조작이 **이름으로는 불가능**하다 — 좌표로 잘라야 한다(§6.4).

### ★ base 프레임은 Z 가 **아래로** 증가한다

팔이 천장에 매달려 있어 base 가 뒤집혀 있다. `world → base` 는 **Y축 180° 회전**
(X→−X, Z→−Z) + 원점 `(0.365, 0, 1.500)`. 패치 좌표를 손으로 적을 때 여기서 제일 많이 틀린다.

`hw_layout.md` §1 의 world 값과 캐시 안의 실제 점을 대조해 확인했다:

| 부재 | world Z | base z | 캐시 안 점 (확인) |
|---|---|---|---|
| 천장 마운트 하면 | 1.500 | **−0.055** | 축 근처 13,314점 |
| 턴테이블 상면 | 0.665 | **0.835** | r<0.13 에 31,395점 |
| 상판 `universal_plate` 상면 | 0.510 | **0.990** | 62,574점 |
| 바닥(캐스터 하단) | −0.071 | **1.571** | 1,040점 |

환산: `base_z = 1.500 − world_z`, `base_x = 0.365 − world_x`, `base_y = −world_y`.

### 6.2 왜 현장에서 못 고치나

| # | 막히는 이유 |
|---|---|
| 1 | **재생성에 Isaac Sim 이 필요하다.** `export_env_mesh.py` 가 `SimulationApp` 을 띄우고 USD stage 를 연다. 현장 PC 엔 `mms-env` 뿐이고 `env_isaacsim` 은 RTX GPU + 수십 GB다 (`install.md` §나머지 env) |
| 2 | **CAD 를 먼저 고쳐야 한다.** 실측 → CAD → STEP → USD → npz 는 반나절 루프고, 그날 케이블 트레이 하나 옮기면 처음부터 |
| 3 | **margin 으로 때울 수 없다.** margin 은 *있는* 장애물과의 거리에만 작용한다. 모델에 **없는** 구조물은 거리 계산에 아예 안 들어가므로 `env_margin_m` 을 100mm 로 올려도 그대로 친다 |

3번이 핵심이다. **CAD 와 실물의 차이는 "여유 부족"이 아니라 "데이터 없음"이다.**

### 6.3 순서 — 위험도 순

#### 0단계. 얼마나 틀렸는지부터 잰다 (먼저, 반드시)

**anchor 부터.** 캐시 전체가 로봇 base 에 앵커돼 있으므로, 프레임이나 로봇 마운트가
움직였으면 **개별 부재가 아니라 130만 점이 통째로 어긋난다.** 이걸 모르고 부재 하나씩
패치하면 영원히 안 맞는다.

CAD 씬을 눈으로 먼저 훑어 어디가 다른지 감을 잡는 게 빠르다 (설치는 `install.md` §5):

```powershell
$ISAAC = "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe"
& $ISAAC scripts\sim\view_scene.py v3      # 현재 기준 씬 — 메시 152개
& $ISAAC scripts\sim\view_scene.py v4      # 턴테이블 이설 검토안 (hw_layout §3)
```

```powershell
python scripts/artec/calibrate.py --only 3  # T_B_F0 재측정 — base 기준 턴테이블 축
```

나온 `T_B_F0` 의 원점을 위 표의 `(0, 0, 0.835)` 와 비교한다.
어긋나면 **거기서 멈추고** 원인(프레임 이동? 로봇 remount? 턴테이블 이설?)을 먼저 찾는다.
어차피 `turntable_frame.yaml` 은 Artec 장착 이전 값이라 재캘리브 1순위다(README §알려진 한계 1).

anchor 가 맞으면 개별 부재로 간다. 부재 위치 실측은 로봇 TCP 를 특징점 옆으로 조그해
읽는 게 가장 정확하다:

```powershell
python scripts/robot/jog.py --dz 20 --dry-run   # 접근
python scripts/robot/status.py                  # TCP 읽기
```

> ⚠ `status.py` 의 TCP 는 **플랜지 기준**(TCP offset 0)이지 툴 끝이 아니다.
> 툴 체인 265mm 를 빼고 읽는다(`hw_layout.md` §1 EE 툴 체인 표).

#### 1단계. 실측 프리미티브 패치 ← **현장에서 할 수 있는 유일한 방법**

바뀐 구조물을 박스·원기둥·평면으로 근사해 yaml 에 적고, **로드할 때 점군으로 샘플링해
`env` 에 합친다.**

**왜 이게 되나** — 환경 SDF 는 점군만 받는다(`_Sdf.__init__(pts, voxel, pad)`).
점의 출처가 CAD 표면 샘플링인지 줄자인지 **구분하지 않는다.** 박스 하나를 5mm 간격으로
샘플링하면 CAD 에서 구운 점과 같은 자격의 장애물이 된다.

**근사는 반드시 실물보다 크게(외접) 잡는다.** 과대추정 = 오탐 = 자세 몇 개 손해,
과소추정 = 미탐 = 파손. 비대칭이므로 고민할 것이 없다.

필요한 것 — **아직 없다. 만들어야 한다:**

| 만들 것 | 내용 |
|---|---|
| `config/cell_env_patch.yaml` | 프리미티브 목록. **base 프레임 m, Z 아래로 +** |
| `utils/collision/env_patch.py` | yaml → 점군. 박스/원기둥/평면 표면 샘플링, `spacing_m=0.005` (SDF 복셀 8mm 보다 촘촘해야 한다 — `mesh_sampling.py` 주석과 같은 이유) |
| 배선 2곳 | `collision_model.CollisionModel.__init__` 의 `env = np.load(env_npz)["env"]` 직후, 그리고 `env_collision.EnvCollision.__init__` 의 `self.env = ...` 직후에 `np.vstack`. **두 곳 다** — 같은 npz 를 서로 독립적으로 읽는다 |
| `scripts/collision/plot_env_slice.py` (가칭) | base 프레임 z 단면을 떠서 눈으로 확인. 패치가 엉뚱한 데 붙는 사고를 막는 **가장 싼 방법** |

yaml 예시:

```yaml
# config/cell_env_patch.yaml
# 2026-09-15 현장 실측. CAD(v3_scene.usd) 와 실물의 차이만 적는다.
# ⚠ world 가 아니라 **로봇 base 프레임** (m). Z 는 아래로 증가 — §6.1
add:
  - {name: cable_tray, type: box,      lo: [0.20, -0.38, 0.95], hi: [0.55, -0.30, 1.02]}
  - {name: new_post,   type: cylinder, center_xy: [0.62, 0.21], z0: 0.10, z1: 1.55, radius: 0.045}
remove: []        # §6.4 — 급하지 않으면 비워 둔다
```

#### 2단계. CAD/USD 동기화 (사무실에서, 수렴 경로)

패치가 쌓이면 그걸 **작업지시서 삼아** CAD 를 고치고 `v3_scene.usd` 재생성 →
`export_env_mesh.py` → npz 교체 → yaml 의 해당 항목 삭제.

yaml 을 "아직 CAD 에 반영 안 된 차이 목록"으로 운용하면 **무엇이 밀려 있는지가 파일 하나에
보인다.** 비어 있으면 CAD 와 실물이 같다는 뜻이다.

### 6.4 ⚠ **없어진** 구조물은 빼기가 훨씬 어렵다

추가는 `vstack` 한 줄이지만 제거는 점군에서 영역을 잘라내는 일이다. 두 가지가 걸린다.

- bbox 로 자르면 그 안에 있던 **다른** 부재까지 사라진다 — 벽과 저울처럼 붙어 있으면 같이 날아간다
- `names` 가 `['mesh']` 하나라 **부재 단위 제외가 원리적으로 안 된다**(§6.1)

그래서 안전한 순서는 **"일단 두고, 자세가 실제로 부족할 때만 잘라낸다"** 이다.
없는 구조물을 남겨두면 손해는 오탐(자세 몇 개)이고, 그건 `is_pose_safe` 가 거부한
사유를 보면 바로 드러난다 — `env(link4,12mm)` 처럼 **어느 링크가 무엇에 걸렸는지** 나온다.

### 6.5 패치가 맞는지 확인하는 법

1. **단면 플롯으로 눈 확인** — 좌표 부호 하나 틀리면 구조물이 반대편에 생긴다. Z 뒤집힘(§6.1)이 제일 흔하다
2. **clearance 대조** — 로봇을 몇 자세에 세우고 `cm.clearance(q)` 가 주는 거리를 **자로 잰 값**과 비교. 부호가 맞고 오차가 margin(자가 20 / 환경 25mm) 안이면 쓸 만하다

   ```python
   from utils.collision import collision_model as cmod
   cm = cmod.get_default(min_sigma=0.0)        # 특이점 판정 끄고 거리만
   s, e, who = cm.clearance(q)                 # (자가 m, 환경 m, 최근접 이름)
   ```

   > ⚠ `q` 는 **명령각**을 쓴다. 실측각을 쓰면 T5 의 `start` 거부 연쇄를 그대로 재현한다.
3. **A/B 로 본다**(T8) — 패치 on/off 를 env 스위치로. "고쳤더니 좋아졌다"를 한 번의 실행으로 판단하지 않는다

### 6.6 하지 말 것

| 금지 | 이유 |
|---|---|
| margin 을 올려서 때우기 | 미탐에 안 듣는다 (§6.2-3) |
| npz 를 지우거나 못 읽는 채로 운전 | `get_default()` 가 `None` 을 돌려주고 **검사가 통째로 skip 된다**(§1). 현장에서 제일 위험한 선택 |
| 캡슐 world 만 고치고 메시 env 를 안 고치기 | 캡슐(`robot_collision.CollisionWorld`)은 Phase 2 후보 **사전 필터**일 뿐이고 최종 게이트는 메시다(T4). 순서는 캡슐 scene → 메시 env → 메시 self |
| world 좌표로 yaml 적기 | base 프레임이다. Z 부호가 반대다 (§6.1) |

### 6.7 한편, 오늘 당장 되는 것 — 턴테이블 쪽은 이미 파라미터다

셀 구조물과 달리 **턴테이블 주변은 CAD 에 묶여 있지 않다.** 실물 파이프라인의 캡슐
world 는 `T_B_F0`(캘리브 결과) + 설정값으로 매번 새로 만든다
(`artec_multipass_scan_session._build_collision_world`).

| 설정 | 뜻 |
|---|---|
| `nbv_turntable_radius_mm` | disc(+프레임) 반경 |
| `nbv_turntable_body_height_mm` | 턴테이블 몸체 높이 |
| `nbv_keepout_radius_mm` / `_height_mm` / `_center_xy` | disc 위 금지 원기둥 |
| `nbv_collision_margin_mm` | 캡슐 여유 |

턴테이블을 교체·이설했다면 **재캘리브 + 이 값만 고치면 되고 npz 는 건드릴 필요가 없다.**
`CollisionWorld` 는 `add_box` / `add_cylinder` / `add_capsule` / `add_halfspace` 를 이미
갖고 있어, 사전 필터 층에는 실측 프리미티브를 **코드 수정 없이 오늘 넣을 수 있다.**
다만 그건 사전 필터일 뿐이므로 §6.3 의 메시 패치를 대체하지 않는다.

---

## 7. 코드 지도

```
utils/collision/collision_model.py   ★ 단일 게이트 (clearance/is_pose_safe/is_path_safe)
utils/collision/mesh_sampling.py     # 삼각형 면적비례 표면 샘플링 (T3)
utils/collision/robot_collision.py   # 캡슐 world + swept_pose_collision (Phase 2 후보 필터)
utils/collision/data/*.npz           # 링크·셀 점군 캐시 (base 프레임, sim·real 공용)
utils/collision/env_patch.py         # 〔미구현 §6.3〕 실측 프리미티브 yaml → 점군
config/cell_env_patch.yaml           # 〔미구현 §6.3〕 CAD 와 실물의 차이 목록
utils/control/joint_path_planner.py  # RRT-Connect 우회 + shortcut
utils/robot/view_pose.py             # solve_view_q — roll·시드·규약을 공용화
utils/robot/xarm7_kinematics.py      # 해석 IK/FK, sigma_min

scripts/sim/export_link_meshes.py    # 링크 캐시 생성
scripts/sim/export_env_mesh.py       # 셀 캐시 생성
scripts/sim/eval_collision_fpfn.py   # 캡슐 vs 메시 오탐/미탐 비교
scripts/sim/eval_azimuth_cost.py     # §5 방위각 실측표 재생성
scripts/sim/eval_sigma_min.py        # σ_min 분포
scripts/sim/view_scene.py            # v2/v3/v4 씬을 GUI 로 열어 본다 (§6.3)
scripts/collision/plot_env_slice.py  # 〔미구현 §6.3〕 base 프레임 z 단면 확인
```

---

# 〔부록〕 Troubleshooting

### T1. IK 시드를 하나만 쓰면 갈 수 있는 자세를 버린다
수치해법(DLS)이라 출발점에 따라 수렴이 갈린다. **실측: 수직 파지가 시드 1개면 0/8,
12개면 8/8.** 증상은 산발적 실패다.

### T2. 특정 고도각 대역이 통째로 실패하면 roll 고정을 의심한다
스캐너는 광축 둘레로 돌려도 같은 면을 본다. 그런데 `look_at(eye, target, up=(0,0,1))` 은
roll 을 하나로 못 박는다. **실측: roll 고정 시 el 30° 자세가 0/8, 풀면 5/8. 시드를 12개로
늘려도 roll 을 고정하면 여전히 0/8** — 시드로 대체되지 않는다.

대역 전체가 실패하면 roll, 산발적으로 실패하면 시드(T1)다.

### T3. ⚠ 표면 샘플링 — 정점만 담으면 부재가 텅 빈다
가장 위험했던 버그다. 스캐너가 1m 짜리 4040 프로파일 기둥을 **관통**했다. 부재 정점
1,694개가 전부 양 끝단에 몰려 있고 중간 1m 에는 0개였다 — 점군 SDF 는 점이 있는 곳만
장애물로 보므로 중간이 텅 빈 복도가 됐다.

CAD 압출·판재는 정점이 모서리에만 있다. `GetPointsAttr()` 만 담으면 안 되고 **삼각형
면적 비례 표면 샘플링**(`mesh_sampling.py`)을 써야 한다.

| | 전체 점 | 기둥 중간(0.3~0.7Z) |
|---|---|---|
| 이전 | 88,190 | **0점** ← 관통 |
| 현재 | 1,301,122 | 6,963점, Z 전 구간 균일 |

점이 15배 늘어도 판정 속도는 그대로다(SDF 격자 조회라 점 수와 무관).

> **검증도 틀렸었다.** "SDF 가 부재를 본다(3.3mm)"고 확인했는데 부재 **자신의 정점**으로
> 질의해 끝단만 본 것이었다. 표면 전체를 훑었어야 했다.

### T4. 캡슐은 꺾인 링크를 직선으로 덮어 오탐을 낸다
링크 캡슐이 관절 원점을 잇는 직선이라 꺾인 link4 를 덮으면 빈 공간까지 침범한다.
**실측: 캡슐 축간 58mm 로 "충돌"인데 실제 메시 거리는 93mm.**

Phase 2 후보 필터의 캡슐 world 는 과대평가(안전 방향)라 그대로 두고 메시 검사를
**추가**했다. 순서는 캡슐 scene → 메시 env → 메시 self 다.

### T5. `start` 거부 연쇄 — 명령각 vs 실측각
`is_path_safe(q0, q1)` 가 `start(...)` 를 돌려주면 **출발 자세가 이미 여유 밖**이라는
뜻이다. 이때 목표까지 거부하면 이후 모든 이동이 같은 이유로 막힌다. hand-eye 하니스
실측에서 **19 자세 중 7개가 이 연쇄로 소실**됐다.

원인은 `q0` 를 어디서 읽느냐다.

| | `q0` 출처 | 연쇄 위험 |
|---|---|---|
| 파이프라인 | `self._q` = **명령** 관절각 | 없음 — 게이트를 통과한 값을 다시 쓴다 |
| 하니스 | 아티큘레이션 **실측** 관절각 | **있음** |

드라이브 계통 오차(joint1 +1.4°) 때문에 게이트를 통과한 목표와 실제 도달 자세가 달라져
여유 밖으로 밀린다. 하니스는 `start` 사유일 때 경고만 남기고 이동한다 — 출발 자세가
나쁠 때 거기 머무는 것이 더 나쁘고, 목표는 이미 `is_pose_safe` 를 통과했기 때문이다.

홈 자세 여유 실측(기준 자가 20 / 환경 25mm): artec home 65.3 / 44.9mm(최근접 link4),
`HOME_JOINTS_DEG["artec"]` 66.9 / 180.4mm. **홈에서 출발하는 한 파이프라인은 이 연쇄에
걸리지 않는다.**

### T6. 메모리 — PC 를 멈춘 적이 있다
표면 샘플링에 상한이 없어 OOM 으로 작업 PC 가 멈췄다(EXIT=137). 세 겹으로 막았다.

1. `per_tri_cap` / `max_pts` / float32 — 극단 입력(10m² @1mm = 1억점)이 8,196점으로 차단
2. 삼각분할 완전 벡터화 — 파이썬 루프가 직접 원인이었다(23만 삼각형 0.03초)
3. `/tmp/runguard.sh <RSS_GB> <명령…>` — 초과 시 프로세스만 죽인다.
   `ulimit -v` 는 CUDA 가상주소 예약을 깨뜨리므로 **Isaac 에 쓰지 말 것**

### T7. ⚠ IK 를 "통일"해도 두 솔버는 여전히 다른 자세를 낸다

2026-06 에 **계획 경로**는 해석 IK 로 통일했고 그건 지금도 유지된다
(`ik_provider.py`, `xarm_interface.py`, `artec_multipass_scan_session.py:2069`).
그런데 **통일의 범위와 한계**를 둘 다 오해하기 쉽다.

**① 범위 — 캘리브 스크립트는 통일 대상이 아니었다.** 아직 SDK IK 를 부르는 곳:

```
scripts/artec/hand_eye_calib.py:210      scripts/phoxi/hand_eye_calib.py:177
scripts/artec/intrinsic_calib.py:157     scripts/phoxi/make_poses.py:171
utils/control/hardware_layer.py:303      utils/control/theta_planner.py:126
```

**② 한계 — FK 를 맞춰도 IK 는 안 맞는다.** 이게 더 중요하다.
7축은 **같은 TCP 를 만드는 팔 형상이 무한히 많다**(여유자유도). 두 솔버가 서로 다른
팔꿈치·손목 분기를 고르면 손끝은 같아도 **팔은 전혀 다른 곳을 지난다.**

2026-09-15 실측 — FK 재교정(1mm/0.18°) **이후**에도:

| | Δq 최대 | 해석 FK 오차 | 컨트롤러 FK 오차 | 같은 TCP? |
|---|---|---|---|---|
| hemi_00 | **207°** | 1.61mm | 0.00mm | **예** |
| hemi_02 | **251°** | 1.61mm | 0.00mm | **예** |
| hemi_04 | **140°** | 1.53mm | 0.00mm | **예** |

**사고** — 캘리브 자세를 `ee_pose` 로 저장했더니 `hand_eye_calib` 이 그걸
컨트롤러 IK 로 **다시 풀어서** 갔다. 해석 IK q 로 통과시킨 충돌 검사가 무의미해졌고,
첫 자세 이동에서 턴테이블과 충돌 직전까지 갔다(비상정지). 8개 중 3개가 위험했다 —
1개는 자세 자체가 거부 대상, 2개는 경로 중간에 `mid(link6)`.

**해법은 "IK 를 통일"이 아니라 "IK 를 다시 풀지 않는 것"이다.**
검증한 **관절각을 그대로 저장하고 그대로 실행**하면 시드·솔버에 의존하지 않는다
(`gen_calib_poses.py` → yaml `joints` 키 → `set_servo_angle`).

> 같이 드러난 것 — `is_pose_safe` 만으로는 부족하다. 자세는 안전한데 **가는 길**이
> 막힌 경우가 있다. 자세마다 home 을 경유한다면 `is_path_safe(HOME, q)` 를 봐야 한다.
>
> 그리고 `kin.ik` 는 해를 관절 한계로 **clip 한 뒤에도 ok=True 를 돌려준다.**
> 한계에 정확히 얹힌 자세가 통과하므로(실측 16개 중 4개가 여유 0.00°) 호출부가
> 여유를 따로 확인해야 한다 — 컨트롤러가 `set_servo_angle code 10` 으로 거부한다.

### T8. 검증할 때 지킬 것
- 대조군을 두고 **A/B** 로 본다(env 스위치). "고쳤더니 좋아졌다"를 한 번의 실행으로
  판단하지 않는다 — 이 프로젝트의 boundary 지표는 실행마다 867~918mm 로 흔들린다.
- 의존성을 늘리지 않는다. hpp-fcl·pinocchio 는 `env_isaacsim` 에만 있고 실물 환경
  (`mms-env`)에는 없어, 쓰면 sim/real 격차가 되살아난다. 현재 구현은 numpy/scipy 뿐이다.
