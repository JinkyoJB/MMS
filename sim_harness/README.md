# sim_harness — Isaac Sim 검증 하니스

알고리즘을 실물에 올리기 전에 **ground-truth 를 아는 가상환경에서 먼저 확인**하는
스크립트들. `docs/1_calibration.md` · `2_phase1.md` · `3_phase2.md` 가 이 파일들을 참조한다.

| 파일 | 검증 대상 | 관련 문서 |
|---|---|---|
| `MMS_ext.py` | 기본 환경·카메라·드라이브 게인 (나머지의 베이스) | — |
| `MMS_ext_calibration.py` | **hand-eye** — ChArUco 렌더 → solvePnP → `AX=ZB` → GT 대조 | `1_calibration.md` |
| `MMS_ext_calibration2.py` | **턴테이블 축** — rim 자동추출 → 원 피팅 → GT 대조 | `1_calibration.md` |
| `MMS_ext_phase1.py` | Phase 1 — GT 누적으로 view-planning 검증 (SLAM 없음) | `2_phase1.md` |
| `MMS_ext_phase1_recovery1.py` | recovery — **빗나감**(대상물을 측면으로 조준) | `2_phase1.md` §6 |
| `MMS_ext_phase1_recovery2.py` | recovery — **윗면 미포착**(너무 낮은 el) | `2_phase1.md` §6 |
| `MMS_ext_phase2_nbv.py` | Phase 2 NBV 루프 | `3_phase2.md` §6.4 |

## ⚠ 실행하려면 Isaac 트리에 두어야 한다

Isaac Sim 의 `standalone_examples` 로더가 경로를 기준으로 동작하므로, 리포에 두고
바로 실행할 수 없다. **복사하거나 심볼릭 링크**를 건다.

```bash
DST=~/isaacsim/standalone_examples/play/MMS
mkdir -p "$DST"
cp sim_harness/MMS_ext*.py "$DST"/          # 또는: ln -s "$PWD"/sim_harness/MMS_ext*.py "$DST"/

env -u PYTHONPATH ~/isaacsim/python.sh "$DST/MMS_ext_calibration.py"
```

> 심볼릭 링크를 쓰면 리포에서 수정한 내용이 바로 반영된다(권장).

### 경로는 자동으로 찾는다

절대경로를 소스에 박지 않는다. 필요하면 환경변수로 지정한다.

| 변수 | 무엇 | 기본 |
|---|---|---|
| `MMS_ROOT` | MMS 리포 위치 | 스크립트 상위 → 알려진 경로 순으로 탐색 |
| `MMS_ASSET_ROOT` | 자산(`2_3Dassets`) 루트 | `mms_paths.py` 가 해석 |
| `MMS_HARNESS_OUT` | 캡처·로그 출력 위치 | `~/isaacsim/standalone_examples/play/MMS` |
| `MMS_XARM_SDK` | xArm SDK **소스** 경로 | 미설정(=`mms-env` 의 pip 설치본 사용) |

Isaac 트리로 복사해서 돌리면 `__file__` 이 리포 밖이라 상위 탐색이 실패한다.
그때는 `MMS_ROOT` 를 준다.

```bash
export MMS_ROOT=/경로/MMS
env -u PYTHONPATH ~/isaacsim/python.sh "$DST/MMS_ext_calibration.py"
```

---

## ⚠ 이 하니스들은 **v2(옛) 씬 기준**이다

작성 시점이 2026-06 이고, 씬은 2026-08 에 **v3 로 바뀌었다**(`docs/v3_sim_migration.md`).
그대로 돌리면 prim 을 못 찾고 조용히 실패한다.

| | 하니스(v2) | 현재(v3) |
|---|---|---|
| 씬 | `frame_xarm7_spider_turntable/v2.usd` | `frame_xarm7_spider_turntable_v2/v3_scene.usd` |
| 카메라 prim | `/World/xarm7/link7/Artec_Space_Spider_mm/Camera` | `/World/xarm7/link7/tool/spider/Camera` |
| 로봇 base | `(0.538, 0, 1.407)` | `(0.365, 0, 1.500)` |

**되살리려면** 각 파일 상단 상수(`USD_PATH`·`CAMERA_PRIM`·`ROBOT_PRIM` 등)를 v3 기준으로
맞춰야 한다. 현재 값은 `mms_artec/backends/isaac/isaac_world.py` 를 참고한다.

> 경로 하드코딩은 이미 제거했으므로(위 환경변수), 남은 작업은 **prim 경로와 씬 파일**
> 갱신이다. 검증 로직 자체는 그대로 쓸 수 있다.

## ★ 설계 원칙 — sim 에 중복 구현하지 않는다

이 하니스는 **USD/Isaac 환경 코드만** 갖는다. 검출·솔버·IK·기하 연산은 MMS 본체
모듈을 **파일경로로 로드해 그대로 쓴다.** sim 과 real 이 다른 코드를 돌면 sim 검증이
의미를 잃기 때문이다.

주의점(자세히는 `docs/1_calibration.md` §6):
- Isaac 런타임에 동명 `utils` 패키지가 있어 `from utils...` 가 깨진다 → **파일경로 로드**
- `@dataclass` 때문에 `sys.modules` 등록이 필수
- 옮길 모듈은 레포 내부 import 가 없는 **자기완결** 형태여야 한다
  (`handeye_geometry` · `ik_provider` · `xarm7_kinematics` 패턴)

## 산출물

실행하면 Isaac 트리 아래 `captures*/` 에 캡처·로그가 쌓인다(리포에는 넣지 않는다).
