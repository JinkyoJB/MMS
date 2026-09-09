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
