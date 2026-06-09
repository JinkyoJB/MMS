# mms_artec.backends — 실물 / Isaac Sim 백엔드 전환

`ArtecMMSConfig.backend` 로 robot / turntable / scanner 를 **실물 하드웨어("real")**
또는 **Isaac Sim 시뮬레이션("isaac")** 으로 바꾼다. 실물이 없어도(Ubuntu + Isaac Sim)
모션·제어·플래닝 코드를 계속 개발할 수 있다.

## 사용법

```python
cfg = ArtecMMSConfig(
    artec=ArtecConfig(...),
    turntable_frame_yaml=..., sensor_frames_yaml=..., T_EC_key="T_EC_artec",
    backend="isaac",          # "real" | "isaac"
    isaac_headless=False,     # isaac 일 때 GUI 표시 여부
)
with ArtecMMS(cfg) as mms:
    robot, turntable = mms.create_hardware()   # backend 에 맞는 robot/turntable
    robot.go_home(sensor="artec", confirm=False)
    ...
```

`main_artec.py` 는 상단 `BACKEND = "real" | "isaac"` 토글로 전환한다.

- **real**: Windows + xArm SDK + Artec SDK + Ezi-SERVO. 평소처럼 `python main_artec.py`.
- **isaac**: Isaac Sim python 으로 실행 — `~/isaacsim/python.sh main_artec.py`.
  (일반 python 으로 isaac 백엔드를 import 하면 실패)

## 구조

| | real | isaac |
|---|---|---|
| robot | `utils.robot.XArmInterface` | `isaac.IsaacXArm` (+ `.arm` shim) |
| turntable | `utils.turntable.Turntable` | `isaac.IsaacTurntable` |
| scanner | `mms_artec.sensor.ArtecClient` | `isaac.IsaacArtecScanner` |

하드웨어/시뮬 의존 모듈은 **팩토리(`build_hardware`/`build_sensor`) 분기 안에서
lazy import** → 어느 플랫폼에서도 `mms_artec.system` import 가 안전하다.

### Isaac 백엔드 핵심 설계
- `isaac_world.IsaacWorld`: 프로세스당 1개 `SimulationApp` + World + `v2.usd`.
  카메라 광학(Space Spider), 드라이브 게인, joint1 한계 정상화, 로봇 중력 비활성.
  robot/turntable/scanner 가 공유.
- `xarm7_kinematics`: 해석적 xArm7 DH FK + 수치 DLS IK (scipy 비의존, 자기일관).
- `IsaacXArm`: **이상적 키네마틱 로봇** — `get_joint_angles/get_pose` 는 명령 관절각
  기반(충돌로 인한 sim 물리 정착 오차와 무관, 정확). 모션은 sim 아티큘레이션을
  관절공간 구동(set_joint_positions + 드라이브 타깃)으로 시각화. `.arm` shim 으로
  orchestration 의 raw `robot.arm.*`(set_position/get_inverse_kinematics/…) 지원.
- `IsaacTurntable`: **실물처럼 RevoluteJoint 각도 드라이브**(`drive:angular:physics:targetPosition`)
  로 원판을 회전. 프레임은 kinematic(anchor), 원판은 dynamic(조인트 구동), 객체는
  kinematic 으로 원판 실측각에 동기. 조인트 부호는 시작 시 자동 캘리브레이션.
  θ(rad) 추적, `getActualPos()` = 실측 디스크각.

## 상태

- **Phase A (완료·검증)**: robot + turntable 제어. go_home/fk/ik/move/플래너/
  hardware_layer 가 sim 에서 동작. (검증: go_home 오차 0°, ik→fk 왕복 0mm, planner
  feasible θ 탐색, 클린 종료)
- **Phase B (예정)**: 스캐너 — Isaac 카메라 depth→포인트클라우드→정합/퓨전→mesh.
  `sim_model` / `sim_mesh_ops` / `IsaacScanSession` 구현 + `artec_process` isaac 분기.
  (open3d 필요: `~/isaacsim/python.sh -m pip install open3d`)
