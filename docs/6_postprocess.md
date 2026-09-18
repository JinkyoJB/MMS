# 후처리와 라이브 시각화

> 스캔이 끝난 뒤 Artec SDK General Pipeline 으로 최종 메시를 만드는 단계와, 스캔 중
> 상태를 눈으로 확인하는 뷰어.

---

## 1. 실행과 설정

후처리는 스캔이 끝나면 `main_artec.py` 가 자동으로 부른다. 스캔 없이 후처리만 반복
실험하려면 `7_real_commands.md` §3 을 본다.

설정은 `ArtecProcessSettings`(`mms_artec/system.py`)다. **끄고 켜는 것으로 대부분을
조절한다.**

| 키 | 뜻 | 기본 |
|---|---|---|
| `dev_mode` | 개발 모드. 켜면 무거운 단계(3a·5)를 건너뛴다 | False |
| `do_serial_registration` | frame-to-frame 정합 | True (streaming 이면 보통 False) |
| `do_global_registration` | IModel 전체 정합 | True (`hints_applied` 면 자동 skip) |
| `do_outliers_removal` | 이상점 제거 | True |
| `do_small_objects_filter` | 작은 파편 제거 | True |
| `fusion` | `"poisson"` / `"fast"` / `"none"` | `"poisson"` |
| `do_simplify` | 메시 단순화 | True |
| `do_texturize` | UV·텍스처 baking | True |
| `export_obj_path` / `export_sproj_path` | 결과 저장 경로 | 타임스탬프로 자동 |

캡처는 **streaming(`IScanningProcedure`) 하나**다. `use_multipass_scan` 으로
멀티패스(lookaround→nbv→flip · tracking-lost 복구) 여부만 고르고,
`streaming_scan_settings` / `multipass_settings` 에 각 설정을 넣는다.

> 옛 discrete 경로(`artec_scan_session.py`)와 그걸 고르던 `use_streaming_scan`·
> `scan_settings` 는 2026-09-16 제거했다. 오래 안 쓰였고, 세 갈래 분기가
> "지금 어느 코드가 도는지" 를 헷갈리게 만들었다.

---

## 2. 파이프라인 순서

**표의 순서가 곧 코드 호출 순서다.** Studio GUI 의 라벨 순서와 다르다.

| 순서 | 알고리즘 | 단위 | 비고 |
|---|---|---|---|
| 1 | SerialRegistration | frame-to-frame | streaming 이 이미 정합했으면 불필요 |
| 2 | GlobalRegistration | IModel 전체 | `hints_applied=True` 면 **자동 skip**(`5_flip.md` §3②) |
| 3a | OutliersRemoval | per-frame | **★ Fusion 전.** `dev_mode` 면 skip |
| 3b | SmallObjectsFilter | per-frame | **★ Fusion 전** |
| 4 | Poisson / FastFusion | clean frames → composite | watertight mesh |
| 5 | MeshSimplify | composite | `dev_mode` 면 skip |
| 6 | Texturization | composite + frame tex | UV/atlas + baking → OBJ/sproj |

자료구조는 SDK native 를 그대로 쓴다(변환·래핑 없음).

```
IFrame → IFrameMesh → IScan → IModel → ICompositeMesh
```

streaming 모드에서는 `result.ctx = None` 이고, θ 와 EE pose 같은 외부 메타는 별도
timeline CSV 로 남는다.

nbv 가 frontier 검출용으로 만드는 임시 메시는 이 파이프라인이 아니라 `fast_fusion`
(정확히는 `pcd_to_mesh_poisson` depth 6)이다. 판단용이라 가볍게 만든다.

---

## 3. 라이브 시각화

스캔 중 노이즈·드리프트·멈춤을 눈으로 확인하는 누적 컬러 포인트클라우드 뷰어다. 결과
메시만으로는 진단이 어렵다.

누적 좌표는 **SDK 자신의 정합행렬** `FrameEvent.transformation`(sensor→scan-world)만 쓴다.
θ 나 `turntable_frame.yaml`, hand-eye 에 **의존하지 않으므로 화면이 곧 SDK SLAM 결과 그
자체**다.

노이즈 제거는 두 겹이다. **A** 는 per-frame voxel(6mm)에서 점 수가 4 미만인 고립 점을
버린다(flying-pixel). **B**(옵션, fail-open)는 probe envelope 수직 실린더 멤버십 게이트다 —
축 대칭이라 물체가 돌아도 불변이고 단일 `T_CB` 로 전 회전을 커버한다. **B 는 표시 필터일
뿐 누적 변환에는 영향이 없다.**

### 실행 — 반드시 별도 터미널

```bash
# 터미널 A
env -u PYTHONPATH $MMS_PYTHON main_artec.py
# 터미널 B
env -u PYTHONPATH $MMS_PYTHON scripts/artec/live_scan_view.py
```

순서는 무관하고, A 가 끝나면 B 도 자동 종료된다. 정합이 끊기면 배경이 빨개진다.

파이프라인 쪽은 컨트롤러 역할만 한다 — OK 프레임을 누적해
`output/_live_latest.npy` 로 atomic write 하고, 뷰어가 그 파일을 읽는다.

---

# 〔부록〕 Troubleshooting

### T1. Texturize 가 `0x80010203` 으로 실패한다
Cleaning(3a/3b)이 Fusion 뒤로 갔을 때 나온다. outlier 가 박힌 composite 가 그대로
Texturize 까지 가기 때문이다. **OutliersRemoval 과 SmallObjectsFilter 는 반드시 Fusion
전**이어야 한다.

### T2. 뷰어 창이 뜨자마자 죽는다
반드시 **사용자가 다른 터미널에서 직접** 실행해야 한다. Open3D 신형
`O3DVisualizer`(Filament)를 `Popen` 자식으로 띄우면 즉시 죽고, legacy `Visualizer` 는 동적
PointCloud 를 못 그린다. 이 조합이 유일하게 검증된 구성이다.

### T3. hint 를 줬는데 GlobalRegistration 이 다시 흩뜨린다
`hints_applied` 가 안 켜진 것이다. flip 의 centroid-pivot hint 가 적용되면 이 플래그가
켜지고 2단계를 건너뛴다. 켜졌는데도 어긋나면 hint 자체를 의심한다
(`5_flip.md` T1).
