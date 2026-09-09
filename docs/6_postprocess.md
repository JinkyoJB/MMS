# 후처리 — `artec_process` & 라이브 시각화

> 스캔이 끝난 뒤 Artec SDK General Pipeline 으로 최종 메시를 만드는 단계와,
> 스캔 중 상태를 눈으로 확인하는 뷰어.

---

## 1. 후처리 파이프라인 — `artec_process`

스캔 완료 후 SDK General Pipeline. **순서가 곧 코드 호출 순서** (Studio GUI 라벨과 다름).

| 순서 | 알고리즘 | 단위 | 비고 |
|---|---|---|---|
| 1 | SerialRegistration | frame-to-frame | `do_serial_registration=False` 기본 (streaming 이 이미 정합) |
| 2 | GlobalRegistration | IModel 전체 | `hints_applied=True` 면 **자동 skip** (§4 hint 가 authoritative) |
| 3a | OutliersRemoval | per-frame | **★ Fusion 전**. dev_mode 면 skip |
| 3b | SmallObjectsFilter | per-frame | **★ Fusion 전** |
| 4 | Poisson/FastFusion | clean frames → composite | watertight mesh |
| 5 | MeshSimplify | composite | 옵션. dev_mode 면 skip |
| 6 | Texturization | composite + frame tex | UV/atlas + baking → OBJ/sproj |

> ★ **Cleaning(3a/3b)은 반드시 Fusion 전.** Fusion 후에 두면 outlier 박힌 composite 가
> Texturize 까지 가서 실패(ErrorCode `0x80010203`). Phase 2 frontier 용 임시 mesh 는 `fast_fusion`.

자료구조 계층: `IFrame → IFrameMesh → IScan → IModel → ICompositeMesh`. Artec 은 SDK native
자료구조를 그대로 차용(변환/래핑 없음). streaming 모드에선 `result.ctx=None`, 외부 메타(θ, EE pose)는 timeline CSV.

---

## 2. 라이브 시각화

스캔 중 노이즈/드리프트/멈춤을 눈으로 확인하는 누적 컬러 포인트클라우드 뷰어. (결과 mesh 만으론 진단 어려움.)

- 누적 좌표 = **SDK 자신의 정합행렬** `FrameEvent.transformation` (sensor→scan-world). θ /
  turntable_frame.yaml / hand-eye 의존 **없음** → 화면 = SDK SLAM 결과 그 자체.
- 노이즈 제거 **A**: per-frame voxel(6mm) 점수<4 고립 점 제거(flying-pixel). **B**(옵션, fail-open):
  probe envelope 수직 실린더 멤버십 게이트(축 대칭이라 물체 회전해도 불변, 단일 `T_CB` 로 전 회전 게이트). B 는 표시 필터일 뿐 누적 변환 불변.
- **아키텍처(검증된 유일 구성)**: 파이프라인측은 컨트롤러만 — OK 프레임 누적 → `output/_live_latest.npy`
  atomic write. 뷰어는 **사용자가 다른 터미널에서 직접 실행** (`scripts/artec/live_scan_view.py`,
  Open3D 신형 **O3DVisualizer(Filament)**). legacy `Visualizer` 는 동적 PointCloud 못 그림, Popen 자식 Filament 창은 즉사 → 별도 터미널 필수.
- 실행: 터미널A `main_artec.py` / 터미널B `live_scan_view.py` (순서 무관, A 종료 시 B 자동 종료, 정합 끊기면 배경 빨강).

---

