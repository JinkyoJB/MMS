# postprocess — 후처리와 라이브 시각화

> 스캔 종료 후 Artec SDK General Pipeline 으로 최종 메시를 생성하는 단계와, 스캔 중
> 상태를 확인하는 뷰어.

---

## 1. 실행과 설정

후처리는 스캔 종료 시 `main_artec.py` 가 자동으로 호출한다. 스캔 없이 후처리만 반복
수행하려면 `7_real_commands.md` §3 을 참조한다.

설정은 `ArtecProcessSettings`(`mms_artec/system.py`)이며, 단계별 on/off 로 조절한다.

| 키 | 내용 | 기본 |
|---|---|---|
| `dev_mode` | 개발 모드. 무거운 단계(3a·5) 생략 | False |
| `do_serial_registration` | frame-to-frame 정합 | True (streaming 시 통상 False) |
| `do_global_registration` | IModel 전체 정합 | True (`hints_applied` 시 자동 생략) |
| `do_outliers_removal` | 이상점 제거 | True |
| `do_small_objects_filter` | 파편 제거 | True |
| `fusion` | `poisson` / `fast` / `none` | `poisson` |
| `do_simplify` / `do_texturize` | 메시 단순화 / 텍스처 baking | True / **False** |
| `export_obj_path` / `export_sproj_path` | 결과 저장 경로 | 타임스탬프 자동 |

캡처 경로는 **streaming(`IScanningProcedure`) 단일 경로**다. `use_multipass_scan` 으로
멀티패스(lookaround→nbv→flip · 추적상실 복구) 적용 여부만 선택하고, 세부는
`streaming_scan_settings` / `multipass_settings` 에 둔다. 구 discrete 경로
(`artec_scan_session.py`)와 `use_streaming_scan`·`scan_settings` 는 2026-09-16 제거하였다.

---

## 2. 파이프라인 순서

**표의 순서가 곧 코드 호출 순서이며, Studio GUI 의 라벨 순서와 다르다.**

| 순서 | 알고리즘 | 단위 | 비고 |
|---|---|---|---|
| 1 | SerialRegistration | frame-to-frame | streaming 이 정합을 마쳤으면 불필요 |
| 2 | GlobalRegistration | IModel 전체 | `hints_applied=True` 면 자동 생략(`5_flip.md` §3②) |
| 3a | OutliersRemoval | per-frame | **Fusion 전**. `dev_mode` 시 생략 |
| 4 | Poisson / FastFusion | clean frames → composite | watertight mesh |
| 4b | SmallObjectsFilter | **composite mesh** | **Fusion 후**. 앞에 두면 `0x80010201` |
| 5 | MeshSimplify | composite | `dev_mode` 시 생략 |
| 6 | Texturization | composite + frame tex | UV/atlas + baking → OBJ/sproj |

자료구조는 SDK native 를 그대로 사용한다(변환·래핑 없음).

```
IFrame → IFrameMesh → IScan → IModel → ICompositeMesh
```

streaming 모드에서는 `result.ctx = None` 이며, θ·EE pose 등 외부 메타는 timeline CSV
로 별도 기록된다. nbv 의 frontier 검출용 임시 메시는 본 파이프라인이 아니라
`fast_fusion`(`pcd_to_mesh_poisson` depth 6)으로 생성한다 — 판단용이므로 경량이다.

---

## 3. 라이브 시각화

스캔 중 노이즈·드리프트·정지를 육안으로 확인하는 누적 컬러 포인트클라우드 뷰어다.
결과 메시만으로는 진단이 어렵다.

누적 좌표는 **SDK 자체 정합행렬** `FrameEvent.transformation`(sensor→scan-world)만
사용한다. θ·`turntable_frame.yaml`·hand-eye 에 의존하지 않으므로 **화면이 곧 SDK SLAM
결과**다.

노이즈 제거는 2단이다. **A** 는 per-frame voxel(6mm) 내 점 수 4 미만의 고립 점을
제거한다(flying-pixel). **B**(옵션, fail-open)는 probe envelope 수직 실린더 멤버십
게이트로, 축 대칭이므로 물체가 회전해도 불변이고 단일 `T_CB` 로 전 회전을 커버한다.
**B 는 표시 필터이며 누적 변환에는 영향이 없다.**

### 실행 — 별도 터미널 필수

```bash
# 터미널 A
env -u PYTHONPATH $MMS_PYTHON main_artec.py
# 터미널 B
env -u PYTHONPATH $MMS_PYTHON scripts/artec/live_scan_view.py
```

순서는 무관하며 A 종료 시 B 도 자동 종료된다. 정합이 끊기면 배경이 적색으로 바뀐다.
파이프라인은 컨트롤러 역할만 수행한다 — OK 프레임을 누적해 `output/_live_latest.npy`
로 atomic write 하고, 뷰어는 그 파일을 읽는다.

---

## 4. run 산출물 — 오프라인 정합 개선

데이터 수집(preview→lookaround→nbv→flip)은 정상 동작하므로, 정합 알고리즘은 **run 을
재수행하지 않고** 아래 산출물로 개선한다. 모두 기본 생성된다(`--no-sproj` 시 sproj 제외).

| 파일 | 내용 |
|---|---|
| `aligned/aligned.sproj` (+ `scans/`) | **SDK 정합 전** IScan. 모든 패스가 파이프라인 변환(핸드아이·턴테이블 각·flip 힌트/yaw/greg)으로 한 좌표계에 놓여 있다. **Studio 로 여는 파일** |
| `final/final.sproj` · `final.obj` | 후처리 완료본(융합 메시 포함) |
| `scan_dumps/scanNN_<stage>_poseK.npz` | pass 별 점·적용 `T_pre_mm`·`R_phys`·`master_T_CB`·`T_BC_new` |
| `events.jsonl` · `run.log` | 추적상실·밴드 전환·복구·병합(method) 이벤트 (`lost_report.py`) |
| `debug/<단계>/` | 거리 이미지·뷰어 녹화 (`3_lookaround.md` §1) |
| `README.txt` · `timeline.csv` | run 별 자동 안내·타임라인 |

경로 규칙은 `utils/run_paths.py` 한 곳에 있다. 2026-09-23 이전 run 은 산재해 있으며
`scripts/artec/tidy_output.py --apply` 로 이관한다(미지정 시 계획만 출력).

**검증·재실험 스크립트**

| 스크립트 | 용도 |
|---|---|
| `reg_hint_test.py --run RUN --preview-master … --preview-flip … [--H-mm]` | 설계 힌트(디스크면+H/2 피벗) 배치와 run 실제 배치를 나란히 출력. 물체 높이 민감도 확인 |
| `reg_hint_post.py` | 위 배치에서 SDK 후처리(Outliers→GReg→Poisson)를 이어 수행하고 GR 이동량을 보고 |
| `reg_offline.py [--sub N] [--methods hint,img,greg]` | 방법별 재정합 후 겹침 NN 거리 비교. 새 방법은 `METHODS` 에 함수 추가 |

### Artec Studio 로 이어서 작업할 때

**`aligned/aligned.sproj` 를 연다.** `final.sproj` 에는 융합 메시가 한 항목 더 들어
있는데(`DataType="1"`), Autopilot·Global Registration 은 원시 스캔 대상 도구이므로 메시가
섞인 프로젝트에서는 정상 동작하지 않는다. `aligned` 는 IScan 만 포함하되 파이프라인 변환이
이미 프레임에 적용되어 있어 패스들이 대략 한 자리에 놓인 채 열린다. 파이프라인이
watertight 를 만들지 못한 run 도 이 파일에서 이어 간다 — Err 확인 → (필요 시 Align 으로
flip 스캔 수동 정합) → Global registration → Fusion → Fix holes → Texture → Export.

**scan 목록의 `Err` 를 먼저 확인한다.** 정상은 0.2~0.3mm 다. 1mm 초과는 그 스캔에
**정합 실패 프레임이 섞인 것**이며 물체나 캘리브 문제가 아니다.
`ignore_registration_errors=True`(필수, `3_lookaround.md` T10) 이므로 SDK 가 정합 실패
프레임도 예측 자세로 포함시키기 때문이다. 파이프라인이 이를 꼬리·중간 모두 제거하나
(`_trim_lost_tail`), 남은 run 이라면 Studio 에서 해당 구간을 삭제하는 편이 빠르다. 로그의
`[정리] 미정합 프레임 제거 — 꼬리 N · 중간 M` 줄이 그 결과다.

---

## 5. 텍스처

SDK Texturize 는 CPU 단일코어다 — `TexturizationSettings` 에 GPU 항목이 없고 1912
프레임에 15~20분 소요된다(실측, GPU 0%). 따라서 **기본 비활성**이며 `--texturize` 로 켠다.

### 저장된 프로젝트로는 텍스처링이 불가하다

**저장→로드 과정에서 프레임별 UV 가 소실된다.** 텍스처 이미지는 정상 보존되고
(`.tscan` 1.5~1.7GB, 프레임당 1280×960×3) `has_image()`·`is_textured()` 도 True 이나,
`uv()` 가 **전부 NaN** 이고 `texturize()` 는 `0x80010203` 으로 실패한다(표본 26프레임 ×
2스캔, 유효 0). 반면 live 에서는 `image_match` 의 `cKDTree(uv_px)` 가 예외 없이 동작하므로
캡처 시점의 uv 는 유한하다. 즉 **`save_project`/`load_project` 왕복에서 텍스처↔기하 매핑이
유실된다.** 따라서 "Studio 에서 텍스처를 입힌다" 는 방안은 성립하지 않는다.

| 대안 | 제약 |
|---|---|
| run 중 SDK Texturize (`--texturize`) | 15~20분 소요. **자체 융합 메시**에만 적용 가능 |
| 바인딩 수정 (근본 해결) | 로드 후 텍스처 좌표 재계산 SDK 호출 노출 필요. C++ 바인딩 작업 |
| 직접 굽기 (구현 완료) | 아래 |

**직접 굽기** — run 이 패스마다 프레임별 사진+uv+정점+변환을
`output/<RUN>/texture_frames/scanNN_<stage>_poseK/` 에 남긴다(`dump_texture_frames=True`,
패스당 40 프레임 ≈ 40MB). uv 가 유효한 live 시점에 추출하므로 Studio 에서 다듬은 메시에도
적용할 수 있다.

```bash
python scripts/artec/bake_texture.py --run <RUN> --mesh output/<RUN>/final.obj
python scripts/artec/bake_texture.py --run <RUN> --mesh <Studio 내보낸 obj> --out x.ply
```

프레임별 (정점↔픽셀) 대응에서 DLT 로 투영행렬을 추정하고(캘리브 불요), 정점을 각
프레임에 투영해 가시성(깊이+법선)을 거른 뒤 입사각 가중 평균으로 정점 색을 부여한다.
uv 의 v 방향(SDK 미명세)은 두 규약으로 구워 색 일관성이 좋은 쪽을 자동 선택한다. 출력은
정점 색 PLY 이며 UV 아틀라스가 아니므로 메시가 성기면 색도 성기다. 합성 시험(융합 메시 +
가상 카메라 30대)에서 정점 83% 착색, 오차 중앙값 <0.05.
⚠ **실물 프레임으로는 미검증** — 다음 run 의 `texture_frames/` 로 확인할 것.
Studio 메시는 **master 스캔월드(mm)** 좌표여야 한다.

정합 조정 중에는 텍스처가 불필요하므로 `--test`(Texturize 생략, 형상만)로 수행한다.

---

# 〔부록〕 문제 해결

### T1. Texturize 가 `0x80010203` 으로 실패한다
OutliersRemoval(3a)이 Fusion 뒤로 이동한 경우다. outlier 가 남은 composite 가 그대로
Texturize 로 전달되기 때문이다. **OutliersRemoval 은 Fusion 전**이어야 한다. 반대로
**SmallObjectsFilter 는 Fusion 후**다 — 메시 입력이므로 스캔만 있는 모델에 적용하면
`0x80010201` 로 실패한다.

### T2. 뷰어 창이 즉시 종료된다
반드시 **사용자가 별도 터미널에서 직접** 실행해야 한다. Open3D 신형
`O3DVisualizer`(Filament)를 `Popen` 자식으로 띄우면 즉시 종료되고, legacy `Visualizer` 는
동적 PointCloud 를 렌더링하지 못한다. 현 구성이 유일하게 검증된 조합이다.

### T3. 힌트를 적용했는데 GlobalRegistration 이 배치를 흩뜨린다
`hints_applied` 가 설정되지 않은 것이다. flip 힌트가 적용되면 이 플래그가 켜지고 2단계를
생략한다. 켜졌는데도 어긋나면 힌트 자체를 의심한다(`5_flip.md` T1).
