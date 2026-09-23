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
| 4 | Poisson / FastFusion | clean frames → composite | watertight mesh |
| 4b | SmallObjectsFilter | **composite mesh** | **★ Fusion 뒤.** fusion 전에 걸면 0x80010201 (2026-09-23 실측, 5s 에 462조각→1) |
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
OutliersRemoval(3a)이 Fusion 뒤로 갔을 때 나온다. outlier 가 박힌 composite 가 그대로
Texturize 까지 가기 때문이다. **OutliersRemoval 은 반드시 Fusion 전**이어야 한다.
반대로 **SmallObjectsFilter 는 Fusion 뒤**다 — 메시 입력이라 스캔만 있는 모델에 걸면
`0x80010201` 로 실패한다(2026-09-23 까지 모든 run 이 그렇게 실패하고 있었다).

### T2. 뷰어 창이 뜨자마자 죽는다
반드시 **사용자가 다른 터미널에서 직접** 실행해야 한다. Open3D 신형
`O3DVisualizer`(Filament)를 `Popen` 자식으로 띄우면 즉시 죽고, legacy `Visualizer` 는 동적
PointCloud 를 못 그린다. 이 조합이 유일하게 검증된 구성이다.

### T3. hint 를 줬는데 GlobalRegistration 이 다시 흩뜨린다
`hints_applied` 가 안 켜진 것이다. flip 의 centroid-pivot hint 가 적용되면 이 플래그가
켜지고 2단계를 건너뛴다. 켜졌는데도 어긋나면 hint 자체를 의심한다
(`5_flip.md` T1).

---

## 4. run 산출물 — 정합을 오프라인에서 개선하려면 (2026-09-22)

데이터 수집(preview→lookaround→nbv→flip)은 돌아가므로, 정합 알고리즘은 **run 을 다시
돌리지 않고** 아래 산출물로 고친다. 모두 기본으로 남는다(`--no-sproj` 면 sproj 만 빠짐).

| 파일 | 무엇 | 용도 |
|---|---|---|
| `output/<RUN>/aligned/aligned.sproj` (+ `scans/`) | **SDK 정합 전** IScan(정점·사진·uv). 모든 패스가 파이프라인 변환(핸드아이·턴테이블 각·flip 힌트/yaw/greg)으로 한 좌표계에 놓여 있다. Artec 은 페이로드를 sproj 옆 `scans/` 에 쓰므로 프로젝트마다 폴더 하나 | **Artec Studio 로 여는 파일** — 수작업 후처리. 정합 재실험·텍스처 매칭의 원본 |
| `output/<RUN>/final/final.sproj` · `output/<RUN>/final.obj` | 후처리 끝난 최종본(융합 메시 포함) | 결과 비교 |
| `output/<RUN>/README.txt` · `timeline.csv` | 파이프라인이 쓰는 안내(어느 파일을 열고 Studio 에서 뭘 하나)·타임라인 | 후임 인수인계 |

> 규칙은 `utils/run_paths.py` 한 곳에 있다. 2026-09-23 이전 run 은 `output/artec_lookaround_<RUN>*`·
> `output/debug/<단계>_<RUN>/`·`output/scan_dumps/<RUN>/`·`events_<RUN>.jsonl` 로 흩어져 있었고
> `python scripts/artec/tidy_output.py --apply` 로 이 규칙으로 옮긴다(옮기기 전 `--apply` 없이 계획만).
> 조회 함수(`scan_dumps_dir`·`events_path`)는 옛 위치도 찾는다.
| `output/<RUN>/scan_dumps/scanNN_<stage>_poseK.npz` | pass 별 점(스캔월드)·적용 `T_pre_mm`·`R_phys`·`master_T_CB`·`T_BC_new`·`S` | 어떤 변환이 적용됐는지 — sproj 의 프레임 변환을 되돌릴 때 |
| `output/<RUN>/events.jsonl` · `run.log` | lost·밴드 전환·복구·병합(method) 이벤트 · 콘솔 로그 | `scripts/artec/lost_report.py` |
| `output/<RUN>/debug/<단계>/` | 거리 이미지·뷰어 녹화(`3_lookaround.md` §0) | 캡처가 의도대로 됐나 |

flip 힌트만 따로 검증: `python scripts/artec/reg_hint_test.py --run RUN --preview-master … --preview-flip …`
— 설계대로(디스크면 + 물체높이/2 피벗) 되돌린 flip 과 run 이 실제 적용한 배치를 나란히
`output/registration_test/<RUN>/` 에 PLY/OBJ(+Poisson 메시, 옆면 그림, report.txt)로 남긴다.
`--H-mm` 로 물체 높이를 바꿔 가며 민감도를 본다(`5_flip.md` §3).

그 배치에서 **SDK 후처리를 그대로** 이어 돌리려면: `python scripts/artec/reg_hint_post.py --run RUN
--preview-master … --preview-flip … [--H-mm] [--show]` — raw sproj 의 flip 프레임 변환만 설계 힌트로
갈아끼운 뒤 OutliersRemoval → GlobalRegistration → PoissonFusion 을 단계마다 sproj/PLY 로 남기고
(`output/registration_test/<RUN>/post_H<mm>/`), GR 이 flip 을 힌트에서 얼마나 움직였는지 report.txt
에 적는다. `--show` 면 fusion 메시를 창으로 띄운다.

재실험: `python scripts/artec/reg_offline.py [--run RUN] [--sub N] [--methods hint,img,greg]`
— sub 스캔을 run 의 T_pre 로 되돌린 뒤 방법별로 다시 정합하고 `output/reg_offline/<RUN>/`
에 PLY 와 지표(겹침 NN 거리)를 남긴다. 새 방법은 그 파일의 `METHODS` 에 함수 하나 추가.

### Artec Studio 에서 이어서 할 때 — 어느 파일을 여나 (2026-09-22)

**`aligned/aligned.sproj` 를 연다.** 최종 sproj 에는 후처리로 만든 **융합 메시가 한 항목 더**
들어 있다(`Scans Count=3`, 마지막이 `DataType="1"` 에 `Items="1"`). Autopilot·Global
Registration 은 원시 스캔을 대상으로 도는 도구라, 메시가 섞인 프로젝트를 주면 원하는 대로
안 돈다. `aligned` 는 IScan 만 있다(`DataType="0"`) — 단, 파이프라인 변환은 이미 프레임에
들어 있어 패스들이 대략 한 자리에 놓인 채 열린다. 정합이 완벽하지 않아 파이프라인이
watertight 를 못 만든 run 이라도 이 파일에서 Studio 로 이어 간다: Err 확인 → (필요하면 Align
으로 flip 스캔을 손으로 맞춘 뒤) Global registration → Fusion → Fix holes → Texture → Export.

```
output/<RUN>/aligned/aligned.sproj   ← Studio 로 여는 것 (IScan 만, 변환 적용됨)
output/<RUN>/final/final.sproj       ← 우리 후처리 결과 (융합 메시 포함)
output/<RUN>/README.txt              ← 같은 안내를 run 마다 자동으로 써 둔다
```

**scan 목록의 `Err` 를 먼저 본다.** 정상은 0.2~0.3mm 다(프레임별 `regErr` 와 같은 값).
1mm 를 넘으면 그 스캔에 **정합 실패 프레임이 섞인 것**이지 물체나 캘리브 문제가 아니다.
`ignore_registration_errors=True`(필수, `3_lookaround.md` T10) 라 SDK 가 정합 실패 프레임도
예측 자세로 스캔에 넣기 때문이다. 파이프라인이 그 프레임들을 **꼬리와 중간 모두** 지우지만
(`_trim_lost_tail`), 지우지 못한 run 의 프로젝트라면 Studio 에서 그 구간 프레임을 지우고
쓰는 편이 빠르다. 로그의 `[정리] 미정합 프레임 제거 — 꼬리 N · 중간 M` 줄과
`남은 프레임 정합오차` 줄이 그 결과다.

## 5. 텍스처는 Artec Studio 에서 (2026-09-22)

SDK Texturize 는 CPU 단일코어다 — `TexturizationSettings` 에 GPU 항목이 없고, 1912 프레임에
15~20분(실측, GPU 0%). 그래서 **기본 끔**(`do_texturize=False`, 켜려면 `--texturize`).

### 🔴 저장된 프로젝트로는 텍스처링이 **안 된다** (2026-09-23 실측)

**저장→로드에서 프레임별 UV 가 사라진다.** `output/20260923_135151/aligned/aligned.sproj` 확인:

| 항목 | 상태 |
|---|---|
| 텍스처 이미지 | **있음** — 프레임마다 1280×960×3, `.tscan` 1.5~1.7GB, 밝기 평균 65~159 (실제 내용) |
| `has_image()` · `is_textured()` | 둘 다 True |
| `uv()` | 배열은 오지만 **전부 NaN** (표본 26프레임 × 2스캔, 유효 0) |
| SDK `texturize()` | **실패 `0x80010203`** (융합 뒤, 소스 스캔을 붙여도 동일) |

**live 에서는 UV 가 정상이다** — run 중 `image_match` 가 `cKDTree(uv_px)` 를 쓰는데 scipy 는
NaN 에 예외를 낸다. 그게 안 났고 특징점을 찾았으니(`sub=844 master=379`) 캡처 시점의 uv 는
유한하다. 즉 **`save_project`/`load_project` 왕복에서 텍스처↔기하 매핑이 유실된다.**

그래서 "Studio 에서 텍스처를 입히면 된다" 던 기존 계획은 **성립하지 않는다.** 남은 길:

| 방법 | 대가 |
|---|---|
| run 중에 SDK Texturize (`--texturize`) | CPU 단일코어 15~20분. 그리고 **우리 융합 메시**에만 입는다 — Studio 에서 정합을 다듬은 메시에는 못 입힌다 |
| 바인딩 수정 (**진짜 해결**) | 로드 후 텍스처 좌표를 다시 계산하는 SDK 호출을 노출하거나, save 에 텍스처 캘리브가 빠지는지 확인. C++ 바인딩 작업 필요 |
| 우리가 직접 굽기 | run 중 프레임별 `image()`+`uv()`+변환을 덤프해 두고, 나중에 어떤 메시에든 투영해 입힌다. SDK 없이 가능 |

### 직접 굽기 (2026-09-23 구현) — Studio 에서 다듬은 메시에도 입힐 수 있다

run 이 패스마다 프레임별 **사진+uv+정점+변환**을 `output/<RUN>/texture_frames/scanNN_<stage>_poseK/`
에 남긴다(`dump_texture_frames=True`, 패스당 40 프레임 ≈ 40MB). uv 가 유효한 live 시점에 뽑는다.

```bash
python scripts/artec/bake_texture.py --run <RUN> --mesh output/<RUN>/final.obj           # 우리 융합 메시
python scripts/artec/bake_texture.py --run <RUN> --mesh <Studio 에서 내보낸 obj> --out x.ply  # Studio 정합·융합 메시
```
프레임마다 (정점↔픽셀) 대응에서 DLT 로 투영행렬을 맞추고(캘리브 불요), 메시 정점을 각 프레임에
투영해 가시성(프레임 자신의 깊이 + 법선 방향)을 거른 뒤 입사각 가중 평균으로 **정점 색**을 입힌다.
uv 의 v 방향(SDK 미명세)은 두 규약으로 구워 프레임 간 색 일관성이 좋은 쪽을 자동 선택한다.
출력은 정점 색 PLY(CloudCompare/MeshLab). v1 은 UV 아틀라스가 아니라 정점 색이다 — 메시가 성기면
색도 성기다. 합성 시험(실제 융합 메시 + 가상 카메라 30대): 정점 83% 착색, 오차 중앙 <0.05.
⚠ 실물 프레임으로는 아직 미검증 — 다음 run 의 `texture_frames/` 로 한 번 돌려 볼 것.
Studio 메시는 **master 스캔월드(mm)** 좌표여야 한다(프로젝트 좌표 그대로 내보내면 된다).

정합을 다듬는 동안은 텍스처가 필요 없으므로 `--test`(Texturize 생략 + 형상만)로 돌린다.
