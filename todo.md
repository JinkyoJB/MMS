# TODO — 정합·후처리 자동화 (인수인계)

> 기준 run: `output/20260923_125718/` (2026-09-23, 눕힌 머스타드 병 ~55mm)
> 읽는 순서: 이 문서 → `README.md` → `docs/1_calibration.md` … `docs/6_postprocess.md`
> 용어·좌표계·캘리브는 `docs/1_calibration.md`, 실물 경로 함정은 `docs/troubleshooting.md`.

## 1. 지금 상태 한 줄

**스캔 경로(preview → lookaround → nbv → flip)로 데이터를 얻는 것까지는 된다.
정합이 안 된다 — 그래서 watertight 메시가 안 나온다.** 남은 일은 정합과 후처리를
사람 손 없이 파이프라인 안에서 끝내는 것이다.

무엇이 되는가 (기준 run 실측):
- lookaround 2자세 × 360° 완주, 프레임 정합오차 중앙 **0.22mm** / p90 0.27mm
- flip 패스도 완주 (726 프레임, 정합오차 중앙 0.18mm)
- 거리추종·밴드전환·복구·디버그 이미지·sproj 저장 전부 자동

무엇이 안 되는가:
- **lookaround ↔ flip 를 한 좌표계로 못 맞춘다** (아래 §2) ← **후임이 풀 문제는 이것**
- 후처리(Fusion → Texture → Export)가 파이프라인 밖, Artec Studio 수작업 (§4)

> ⚠ 기준 run 은 **flip 패스가 master 에 병합되지 않은 채** 끝났다(§1). 그 원인(게이트 문턱)은
> 2026-09-23 에 고쳤지만 **실측으로 확인되지 않았다.** 그러니 정합을 손대기 전에 run 을 한 번
> 돌려 `master scan_count ≥ 2` 를 먼저 확인할 것 — 그래야 정합 품질을 논할 수 있다.

---


## 2. 🔴 본문제 — flip 정합 (lookaround ↔ 뒤집은 뒤)

### 2.1 무엇이 어려운가

물체를 **사람이 손으로 뒤집어 다시 놓는다.** 힌트(핸드아이 + 턴테이블 각 + 180° 뒤집기)가
아는 것은 뒤집기 축과 반사면뿐이고, 다시 놓을 때 생긴 **축 둘레 회전(yaw) + 수평 이동(xy)
3 자유도는 원리적으로 모른다.** 그 3 자유도를 데이터로 찾아야 하는데:

- **기하**: 눕힌 병 footprint 가 거의 2회 대칭이라 최솟값이 둘이다 → 못 정한다
- **텍스처**: 라벨면이 lookaround 에선 위, flip 에선 바닥이라 **공통 텍스처 면이 없다**

### 2.2 지금까지 시도한 것 — 전부 실측 결과 있음

| # | 방법 | 코드 | 결과 |
|---|---|---|---|
| 1 | **힌트** (핸드아이+턴테이블각+뒤집기) | `_flip_unflip_B`, `_flip_pivots_B` | 기본값. 피벗을 **무게중심 → 디스크면+H/2** 로 고쳐 +20~24mm 오차 제거(2026-09-23). yaw/xy 는 못 잡음 |
| 2 | **H(물체 높이) 추정** | `robust_top_height` ([lookaround.py:148](utils/nbv/lookaround.py#L148)) | preview p1/max 가 80mm 물체를 119~129mm 로 부풀려 → 윗밴드 허공·flip 뒷면 겹침 유발. 밀도 기준으로 교체해 해결 |
| 3 | **텍스처 특징점 매칭** (RootSIFT) | [utils/nbv/image_match.py](utils/nbv/image_match.py) | 오프라인 벤치에선 flip 을 깼다(inlier 549). **실물 run 에선 실패** — 기준 run: 매칭 3 → inlier 0, 21.6초. 공통 텍스처 면이 없어서 |
| 4 | **footprint yaw 맞춤** | `_flip_yaw_fit`, `scripts/artec/reg_hint_yaw.py` | 옆면 최근접 yaw 스윕. **항상 두 골** → 설계대로 미채택. 기준 run: 207°:7.9mm vs 30°:9.2mm |
| 5 | **윗면 윤곽(contour) 매칭** | `scripts/artec/reg_hint_contour.py` | 방위각별 최대반경 프로파일. **역시 두 골**(251° vs 100°, 11% 차)이고 최선 yaw 가 옆면 겹침을 6.4→7.4mm 로 **악화**. 원인은 알고리즘이 아니라 flip 패스에 목·낮은 옆면이 없어서(점의 71%가 h 40~60mm) |
| 6 | **기하 전역정합 합의** (FGR + FPFH-RANSAC) | `utils/nbv/global_registration.register_consensus` | 두 방법이 **서로 다른 답**(ΔR=170.6° Δt=1772mm) → 불합의로 힌트 유지. fitness 는 0.65/0.78 로 그럴듯해 보이는 게 함정 |
| 7 | **학습기반 정합** (GeoTransformer, Predator) | `scripts/artec/reg_benchmark`, env `reg-dl` | 이 물체에선 **고전 FPFH/FGR 가 더 정확·빠름**. 투자 대비 효과 없었음 |
| 8 | **SDK GlobalRegistration** (후처리) | `mms_artec/system.py` | 힌트가 있으면 자동 skip(힌트가 authoritative). 강제로 돌리면 35mm/5° 를 **무조건** 움직이고 힌트 오차를 못 고침 |

### 2.3 결론과 다음 수

**정합 알고리즘이 아니라 데이터가 부족한 게 원인이었다.** 두 패스가 공유하는 면이
기하적으로도(목·낮은 옆면) 텍스처적으로도(라벨) 없었다.

### ⚠ 윤곽 매칭 — 한때 유망해 보였으나 **철회** (2026-09-23)

처음엔 기준 run 에서 윤곽 매칭이 "유일 최솟값 294°(두 번째보다 43% 낮음), 옆면 겹침
12.1→8.8mm 개선" 을 냈다고 봤다. **틀렸다.** `reg_hint_contour.py` 에 버그가 둘 있었다:

1. **H 를 파이프라인과 다르게 썼다** — 스크립트는 preview 둘의 최댓값(75mm), 파이프라인은
   master 메시까지 포함한 최댓값(95mm). 그래서 flip 을 20mm 낮게 놓았다. 위에서 본 그림은
   멀쩡한데 3D 로 열면 두 패스가 **세로로 어긋나** 있었다(flip 이 디스크 아래 −12mm 까지).
2. **교차확인 지표가 그 어긋남을 숨겼다** — flip 점을 master 의 높이대로 걸러서 쟀기 때문에
   "맞는 부분만" 골라 재게 됐다. 진짜 15.7mm 인 배치를 8.8mm 로 보고했다.

둘 다 고친 뒤 같은 데이터를 다시 재면(양방향 최근접 중앙값):

```
yaw 0(힌트)  flip→master 20.6mm · master→flip 17.9mm
yaw 294°     flip→master 14.2mm · master→flip 20.9mm   ← 한쪽만 좋아지고 반대쪽은 나빠짐
```
즉 **윤곽이 고른 yaw 는 실제 겹침을 개선하지 않는다.**

**교차 검증(ICP를 심판으로):** 힌트에서 yaw 를 10° 간격으로 바꿔 놓고 각각 ICP 를 끝까지
돌렸더니 **모든 yaw 가 같은 품질로 수렴했다**(fitness 0.954~0.956, RMSE 5.3~5.6mm).
1위와 2위 차이가 0%다 — 이 물체·이 데이터에서는 **yaw 가 기하적으로 정해지지 않는다**는
직접 증거다. run_102221 에서 내린 결론과 같다.

> 교훈: 정합 지표를 만들 때 **한쪽 점군을 다른 쪽의 범위로 거르지 말 것.** 어긋난 배치일수록
> 점수가 좋아진다. 항상 **양방향**으로 재고, 3D 로 한 번 열어 볼 것.

### 참고: H(물체 높이) 추정도 못 믿는다
같은 run 에서 preview 75mm · master 점군 75mm · master 메시 95mm 인데, 데이터에 맞춰
H 를 훑으면 115mm 이상에서 겹침이 계속 좋아진다. H 는 flip 피벗을 직접 정하므로
**yaw 문제와 별개로 세로 오차를 만든다.** 둘을 같이 풀어야 한다.

- [ ] **물체 배치 규약.** 뒤집었을 때도 **텍스처가 있는 면이 보이도록** 놓게 한다
      (작업 지시 + 화면 안내). 이것만으로 3번이 살아난다.
- [ ] **턴테이블 프레임 재캘리브.** `config/calibration/turntable_frame.yaml` 의 `T_B_F0` 가
      **2026-04-23**(Artec 이전, PhoXi 시절) 값이다. 힌트·NBV·flip 이 전부 이 축에 의존한다.
      → `scripts/artec/verify_turntable_axis.py` 로 먼저 오차를 재고, 크면 재캘리브.
- [ ] **마커/지그 검토** (근본 해결). 턴테이블 원판에 비대칭 마커를 두면 yaw 3 자유도가
      관측 가능해진다 — 위 세 가지로 안 되면 이쪽이 정답일 가능성이 높다.
- [ ] 실패해도 **사람이 이어받을 수 있게**: 지금처럼 힌트 배치로 `aligned.sproj` 를 남기고
      Studio 에서 Align → GR 하게 한다 (§4). 이건 이미 된다.

---

## 3. 🟡 정합을 파이프라인에 넣기 (자동화)

지금 `_flip_global_refine` 이 **텍스처 → yaw맞춤 → greg합의 → 힌트** 순으로 시도하고
전부 실패하면 힌트를 쓴다. 구조는 맞다. 채울 것:

- [ ] **채택/미채택 근거를 events.jsonl 에 남긴다** (지금은 콘솔 로그만). 후처리 스크립트가
      "이 run 은 어느 방법으로 정합됐나" 를 읽을 수 있어야 한다.
- [ ] **정합 품질 지표를 산출물에 남긴다** — 두 패스 옆면 최근접 중앙값/p90 을 재서
      `README.txt` 와 events 에. 지금은 "잘 됐나" 를 사람이 눈으로 본다.
- [ ] 품질이 기준 미달이면 **재시도 또는 사용자 확인**으로 분기 (지금은 그냥 진행).
- [ ] 오프라인 재실험 스크립트들(`scripts/artec/reg_*.py`)을 **하나의 진입점**으로 묶기.
      지금 5개가 비슷한 로딩 코드를 각자 갖고 있다.

---

## 4. 🟡 후처리를 파이프라인에 넣기

지금 파이프라인은 SerialReg → (GR) → OutliersRemoval → **PoissonFusion** →
SmallObjectsFilter → (Simplify) → (Texturize) 까지 돌고 `final/final.sproj` + `final.obj`
를 남긴다. 텍스처와 구멍 메우기는 **Artec Studio 수작업**이다.

알아 둘 것 (실측):
- **SDK 알고리즘 순서**: OutliersRemoval 은 Fusion **전**(0x80010203), SmallObjectsFilter 는
  Fusion **뒤**(메시 입력, 전에 걸면 0x80010201). `docs/6_postprocess.md` §2
- **Texturize 는 CPU 단일코어** — 1900프레임에 15~20분. Studio(GPU)는 1분. 그래서 기본 끔
- **PoissonFusion 구멍 메우기**: `fillType` ALL 은 열린 끝을 **풍선처럼 부풀린다**.
  ByRadius(`maxHoleRadius`, 기본 5mm)가 안전
- watertight 가 안 되는 진짜 이유는 융합 설정이 아니라 **커버리지**(바닥면·라벨면 미수집)

- [ ] Texturize 를 GPU 로 할 방법 찾기 (SDK 옵션 없음 → Studio CLI/배치 가능한지 조사)
- [ ] 구멍 반경별 메우기 정책을 자동으로 (지금 `fusion="poisson"` 고정)
- [ ] 최종 메시 **품질 검사 자동화** — boundary edge 수, 조각 수, watertight 여부를
      `final/` 옆에 리포트로. `scripts/artec/reg_hint_fusion.py` 가 이미 이걸 잰다
- [ ] 실패 시 `aligned.sproj` + `README.txt` 로 사람에게 넘기는 경로 유지(이미 작동)

---

## 5. 🟢 작은 것들 (해두면 좋음)

- [ ] `nbv` 가 기준 run 에서 0.8초 만에 "도달 가능한 자세 없음" 으로 끝났다. gap 12개 중
      8개 아랫면(flip 몫) · 1개 원판 근처(flip 몫) · 나머지 3개는 **물체근접 42회 기각**.
      근접 판정이 과한지 확인할 것 (`4_nbv.md`)
- [ ] `H` 추정이 아직 크다 — 기준 run 에서 preview 75mm / master 메시 95mm 인데 실제 ~55mm.
      master 메시 쪽 H 가 노이즈에 부푼다. `_flip_pivots_B` 가 셋 중 **최댓값**을 쓰므로 영향
- [ ] 뒤집기 2회 계획인데 1회만 수행되고 끝난다 (`reason: 정의된 뒤집기 2회 소진`) — 의도 확인
- [ ] `README.md` 의 산출물 경로가 옛 배치 그대로다 (2026-09-23 에 `output/<RUN>/` 로 통일).
      `docs/3_lookaround.md` §0 목록으로 교체할 것

---

## 6. 어디에 무엇이 있나

### 코드
| 파일 | 역할 |
|---|---|
| [main_artec.py](main_artec.py) | 진입점. `RUN_TS`·`RUN_DIR`·후처리 설정·CLI |
| [mms_artec/system.py](mms_artec/system.py) | `artec_process` — 스캔 후 SDK 후처리 체인·export |
| [mms_artec/nbv/artec_multipass_scan_session.py](mms_artec/nbv/artec_multipass_scan_session.py) | **핵심.** 패스 흐름·힌트·정합(`_flip_global_refine`)·병합 게이트·덤프 |
| [mms_artec/nbv/artec_streaming_scan_session.py](mms_artec/nbv/artec_streaming_scan_session.py) | 밴드 1개 캡처(회전 중 거리추종·lost 판정·디버그 이미지) |
| [utils/nbv/lookaround.py](utils/nbv/lookaround.py) | preview 점 수집·밴드 계획·`robust_top_height` |
| [utils/nbv/global_registration.py](utils/nbv/global_registration.py) | FGR/FPFH-RANSAC 합의 (`register_consensus`) |
| [utils/nbv/image_match.py](utils/nbv/image_match.py) | 텍스처 특징점 정합 (RootSIFT) |
| [utils/run_paths.py](utils/run_paths.py) | **산출물 경로 규칙 한 곳** (2026-09-23) |

### 오프라인 도구 (`scripts/artec/`)
| 스크립트 | 무엇 |
|---|---|
| `reg_hint_test.py` | 설계 힌트 vs run 실제 배치 비교 (`--H-mm` 민감도) |
| `reg_hint_post.py` | raw sproj 의 flip 변환만 갈아끼우고 SDK 체인 재실행 |
| `reg_hint_fusion.py` | 융합 변형별 비교 (구멍·조각 통계) |
| `reg_hint_yaw.py` | yaw 스윕 (옆면 최근접) |
| `reg_hint_contour.py` | yaw 스윕 (윗면 윤곽 프로파일) |
| `reg_offline.py` | 방법별 오프라인 재정합 |
| `lost_report.py` | events.jsonl 집계 (lost·밴드전환·복구) |
| `live_range_view.py` | 디버그 이미지 라이브 뷰어 (+녹화) |
| `tidy_output.py` | 옛 배치 산출물을 run 폴더 규칙으로 이동 |
| `show_obj.py` | 텍스처 없는 OBJ 창으로 보기 |

### 산출물 (`output/<RUN_TS>/`)
```
README.txt               어느 파일을 Studio 로 열고 무엇을 하나 (자동 생성)
aligned/aligned.sproj    ★ 파이프라인 변환 적용·SDK 정합 전 — Studio 로 여는 것
final/final.sproj        후처리 결과(융합 메시 포함) · final.obj
scan_dumps/*.npz         IScan 별 점군 + 적용 변환 (오프라인 재정합 원본)
events.jsonl · run.log · timeline.csv
debug/{preview,lookaround,nbv,flip}/   거리 이미지 · cam/ 카메라 · range.avi
```

### 반드시 읽을 문서
- `docs/5_flip.md` — 힌트 수식·피벗·yaw 문제 (이 TODO 의 §2 배경 전부)
- `docs/6_postprocess.md` §4 — run 산출물과 Studio 수작업 절차
- `docs/troubleshooting.md` — 실물 경로에서 반복된 함정
- `docs/3_lookaround.md` T10 — `ignore_registration_errors=True` 가 **필수**인 이유
  (False 로 바꾸면 밴드가 1초 만에 죽는다. 건드리지 말 것)
