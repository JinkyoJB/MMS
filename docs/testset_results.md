# 테스트셋 9종 일반성 검증 (Isaac Sim)

> 웹판: https://claude.ai/code/artifact/ec1c4f39-e223-4e0e-a595-636e8fe4b06c
> 재생성: `scripts/sim/testset_sweep.sh` → `eval_vs_gt.py` → `build_results_page.py`

120프레임/rev, Phase 1(측면 전회전) → 2(부족면 NBV) → 3(flip 바닥면) 전체 실행.
스윕이 코드 결함 **7건**을 드러냈고, 모두 수정한 상태로 9종을 한 번에 돌린 결과다.

**평균 F@1mm 81.7% · 최저 64.4% · 평균 Chamfer 0.77mm**

## 최종 품질 — GT 대조 (표준 지표)

| 물체 | 완전성@1mm | 정확도@1mm | F@1mm | F@2mm | Chamfer | flip el | 팽창 |
|---|---|---|---|---|---|---|---|
| drug bottle | 99.8% | 99.9% | **99.9%** | 100.0% | 0.24mm | 70° | +0.3mm |
| povidone iodine | 98.5% | 99.9% | **99.2%** | 99.7% | 0.25mm | 70° | +0.3mm |
| mustard | 95.1% | 98.9% | **97.0%** | 99.1% | 0.39mm | 60° | +0.4mm |
| spray can | 88.2% | 99.7% | **93.6%** | 94.8% | 0.79mm | 60° | +1.3mm |
| alarm clock | 69.1% | 80.5% | **74.4%** | 91.4% | 0.91mm | 60° | +4.3mm |
| protein drink | 70.7% | 75.2% | **72.9%** | 94.3% | 0.74mm | 70° | +2.0mm |
| mug | 55.1% | 92.6% | **69.1%** | 73.0% | 1.40mm | 70° | +3.3mm |
| hand drill | 61.5% | 67.9% | **64.5%** | 90.8% | 1.14mm | 60° | +0.4mm |
| laundry detergent | 63.1% | 65.7% | **64.4%** | 91.7% | 1.11mm | 60° | +0.9mm |

- 작은 회전체(drug bottle·povidone)는 99% 대, mustard 97.0%.
- **mug**: 완전성 55% / 정확도 93% — 얻은 것은 정확하되 오목한 안벽이 남았다(원래부터의 과제).
- 남은 표적: 자가폐색이 심한 **hand drill**, 최대 크기(293mm) **laundry detergent** — 둘 다 64% 대.
- flip 관측 el 이 60~70° 로 잡히며 바닥면이 안정적으로 취득된다(아래 결함 5·6).

## 스윕이 찾아낸 결함 7건 (모두 수정)

| # | 결함 | 수정 | 근거 |
|---|---|---|---|
| 1 | 밴드 1개 실패 시 Phase 2·3 전체 중단 | 실패 밴드만 건너뛰고 계속 | hand drill·spray can 중단 → 완주 |
| 2 | 충돌 게이트가 밴드 경로에 누락 | legacy 와 동일 게이트 (충돌 시 다음 az) | az0 은 4mm 차 자가충돌, az30 통과 |
| 3 | 경계 지표를 "낮을수록 좋다"로 역해석 | GT 대조층 신설 + 수렴을 신규 점유복셀로 교체, gap 단위 dry 회계 | drill compl 37.9→52.7% 인데 경계는 519→800 |
| 4 | 정합 게이트가 평행이동 오류를 통과 | 합집합 bbox 팽창 검사 (sim·real 양쪽) | laundry 45.1→58.5%, protein 56.5→68.9%, 오탐 0건 |
| 5 | flip 후 관측 el 을 측면뷰 30° 로 고정 | 새로 드러난 면의 법선에서 el 산출 | mug 바닥 중앙 미취득 100%→0% |
| 6 | 후보 el 이 `[70°, 30°]` 로 끊김 | 예측·실행이 **같은 사다리** 사용(`_flip_view_els`) | 60° 를 건너뛰어 30° 추락 → 최저 57.7→64.4% |
| 7 | flip 각 판정이 종횡비(대리 지표) | **도달성**으로 판정 + 종횡비는 스캔 점군에서 | 사각지대·임계 경계(2.01) 문제 동시 해소 |

> **6·7 이 이번 라운드의 핵심.** 90° 눕히기는 "키 큰 물체는 바닥을 내려다볼 수 없다"는
> 도달 한계를 우회하려 넣은 것인데, 사다리를 고치자 293mm 세제까지 el 60° 로 바닥을 본다 —
> 한계로 보이던 것은 후보 목록이 60° 를 건너뛴 탓이었다. 판정을 도달성으로 바꾼 덕에 드러났다.
> 이번 9종은 모두 180° 하나로 충분했고, 90° 는 정말 도달 불가한 형상의 안전판으로 남는다.

## 실행 지표 (경계/gap 은 실행 중 관측치 — 판단은 위 GT 표로)

| 물체 | 크기(H×D) | 밴드 | Phase 1 | 패치 | 수렴 | Phase 2 | flip |
|---|---|---|---|---|---|---|---|
| mustard | 192×96mm | 1 | 428mm/8 | 4 | 후보소진 | 273mm/5 | 180° |
| hand drill | 187×162mm | 2 | 547mm/11 | 8 | 상한 | 717mm/15 | 180° |
| alarm clock | 174×132mm | 1 | 600mm/11 | 8 | 상한 | 478mm/9 | 180° |
| spray can | 207×68mm | 3 | 270mm/5 | 4 | 조기(복셀) | 222mm/4 | 180° |
| mug | 77×126mm | 1 | 559mm/11 | 8 | 상한 | 547mm/10 | 180° |
| laundry detergent | 293×194mm | 2 | 864mm/16 | 8 | 상한 | 624mm/11 | 180° |
| drug bottle | 66×46mm | 1 | 192mm/4 | 4 | 조기(복셀) | 150mm/3 | 180° |
| povidone iodine | 90×34mm | 1 | 133mm/3 | 2 | 후보소진 | 105mm/2 | 180° |
| protein drink | 57×144mm | 1 | 295mm/5 | 6 | 후보소진 | 302mm/6 | 180° |

## 물체별 결과 메시

### mustard
F@1mm **97.0%** · 완전성 95.1% · Chamfer 0.39mm · 169,129면 · bbox 58×95×192mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0001_mustard__정면.jpg) | ![측면](figures/testset/0001_mustard__측면.jpg) | ![윗면](figures/testset/0001_mustard__윗면.jpg) | ![아랫면](figures/testset/0001_mustard__아랫면.jpg) |

### hand drill
F@1mm **64.5%** · 완전성 61.5% · Chamfer 1.14mm · 209,818면 · bbox 162×51×187mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0002_hand_drill__정면.jpg) | ![측면](figures/testset/0002_hand_drill__측면.jpg) | ![윗면](figures/testset/0002_hand_drill__윗면.jpg) | ![아랫면](figures/testset/0002_hand_drill__아랫면.jpg) |

### alarm clock
F@1mm **74.4%** · 완전성 69.1% · Chamfer 0.91mm · 307,473면 · bbox 66×131×173mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0086_alarm_clock__정면.jpg) | ![측면](figures/testset/0086_alarm_clock__측면.jpg) | ![윗면](figures/testset/0086_alarm_clock__윗면.jpg) | ![아랫면](figures/testset/0086_alarm_clock__아랫면.jpg) |

### spray can
F@1mm **93.6%** · 완전성 88.2% · Chamfer 0.79mm · 139,728면 · bbox 68×67×208mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0101_spray_can__정면.jpg) | ![측면](figures/testset/0101_spray_can__측면.jpg) | ![윗면](figures/testset/0101_spray_can__윗면.jpg) | ![아랫면](figures/testset/0101_spray_can__아랫면.jpg) |

### mug
F@1mm **69.1%** · 완전성 55.1% · Chamfer 1.40mm · 324,820면 · bbox 97×126×80mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0146_mug__정면.jpg) | ![측면](figures/testset/0146_mug__측면.jpg) | ![윗면](figures/testset/0146_mug__윗면.jpg) | ![아랫면](figures/testset/0146_mug__아랫면.jpg) |

### laundry detergent
F@1mm **64.4%** · 완전성 63.1% · Chamfer 1.11mm · 200,638면 · bbox 193×103×293mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0154_laundry_detergent__정면.jpg) | ![측면](figures/testset/0154_laundry_detergent__측면.jpg) | ![윗면](figures/testset/0154_laundry_detergent__윗면.jpg) | ![아랫면](figures/testset/0154_laundry_detergent__아랫면.jpg) |

### drug bottle
F@1mm **99.9%** · 완전성 99.8% · Chamfer 0.24mm · 355,146면 · bbox 45×45×66mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0159_drug_bottle__정면.jpg) | ![측면](figures/testset/0159_drug_bottle__측면.jpg) | ![윗면](figures/testset/0159_drug_bottle__윗면.jpg) | ![아랫면](figures/testset/0159_drug_bottle__아랫면.jpg) |

### povidone iodine
F@1mm **99.2%** · 완전성 98.5% · Chamfer 0.25mm · 159,431면 · bbox 33×35×90mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0160_povidone_iodine__정면.jpg) | ![측면](figures/testset/0160_povidone_iodine__측면.jpg) | ![윗면](figures/testset/0160_povidone_iodine__윗면.jpg) | ![아랫면](figures/testset/0160_povidone_iodine__아랫면.jpg) |

### protein drink
F@1mm **72.9%** · 완전성 70.7% · Chamfer 0.74mm · 182,721면 · bbox 143×59×56mm

| 정면 | 측면 | 윗면 | 아랫면 |
|---|---|---|---|
| ![정면](figures/testset/0263_protein_drink__정면.jpg) | ![측면](figures/testset/0263_protein_drink__측면.jpg) | ![윗면](figures/testset/0263_protein_drink__윗면.jpg) | ![아랫면](figures/testset/0263_protein_drink__아랫면.jpg) |

---
렌더: Poisson depth 8, 정점색 제거 + 균일 재질. 아랫면 뷰가 Phase 3 flip 성패를 보여준다. 원본 OBJ `scripts/sim/log/testset_sweep/render/`, GT `scripts/sim/log/gt/`.