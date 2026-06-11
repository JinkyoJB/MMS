# Registration Benchmark — 6개 정합 방법 비교

0/90/180° ScanSession 3개를 하나의 객체로 정합하는 6가지 방법을 같은 지표로 비교.

## 데이터
- `output/scan_raw/20260520_153357/master.sproj` — 3 ScanSession(각 222프레임, textured).
- Artec Studio `.a3d`(LevelDB형)는 SDK(`max project version=4`)가 못 읽음 → 반드시
  SDK가 만든 `.sproj`(`save_raw_scan.py` 산출물)를 사용.

## ⚠ 핵심 발견 (2026-06-10)
이 데이터의 3 세션은 **raw SLAM 프레임에서 이미 거의 정렬**돼 있다(상대자세 ≈ identity
+ 드리프트 7~13°). `meta.npz`의 recorded hint(90/180°)는 이 데이터엔 **오답**
(적용 시 오히려 어긋남). → 실데이터는 "큰 flip 복원" 난이도를 못 담는다.

그래서 두 트랙으로 평가한다:
- **real**: 실제 세션. GT = identity 에서 다단 ICP 정합(best). 이미 쉬워서 변별력 낮음.
- **synthetic**: session0 에 **알려진 회전(90/180°) + 부분 crop**을 적용해 저중첩 flip 을
  인위적으로 생성. 정확한 GT 로 ΔR/Δt 엄밀 측정. ← **진짜 변별 트랙**.

## 방법
| # | 방법 | 트랙 | 비고 |
|---|---|---|---|
| 1 | FPFH + RANSAC | real+syn | Open3D |
| 2 | Fast Global Registration | real+syn | Open3D |
| 3 | Artec GlobalRegistration | **real only** | SDK IModel 필요. 바인딩이 numpy→IFrameMesh 미지원이라 합성 점군 주입 불가 |
| 4 | Predator | real+syn | torch (reg-dl env) |
| 5 | GeoTransformer | real+syn | torch (reg-dl env) |
| 6 | 이미지 특징점 매칭(SIFT) | **real only** | 프레임 RGB+uv 필요 |

baseline: `identity`, `icp_identity`, `gt`(syn oracle), `meta_hint`(real, 오답 확인용).

## 지표
- `fit@{3,5}mm`, `rmse@5mm`, `chamfer` — overlap 품질(저중첩에선 GT라도 낮음 주의)
- `rot°` — 추정 회전각, `gtRot/gtT` — GT 대비 ΔR/Δt (**저중첩 정확도 핵심**)

## 실행 (mms-env, py3.11)
```
# 캐시 생성(최초 1회)
python -m scripts.artec.reg_benchmark.common
# 벤치마크
python -m scripts.artec.reg_benchmark.run_benchmark --track both
python -m scripts.artec.reg_benchmark.run_benchmark --track synthetic --methods fpfh_ransac,fgr
# 오프라인 렌더(헤드리스 PIL — filament EGL 불가 환경)
python -m scripts.artec.reg_benchmark._render
```
결과: `output/reg_benchmark/results.csv` + 병합 PLY + `renders/*.png`.

## 잠정 결과 (방법 1·2·baseline)
- real: meta_hint=오답(gtRot 90/177°), identity/icp/fpfh/fgr 모두 수렴(gtRot≈0.3°).
- syn full: fpfh·fgr 완벽(gtRot 0°), icp-from-identity는 대회전 실패.
- **syn partial(~40% 중첩): fpfh_ransac만 성공(gtRot 3°), fgr 실패(75/97°)** ← 학습기반 시험 지점.

## 학습기반(4·5) 셋업 → `SETUP_learned.md`
