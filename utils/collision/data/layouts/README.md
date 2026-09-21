# 충돌 환경 캐시 — 레이아웃 변형

> 여기는 **셀 환경(`cell_env.npz`) 변형본** 보관소다. 캐시 파일이 둘인데 뭐가 뭔지
> 모르겠으면 먼저 → `../README.md`

`cell_env.npz` 는 코드가 읽는 **활성본** 하나뿐이라, 변형은 여기 쌓아두고
`scripts/collision/use_layout.py` 로 갈아끼운다. 배경은 `docs/collision.md` §6.
새로 굽는 법은 `scripts/collision/bake_layout.py`.

⚠ 여기 `*.npz` 는 `.gitignore` 대상이라 **git 으로 따라가지 않는다.** 다른 PC 로
옮길 때는 파일을 직접 복사할 것.

| 별칭 | 출처 USD | 루트 prim | 로봇 base(world) | 점 수 | 턴테이블 |
|---|---|---|---|---|---|
| **`v2_real_260917`** ★활성 | `frame_xarm7_spider_turntable/v2_real_260917.usd` | `/World/Frame/frame_structure` + `/World/frame` | `[0.406, 0, 1.407]` | 2,350,544 | ✅ v3 CAD 조립체 |
| `v3_layout_sim` | `frame_xarm7_spider_turntable_v2/v3_scene.usd` | `/World/frame` | `[0.365, 0, 1.500]` | 1,301,122 | ✅ 포함 |
| `v2_layout_real` | `frame_xarm7_spider_turntable/v2_real.usd` (**이 머신에 없음**) | `/World/Frame/frame_structure` | `[0.406, 0, 1.407]` | 2,031,396 | ⚠ placeholder ×1.6 |

> `v2_layout_real` 이 **현장 실측 반영본**이지만 출처 USD 가 이 머신에 없다(릴리스에도 없음).
> 그래서 `v2_real_260917` 을 만들어 대체했다 — 두 점군은 양방향 최근접 **중앙값 0.78mm ·
> 5mm 이내 98.4%** 로 사실상 같은 셀이다. 갈리는 곳은 (a) 2026-09-17 현장보정 부재 2개,
> (b) 턴테이블(실측본은 v2 placeholder 를 1.6배 확대, 우리 건 v3 CAD 조립체)뿐이다.

## ★ v2_real_260917 — 현행 활성본 (2026-09-17)

**실물 셀 = v2 배치다.** v3 로 바꾸기로 했다가 실제로는 안 바꿨다(2026-09-16 현장 확인).
`scripts/sim/build_scene_v2_real.py` 가 `v2.usd` 위에 오버라이드만 얹어 실물 배치를
재현하고, 이 캐시는 그 씬에서 구운 것이다. **씬과 충돌 캐시가 같은 출처**라 서로
어긋날 수 없다.

```bash
env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python scripts/sim/build_scene_v2_real.py
env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python scripts/sim/export_env_mesh.py \
    --scene "$(...)/frame_xarm7_spider_turntable/v2_real_260917.usd" \
    --root /World/Frame/frame_structure /World/frame --robot /World/xarm7 \
    --out utils/collision/data/layouts/cell_env.v2_real_260917.npz
```

> `--root` 를 **두 개** 준다 — 이 씬은 구조물(`/World/Frame/frame_structure`)과
> 턴테이블(`/World/frame`)이 서로 다른 루트에 있다. 하나만 주면 턴테이블이 통째로
> 빠져 충돌 게이트가 못 본다(실측: 디스크 밑 대역 점 수 19,545 → 178).

배치 근거·검증 수치는 빌더 스크립트 docstring 에 있다(전부 실측 역추적, 오차 0.00mm).

### ⚠ 굽기는 **복셀 다운샘플**이다 — stride 로 솎지 말 것

`--voxel`(기본 2mm)로 줄인다. 예전에는 `--max-pts` 까지 **균일 stride** 로 잘랐는데,
그러면 큰 벽이든 작은 부품이든 같은 비율로 버려서 **작은 부재가 굶는다.**
실측 2026-09-17: 원본 30,153,025점을 stride 15 로 2M 까지 줄이자 턴테이블 원판 상면
점이 2,532 → **163개**가 됐다(8mm SDF 복셀 기준 구멍투성이 = 충돌 게이트가 원판을
제대로 못 봄). 2mm 복셀로 줄이면 같은 크기(2,350,544점)에 **표면 어디든 균일**하다.

    원판 상면 ±4mm 점 :  163 → 1,395     턴테이블 대역 : 21,811 → 77,931

### 남은 확인거리

턴테이블 **디스크 아래 몸체**가 v3 CAD 와 같은 물건인지 실물에서 확인 안 됐다.
디스크 반경은 v3 CAD 119mm 로 실물 rim 실측(121.52mm)에 가장 가깝지만, 몸체 형상은
셀 CAD 를 그대로 믿고 있는 상태다.

## v3_layout_sim — CAD 원본 (2026-04 이전 셀)

`build_scene_v3.py` 가 만든 sim 씬 그대로. 부재 이름이 살아 있고 턴테이블이
`/World/frame/turntable_disc` 로 셀 안에 있다. **실물 셀과는 다르다** — 이게 §6 의 출발점.

## v2_layout_real — 2026-09-15 현장 실측 반영

`v2.usd` 를 Isaac Sim GUI 에서 실물에 맞춰 편집한 `v2_real.usd` 기준.

- 부재 189 → 78 개 (실물에 없는 것 삭제)
- 로봇 base X 0.538 → 0.406 (−132mm)

> ⚠ **턴테이블이 들어 있지 않다.** v2 계열은 턴테이블이
> `/World/ScanTarget/turntable_demo` 라 `frame_structure` 루트 밖이다.
> 실측: 턴테이블 대역(r<0.13, base z 0.7~1.0) 점 수가
> `v3_layout_sim` 102,776 → `v2_layout_real` **2,885**.
>
> 메시 게이트가 턴테이블을 **못 본다**(미탐). 캡슐 world 가 `T_B_F0` 로 따로 넣지만
> 그건 nbv 후보 사전 필터일 뿐 최종 게이트가 아니다 (`docs/collision.md` §6.6).
>
> **고쳤다** — `v2_real_260917`(위)이 턴테이블 조립체를 포함한다. 아래는 옛 수동 절차다.
>
> **(옛) 고치는 법** — GUI 에서 `turntable_demo` 를 `frame_structure` 아래로 옮기고 재추출:
> ```powershell
> $ISAAC = "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe"
> & $ISAAC scripts\sim\export_env_mesh.py `
>     --scene C:\dev\2_3Dassets\frame_xarm7_spider_turntable\v2_real.usd `
>     --root /World/Frame/frame_structure `
>     --out utils\collision\data\layouts\cell_env.v2_layout_real.npz
> & $ISAAC ..\..\scripts\collision\use_layout.py v2_layout_real
> ```

## 새 변형을 추가할 때

1. `export_env_mesh.py --out utils/collision/data/layouts/cell_env.<별칭>.npz`
2. `python scripts/collision/use_layout.py` 로 목록·범위 확인
3. `python scripts/collision/use_layout.py <별칭>` 로 활성화

⚠ 루트 prim 은 씬마다 다르다 — v3 는 `/World/frame`(소문자), v2 계열은
`/World/Frame/frame_structure`(대문자 + 한 단계 아래). v2 에서 `/World/Frame` 을
그대로 주면 `Prototypes` 스코프의 원본 메시까지 빨려들어가 **유령 장애물**이 생긴다.
