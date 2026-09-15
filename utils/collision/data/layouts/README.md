# 충돌 환경 캐시 — 레이아웃 변형

`cell_env.npz` 는 코드가 읽는 **활성본** 하나뿐이라, 변형은 여기 쌓아두고
`scripts/collision/use_layout.py` 로 갈아끼운다. 배경은 `docs/4_collision.md` §6.

| 별칭 | 출처 USD | 루트 prim | 로봇 base(world) | 점 수 | 턴테이블 |
|---|---|---|---|---|---|
| `v3_layout_sim` | `frame_xarm7_spider_turntable_v2/v3_scene.usd` | `/World/frame` | `[0.365, 0, 1.500]` | 1,301,122 | ✅ 포함 |
| `v2_layout_real` | `frame_xarm7_spider_turntable/v2_real.usd` | `/World/Frame/frame_structure` | `[0.406, 0, 1.407]` | 2,002,869 | ❌ **빠짐** |

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
> 그건 Phase 2 후보 사전 필터일 뿐 최종 게이트가 아니다 (`docs/4_collision.md` §6.6).
>
> **고치는 법** — GUI 에서 `turntable_demo` 를 `frame_structure` 아래로 옮기고 재추출:
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
