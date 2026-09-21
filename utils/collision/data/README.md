# 충돌 캐시 — 이 폴더에 뭐가 있나

충돌 검사는 **"로봇"과 "환경"** 두 점군을 비교한다. 그래서 npz 도 두 개다.
둘은 **역할도 좌표계도 다르다.**

```
       xarm7_spider_links.npz            cell_env.npz
       = 움직이는 쪽 (로봇)         vs    = 가만히 있는 쪽 (셀)
       링크 로컬 좌표                      base 좌표
       관절각으로 FK 해서 배치             그대로 씀
```

| | `xarm7_spider_links.npz` | `cell_env.npz` |
|---|---|---|
| **담긴 것** | 로봇 링크 7개 + 툴(스캐너) 표면점 | 벽·상판·저울·툴스탠드·**턴테이블** 표면점 |
| **좌표계** | **각 링크 로컬** (link7 기준 tool) | **로봇 base** |
| **쓸 때** | 관절각 `q` 로 FK 해서 base 로 배치 | 변환 없이 바로 |
| **크기** | 2.2 MB · 링크당 2~3.5만 점 | 40 MB · 235만 점 |
| **언제 바뀌나** | 로봇/툴 하드웨어가 바뀔 때 (거의 없음) | 셀을 개조하거나 **턴테이블이 틀어질 때** |
| **만드는 것** | `scripts/sim/export_link_meshes.py` | `scripts/collision/bake_layout.py` |

둘 다 `utils/collision/collision_model.py` 가 읽는다. 링크 점군은 두 군데에 쓰인다 —
셀과의 **환경충돌**, 그리고 링크끼리의 **자가충돌**(팔이 자기 몸을 치는 것).

다만 **전부를 다 검사하지는 않는다.** 같은 파일 상단에 이유와 함께 적혀 있다:

| 상수 | 값 | 왜 |
|---|---|---|
| `ENV_LINKS` | `3,4,5,6,7` | link1·2 는 천장 마운트에 상시 근접 → 넣으면 늘 충돌이다 |
| `SELF_PAIRS` | `tool↔1~5`, `link6↔1~3` | link6·7 은 툴이 붙은 인접 링크라 **항상** 닿아 있다 |

그래서 "링크 npz 에 link1 이 있는데 왜 벽 충돌을 안 잡지?" 는 버그가 아니라 설계다.

---

## 파일 목록

| 파일 | 설명 |
|---|---|
| `xarm7_spider_links.npz` | 로봇 링크·툴 메시 (위 표 참조) |
| `cell_env.npz` | **활성** 셀 환경. 코드가 읽는 것은 이 파일 **하나**다 |
| `cell_env.meta.yaml` | 활성본이 **어느 턴테이블 자리로** 구워졌나 (아래 참조) |
| `ACTIVE_LAYOUT.txt` | 활성본이 어느 별칭인지 (예: `v2_real_260917`) |
| `layouts/` | 셀 환경 **변형본** 보관. `use_layout.py` 로 갈아끼운다 → `layouts/README.md` |

> ⚠ `layouts/*.npz` 는 `.gitignore(*.npz)` 대상이라 **git 으로 따라가지 않는다.**
> 활성본(`cell_env.npz`)과 링크 npz 는 이미 추적 중이라 예외적으로 커밋된다.
> 다른 PC 로 레이아웃을 옮길 때는 npz 파일을 **직접 복사**할 것.

---

## `cell_env.meta.yaml` 이 왜 필요한가

`cell_env.npz` 는 그냥 점 뭉치라 **"어느 점이 턴테이블인지"** 를 스스로 말해주지
않는다. 그런데 턴테이블을 재캘리브하면 그 점들만 새 자리로 옮겨야 한다
(테이블·벽은 안 움직였으니 그대로 둬야 한다).

`T_B_F0_at_bake` 가 **캐시를 구울 때의 턴테이블 자리**를 기록해서, 미세조정
(`scripts/collision/rebuild_from_calib.py`, `calibrate.py` 4단계)이 "어디서 어디로"
옮길지 알 수 있게 한다. `bake_layout.py` 가 구울 때마다 자동으로 적는다.

meta 가 없으면 캐시에서 원판을 찾아 추정하는데 정확도가 떨어진다
(실측: 60mm 어긋남을 53mm 로 추정). 그래서 미세조정은 고친 뒤 결과를 **다시 재서
나아지지 않으면 되돌린다.**

---

## 언제 무엇을 다시 만드나

| 상황 | 할 일 |
|---|---|
| 턴테이블이 틀어짐 / 재캘리브함 | `calibrate.py` 4단계가 자동 (장비·Isaac 불요) |
| 셀 형상이 바뀜 (테이블·벽·마운트·CAD) | `scripts/collision/bake_layout.py` (Isaac + USD 자산 필요) |
| 로봇이나 툴 하드웨어가 바뀜 | `scripts/sim/export_link_meshes.py` |
| 지금 맞는지 확인 | `python scripts/artec/check_calibration.py` |

배경과 설계 근거는 `docs/collision.md`, 캘리브와의 연결은 `docs/1_calibration.md` Part 3.
