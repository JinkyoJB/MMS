"""mms_paths — 자산(USD 등) 루트를 리포 위치와 무관하게 해석한다.

왜 필요한가
-----------
씬 USD·STEP 등 대용량 자산은 git 에 넣지 않으므로 리포 밖에 있다. 예전에는 개발
머신의 절대경로를 소스에 박아 뒀는데, 다른 사람이 클론하면 전부 깨진다.
이 모듈이 **알려진 배치들을 순서대로 탐색**해 실제 존재하는 곳을 고른다.

탐색 순서
---------
1. 환경변수 ``MMS_ASSET_ROOT``            ← 어디에 두든 이걸로 지정 가능
2. ``<repo>/../../2_데이터/2_3Dassets``    ← 인수인계 폴더 배치
                                             (A1_.../1_코드/MMS ↔ A1_.../2_데이터)
3. ``<repo>/../2_3Dassets``               ← 원본 개발 배치
                                             (1_MMS/7_MMS_framework ↔ 1_MMS/2_3Dassets)
4. ``<repo>/2_3Dassets``                  ← 리포 안에 직접 둔 경우
5. 구 개발 머신 절대경로                    ← 하위호환

사용
----
    from mms_paths import asset
    SCENE = asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")

없으면 import 시점에 죽지 않고 후보 1순위를 돌려준다(모듈 상수 정의를 깨지 않기
위해서). 대신 경고를 한 번 찍으므로, 파일 열기 실패 시 원인을 바로 알 수 있다.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parent

#: 자산 루트 후보 (앞에서부터 존재하는 것을 채택)
_CANDIDATES = (
    _REPO.parent.parent / "2_데이터" / "2_3Dassets",   # 인수인계 배치
    _REPO.parent / "2_3Dassets",                       # 원본 개발 배치
    _REPO / "2_3Dassets",                              # 리포 내부
    Path("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets"),  # legacy
)

_warned = False


def asset_root() -> Path:
    """자산 루트 디렉터리. 존재하는 첫 후보를 돌려준다."""
    global _warned

    env = os.environ.get("MMS_ASSET_ROOT")
    if env:
        return Path(env).expanduser().resolve()

    for cand in _CANDIDATES:
        if cand.is_dir():
            return cand.resolve()

    if not _warned:
        _warned = True
        print(
            "[mms_paths] ⚠ 자산 루트(2_3Dassets)를 찾지 못했다. 다음을 순서대로 확인했다:\n"
            + "\n".join(f"    - {c}" for c in _CANDIDATES)
            + "\n  → 환경변수로 지정하라:  export MMS_ASSET_ROOT=/경로/2_3Dassets",
            file=sys.stderr,
        )
    return _CANDIDATES[0]


def asset(*parts: str) -> str:
    """자산 루트 기준 경로를 문자열로. 예: asset('spider', 'spider.usd')"""
    return str(asset_root().joinpath(*parts))


def testset_dir() -> str:
    """testset USD 트리 (Isaac standalone 아래). ``MMS_TESTSET_DIR`` 로 override."""
    return os.environ.get(
        "MMS_TESTSET_DIR",
        os.path.expanduser("~/isaacsim/standalone_examples/play/MMS/testset"),
    )


#: 자주 쓰는 씬
V3_SCENE = asset("frame_xarm7_spider_turntable_v2", "v3_scene.usd")
V2_USD = asset("frame_xarm7_spider_turntable", "v2.usd")
ASSET_V2_DIR = asset("frame_xarm7_spider_turntable_v2")


if __name__ == "__main__":
    print(f"asset_root   = {asset_root()}")
    print(f"V3_SCENE     = {V3_SCENE}   (exists={Path(V3_SCENE).exists()})")
    print(f"V2_USD       = {V2_USD}   (exists={Path(V2_USD).exists()})")
    print(f"testset_dir  = {testset_dir()}")
