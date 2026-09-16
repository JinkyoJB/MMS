"""캡처 재시도 + 스캐너 세션 재초기화.

왜 필요한가
----------
2026-09-16 intrinsic 실행에서 **20자세 중 13자세까지만 캡처됐다.** 로봇은 계속
움직이고 있었고(자세당 ~30초가 끝까지 유지됐다) 자세 목록·IK 도 문제가 없었다.
바로 뒤에 새 프로세스로 돈 hand-eye 는 같은 목록에서 **19/20** 을 받았다.
즉 자세가 아니라 **스캐너 세션이 도중에 죽은 것**이고, 세션을 새로 열면 회복된다.

기존 `_capture` 들은 `RuntimeError` 를 한 번 잡고 `None` 을 돌려줄 뿐이라,
한번 죽으면 남은 자세가 전부 조용히 skip 됐다. `charuco 부족` 같은 정상적인
skip 과 구분도 안 됐다.

여기서는
  1. 짧은 간격으로 몇 번 재시도하고,
  2. 그래도 안 되면 **shutdown → initialize** 로 세션을 새로 열고 다시 시도하며,
  3. 재초기화까지 실패하면 그 사실을 분명히 출력한다.
"""
from __future__ import annotations

import time


def capture_with_retry(client, *, name: str = "", tries: int = 3,
                       delay_s: float = 0.6, allow_reinit: bool = True):
    """캡처 1장. 실패하면 재시도하고, 필요하면 스캐너를 다시 연다.

    Returns: FrameMeshHandle 또는 None (완전 실패).
    """
    tag = f"[{name}] " if name else ""

    for attempt in range(1, tries + 1):
        try:
            fmh = client.capture_frame(capture_texture=True)
        except RuntimeError as e:
            # SDK 가 reconstructAndTexturizeMesh 실패 시 0x80070803 등을 올린다.
            fmh = None
            if attempt == tries:
                print(f"  {tag}capture 예외 ({attempt}/{tries}): {e}")
        if fmh is not None and fmh.has_image():
            if attempt > 1:
                print(f"  {tag}capture 재시도 {attempt}회째 성공")
            return fmh
        if attempt < tries:
            time.sleep(delay_s)

    if not allow_reinit:
        print(f"  {tag}capture 실패 ({tries}회)")
        return None

    # ── 세션 재초기화 ──────────────────────────────────────────────────
    print(f"  {tag}capture {tries}회 연속 실패 — 스캐너 세션을 다시 연다")
    try:
        client.shutdown()
    except Exception as e:
        print(f"    shutdown 경고: {e}")
    time.sleep(1.0)
    try:
        client.initialize()
    except Exception as e:
        print(f"    ✘ 재초기화 실패: {e} — 이후 자세도 캡처되지 않을 수 있다")
        return None

    try:
        fmh = client.capture_frame(capture_texture=True)
    except RuntimeError as e:
        print(f"    ✘ 재초기화 후에도 실패: {e}")
        return None
    if fmh is None or not fmh.has_image():
        print("    ✘ 재초기화 후에도 빈 프레임")
        return None
    print("    ✓ 재초기화 후 캡처 성공")
    return fmh
