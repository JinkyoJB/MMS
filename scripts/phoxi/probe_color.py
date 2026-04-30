#!/usr/bin/env python
# scripts/phoxi_probe_color.py
#
# PhoXi 3D Scanner Gen3 의 RGB/Color 관련 GenICam 노드가 실제로 노출되는지 확인.
# 장치 반납 전 1회 실행해서 매뉴얼 기능(ColorCameraImage, CameraSpace=ColorCamera 등)
# 이 GenTL 레벨에서 어떤 이름으로 접근 가능한지 스냅샷 저장.
#
# 출력:
#   - 콘솔 로그
#   - datasets/phoxi_color_probe_YYYYMMDD_HHMMSS.json
#     {
#       "device": {...},
#       "components": {selector symbolics, 현재 enable 상태},
#       "nodes": {노드 이름 → type, 현재 값, 선택 가능한 심볼릭, accessible 여부},
#     }

from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms_phoxi.sensor.phoxi_client import PhoxiClient, PhoxiConfig


# 시리얼 고정 — Helios 오선택 방지
PHOXI_SERIAL = "SEA-023"

# 확인할 노드 후보 (매뉴얼 PhoXiControl 1.16 기준)
COLOR_NODE_CANDIDATES = [
    "ColorCameraImage",
    "ColorCameraImageEnable",
    "CameraSpace",
    "Scan3dCoordinateSelector",
    "Scan3dDistanceUnit",
    "Scan3dOutputMode",
    "OutputTopology",
    "TextureSource",
    "CapturingSettingsTextureSource",
    "CapturingSettings_TextureSource",
    "ColorSettings",
    "ColorResolution",
    "ColorSettings_Resolution",
    "Scan3dFocalLength",
    "Scan3dPrincipalPointU",
    "Scan3dPrincipalPointV",
    "PayloadType",
]


def _dump_node(features, name: str) -> dict:
    info: dict = {"exists": False}
    try:
        node = features.get_node(name)
    except Exception as e:
        info["error"] = f"{type(e).__name__}: {e}"
        return info
    if node is None:
        return info
    info["exists"] = True
    info["type"] = type(node).__name__
    # 값
    try:
        info["value"] = node.value
    except Exception as e:
        info["value_error"] = str(e)
    # 심볼릭 (Enumeration)
    try:
        info["symbolics"] = list(node.symbolics)
    except Exception:
        pass
    # access mode
    try:
        info["access_mode"] = str(getattr(node, "access_mode", ""))
    except Exception:
        pass
    return info


def main() -> None:
    cfg = PhoxiConfig(
        serial_number=PHOXI_SERIAL,
        trigger_timeout_s=15.0,
    )
    client = PhoxiClient(cfg)
    client.initialize()

    report: dict = {
        "timestamp": datetime.now().isoformat(),
        "device": {
            "serial": PHOXI_SERIAL,
            "firmware": None,
        },
        "components": {},
        "nodes": {},
        "notes": [],
    }

    try:
        features = client._features

        # firmware
        try:
            report["device"]["firmware"] = str(features.DeviceFirmwareVersion.value)
        except Exception:
            pass

        # Component 목록
        try:
            all_comps = list(features.ComponentSelector.symbolics)
            report["components"]["all_symbolics"] = all_comps
            print(f"\n[probe] ComponentSelector.symbolics = {all_comps}")

            enabled = {}
            for c in all_comps:
                try:
                    features.ComponentSelector.value = c
                    enabled[c] = bool(features.ComponentEnable.value)
                except Exception as e:
                    enabled[c] = f"error: {e}"
            report["components"]["enabled"] = enabled
            print(f"[probe] 현재 enabled 상태: {json.dumps(enabled, indent=2)}")

            # ColorCamera 관련 키워드 가진 컴포넌트 하이라이트
            color_comps = [c for c in all_comps if "color" in c.lower()]
            if color_comps:
                report["notes"].append(
                    f"color/rgb 관련 컴포넌트 후보: {color_comps}"
                )
                print(f"\n[probe] ★ RGB 관련 컴포넌트: {color_comps}")
            else:
                report["notes"].append(
                    "ComponentSelector 심볼릭에 'Color' 포함 없음 — "
                    "ColorCameraImage 는 다른 경로로 노출될 수 있음"
                )
                print("[probe] ComponentSelector 에 Color* 컴포넌트 없음")
        except Exception as e:
            report["components"]["error"] = str(e)
            print(f"[probe] ComponentSelector 접근 실패: {e}")

        # 개별 노드 확인
        print("\n[probe] 개별 노드 조회:")
        for name in COLOR_NODE_CANDIDATES:
            info = _dump_node(features, name)
            report["nodes"][name] = info
            status = "✓" if info.get("exists") else "✗"
            extra = ""
            if info.get("exists"):
                extra = f"  type={info.get('type','?')}  value={info.get('value','?')}"
                if info.get("symbolics"):
                    extra += f"  symbolics={info['symbolics']}"
            print(f"  {status} {name:<38s}{extra}")

        # 추가: 대문자/언더스코어 변형까지 searching (노드 이름 혼동 방지)
        # node_map 의 모든 노드 나열은 대량일 수 있어 요약만
        try:
            all_nodes = features.get_feature_names() if hasattr(features, "get_feature_names") else []
            color_related = [n for n in all_nodes if "color" in n.lower() or "rgb" in n.lower()]
            if color_related:
                report["nodes"]["_color_related_in_nodemap"] = color_related
                print(f"\n[probe] node map 에서 color/rgb 포함 이름: {color_related}")
        except Exception:
            pass

    finally:
        client.shutdown()

    # 저장
    out_dir = _PROJECT_ROOT / "datasets"
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"phoxi_color_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n[probe] 리포트 저장: {out}")


if __name__ == "__main__":
    main()
