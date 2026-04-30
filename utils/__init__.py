"""
Sensor-agnostic infrastructure shared by mms_phoxi / mms_artec / mms_orbbec.

- transforms.py        : 좌표계 변환 (T_AB notation)
- scan_result.py       : ScanResult 공통 dataclass
- calibration/         : hand_eye_calibrator (AX=XB)
- robot/               : XArmInterface
- turntable/           : Ezi-SERVO Turntable
- control/             : theta_planner, hardware_layer
- nbv/                 : sensor-agnostic algorithms (frontier, icp, manual_picker, _progress_vis)
"""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent    # utils/ → 프로젝트 루트
