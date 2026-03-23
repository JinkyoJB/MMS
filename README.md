# MMS
Multi Modal 3D Scanning System


MMS/
├── README.md
├── main.py              # 오케스트레이션만 담당
└── mms/
    ├── __init__.py
    ├── robot/
    │   ├── __init__.py
    │   └── xarm_ros_client.py   # xarm_ros 기반 래퍼 인터페이스
    ├── turntable/
    │   ├── __init__.py
    │   └── controller.py        # 앞으로 구현
    └── sensor/
        ├── __init__.py
        └── photoneo_client.py   # photoneo 예제 기반 래퍼[web:1]
