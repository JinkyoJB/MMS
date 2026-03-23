# MMS — Multi Modal 3D Scanning System

로봇(xArm), 턴테이블, 3D 센서(Photoneo/GenICam)를 통합 제어하는 멀티모달 3D 스캐닝 시스템입니다.

---

## 환경 설정

### 1. 레포지토리 클론

\```bash
git clone https://github.com/JinkyoJB/MMS
cd MMS
\```

### 2. 가상환경 생성 및 패키지 설치

\```bash
conda create -n mms311 python=3.11
conda activate mms311
pip install -r requirements.txt
\```

---

## 개발자용: 패키지 동결 (requirements.txt 갱신)

새 패키지를 설치한 후 아래 명령으로 requirements.txt를 업데이트합니다.

\```bash
conda activate mms311
pip freeze > requirements.txt
\```

> ⚠️ ROS 관련 패키지(xarm_ros)는 pip로 관리되지 않으므로 별도 설치가 필요합니다.
