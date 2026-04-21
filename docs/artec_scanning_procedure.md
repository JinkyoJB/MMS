아래 내용은 **Artec 스캐닝 파이프라인 전체 구조를 Photoneo PhoXi용으로 벤치마킹하기 좋게 정리한 Markdown 문서**입니다. Artec Studio + SDK 공식 흐름(Scanning → Cleaning → Alignment → Registration → Fusion → Postprocessing)을 기반으로, SDK 관점의 Scanning Procedure + General Pipeline까지 한 번에 묶었습니다. [docs.artec3d](https://docs.artec3d.com/as/18/en/qsg.html)

***

```markdown
# Artec 3D 스캐닝 파이프라인 구조

Artec Studio + Artec SDK 2.0이 사용하는 **end-to-end 스캐닝 파이프라인 구조**를 정리한 문서이다.  
Photoneo PhoXi 3D로 구현할 때, 이 구조를 그대로 벤치마킹할 수 있도록 단계·역할·입출력 기준으로 나눈다.

---

## 0. 전체 개요

Artec 에코시스템에서 3D 모델을 만드는 표준 흐름은 아래와 같이 요약된다. [web:314][web:313][web:157]

1. **Scanning**  
   - 스캐너로 프레임을 캡처하고, PC로 전송한다.
   - 실시간으로 프레임을 재구성하고, 하나의 좌표계에 정렬한다. [web:184][web:27]

2. **Cleaning**  
   - 불필요한 바닥/배경/노이즈 영역을 지운다. (Studio에서는 Eraser 도구로 rough cleaning) [web:314][web:318]

3. **Alignment**  
   - 여러 개의 scan(서로 다른 위치/자세에서 촬영된 시퀀스)을 서로 정렬한다. [web:314][web:211]

4. **Registration**  
   - 각 scan 내부의 frame 정렬을 최적화하고 (fine/serial),  
   - 전체 scan 세트를 한 좌표계로 global registration 한다. [web:314][web:212][web:157]

5. **Fusion**  
   - 여러 scan을 하나의 mesh(Composite Mesh)로 합친다. (Fast/Poisson Fusion) [web:314][web:212]

6. **Postprocessing**  
   - 작은 객체/노이즈 제거, mesh simplification, smoothing, texturing 등. [web:314][web:157][web:212]

SDK 2.0 관점에서 보면 이 흐름은:

- **Capturing/Scanning API**가 1단계를 담당,
- **Algorithm API**가 4~6단계를 담당,
- **Project API**가 프로젝트로 저장/불러오기(입출력)를 담당한다. [web:2][web:142][web:27]

---

## 1. 스캐닝 절차 (Scanning Procedure)

### 1.1 개념

Artec SDK의 `ScanningProcedure`는 **“프레임 캡처 → 재구성 → 등록 → IModel에 저장”까지를 하나의 파이프라인으로 묶은 고수준 스캐닝 엔진**이다.  
Basics 문서에서 “가장 일반적인 사용 시나리오에서는 scanning procedure를 사용하는 것이 최선”이라고 명시한다. [web:2]

```text
Scanner → Frames (IFrame)
       → Reconstruction → IFrameMesh
       → Registration   → 공통 좌표계
       → 저장           → IModel (scan 단위)
```

- `IScanningProcedure`는 `ScanningProcedureSettings`를 인자로 생성되며,  
  이 안에 **어떤 파이프라인 단계들을 켤 것인지(ScanningPipeline flags)** 와  
  **어떤 registration 타입을 쓸지(RegistrationAlgorithmType)** 등을 지정한다. [web:40]

- Scanning Procedure는 콜백 인터페이스(`IScanningProcedureObserver`)를 지원해서,  
  각 frame이 캡처/등록될 때 상태와 데이터를 실시간으로 통지할 수 있다.  
  (예: 프리뷰 렌더링, 온라인 품질 평가, NBV 후보 결정 등) [web:40][web:2]

### 1.2 ScanningPipeline 플래그

`ScanningPipeline` enum은 **스캐닝 시 어느 단계까지 실시간으로 수행할지**를 정의하는 bit flags 집합이다. [web:40][web:2]

주요 플래그(개념 수준):

- `Capture`   – 프레임 캡처
- `Reconstruct` – 2D frame → 3D surface(IFrameMesh) 재구성
- `Register`  – 현재 frame을 scan 내 공통 좌표계로 등록
- `Store`     – 재구성/등록된 frame을 IModel에 저장

실제 SDK 내부에서는 이 flag 조합을 `ScanningProcedureSettings.pipelineConfiguration`에 넣어 사용한다. [web:40]

---

## 2. 데이터 계층: Frame → Scan → Model

Artec SDK는 스캐닝 데이터를 아래 3단계 계층으로 관리한다. [web:142][web:2]

1. **Frame (IFrame / IFrameMesh)**  
   - 스캐너가 캡처한 단일 시점의 2D 이미지/깊이.  
   - 재구성 후에는 `IFrameMesh`로 표면(mesh patch)이 된다. [web:142]

2. **Scan (IModel의 scan 항목)**  
   - 프레임들의 시퀀스, 하나의 스캔 세션 결과.  
   - Scanning Procedure가 만들어내는 기본 단위. [web:2]

3. **Final Model (ICompositeMesh)**  
   - 여러 scan을 정합/융합하여 만든 최종 mesh.  
   - Fusion 알고리즘의 결과물. [web:142]

이 계층 구조를 Photoneo에서도 그대로 가져가면 좋다:

- PhoXi frame → PhoXiFrameMesh  
- PhoXi scan(시퀀스) → ScanEntity  
- 최종 fused mesh → FinalModel

---

## 3. SDK 기반 전체 파이프라인 (General Pipeline 기준)

Algorithm SDK의 General Pipeline 문서는 **후처리 단계 알고리즘을 어떤 순서로 돌려야 하는지**를 설명한다. [web:47][web:142][web:157]

대표적인 순서는 다음과 같다:

1. **Align (Alignment / Serial Registration)**  
   - 여러 scan을 대략적 또는 정밀하게 서로 정렬한다.  
   - `SerialRegistration` 알고리즘 사용. [web:212][web:211]

2. **Global Registration**  
   - 전체 frame/scan 세트를 하나의 최적화된 좌표계로 맞춘다. [web:157][web:27]

3. **Outliers Removal / Small Objects Filter**  
   - 작은 노이즈/불필요한 객체를 제거한다. [web:157][web:212]

4. **Fusion (Fast / Poisson)**  
   - 여러 scan을 하나의 mesh로 융합한다. [web:314][web:212]

5. **Mesh Simplification**  
   - triangle 수를 줄이되, 품질을 유지한다. [web:157][web:47]

6. **Texturing**  
   - UV/atlas 기반 텍스처를 생성한다. [web:157][web:68]

SDK 2.0에서는 이 단계들을 `artec::sdk::algorithms` 네임스페이스의 알고리즘 클래스로 제공한다. [web:142][web:2]

---

## 4. 사용자 레벨 스캐닝 워크플로우 (Studio 관점)

Artec Studio 매뉴얼의 “3D Scanning at a Glance” 챕터는 사용자가 따라가는 전체 흐름을 이렇게 정리한다. [web:314][web:313][web:157]

1. **Scanning**  
   - 프리뷰 모드에서 퀄리티/거리 확인 후,  
   - Record를 누르고 대상 주위를 돌며 모든 면을 캡처한다. [web:157][web:184]

2. **Cleaning**  
   - Eraser 도구 등으로 바닥/배경/불필요한 객체 rough cleaning. [web:314][web:318]

3. **Alignment**  
   - 여러 scan을 서로 align한다. (Align 기능) [web:212][web:211]

4. **Registration**  
   - Global registration으로 frame/scan 전체를 정밀 정합. [web:314][web:157]

5. **Fusion**  
   - Fast/Sharp/Smooth Fusion으로 단일 mesh 생성. [web:314][web:212]

6. **Postprocessing**  
   - Small-object filter, mesh simplification, smoothing, hole filling, texturing 등. [web:314][web:157][web:68]

Photoneo 파이프라인을 설계할 때도 “사용자 관점”과 “SDK 관점”이 둘 다 보이게 맞추는 게 좋다.

---

## 5. 단계별 역할 / 입출력 요약

### 5.1 Scanning 단계

- **입력**  
  - PhoXi/Artec 스캐너 장치  
  - 스캔 설정(geometry only / geometry+texture, frame rate, exposure 등)

- **역할**  
  - 프레임 촬영, 3D 재구성, 실시간 registration(선택), IModel-like 구조에 축적. [web:2][web:184]

- **출력**  
  - 하나 이상의 Scan 객체(프레임 시퀀스)  
  - 필요 시, 스캔 직후에 곧바로 Fusion(Real-time fusion 모드) 결과도 생성 가능. [web:157][web:184]

### 5.2 Cleaning 단계

- **입력**  
  - 한 개 이상의 Scan/FrameMesh 집합

- **역할**  
  - 바닥/배경/잡물 제거  
  - 이론상 알고리즘으로 자동화할 수 있으나, Studio에서는 수동 도구(Eraser) 중심. [web:314][web:318]

- **출력**  
  - 정리된 Scan/FrameMesh 데이터

### 5.3 Alignment / Registration 단계

- **입력**  
  - 여러 Scan (서로 다른 촬영 위치) [web:211][web:212]

- **역할**  
  - Alignment: scan들 간에 rigid transform을 계산해 대략 맞춘다.  
  - Serial/Global Registration: frame/scan 전체를 정밀하게 한 좌표계로 맞춘다. [web:157][web:27]

- **출력**  
  - 전부 하나의 좌표계에 있는 Scan 집합  
  - registration 품질 메트릭(오차, 실패 프레임 비율 등)

### 5.4 Fusion 단계

- **입력**  
  - 정렬된 Scan 전체 (registered scans) [web:314][web:212]

- **역할**  
  - surface reconstruction: 여러 scan의 중복 영역을 합쳐 **연속적인 표면(mesh)** 생성  
  - noise/holes 처리 옵션 포함

- **출력**  
  - 단일 Composite Mesh (Final Model)

### 5.5 Postprocessing 단계

- **입력**  
  - Composite Mesh [web:157][web:68]

- **역할**  
  - Small-object filter: 작은/불필요한 객체 제거  
  - Mesh simplification: polycount 줄이기  
  - Smoothing, sharpening, hole filling  
  - Texturing: UV unwrapping, atlas 생성, texture baking [web:68][web:157]

- **출력**  
  - 최종 3D 모델 (mesh + texture)

---

## 6. Photoneo PhoXi에 벤치마킹할 때의 매핑 아이디어

Artec 파이프라인을 Photoneo PhoXi로 옮기려면, 단계별로 대응을 잡으면 된다.

| Artec 개념 | PhoXi 대응 | 설명 |
|---|---|---|
| IFrame | PhoXi depth frame / point cloud | 단일 촬영 |
| IFrameMesh | PhoXi frame에서 재구성한 mesh patch | 내부 또는 외부 meshing 사용 |
| Scan (IModel 내 scan) | PhoXi scan 세션 | 같은 target을 여러 pose에서 캡처한 시퀀스 |
| Scanning Procedure | PhoXi + 자체 pipeline | trigger → reconstruct → register → accumulate |
| Alignment / Registration | Pose estimation / ICP | PhoXi frame들 간 또는 PhoXi vs world간 정합 |
| Fusion | volumetric TSDF/Poisson | 여러 scan을 하나의 mesh로 융합 |
| Postprocessing | 동일 | small object removal, simplification, texturing 등 |

Photoneo에서는:

- PhoXi SDK로 raw frames/point clouds를 받고,
- Artec Scanning Procedure와 유사한 상위 레이어를 만들어  
  **“Capture → Reconstruct → Register → Accumulate(Scan) → Process(Algorithms)”** 흐름을 그대로 재현하는 것이 목표가 될 수 있다. [web:142][web:2][web:27]

---

## 7. 파이프라인을 코드 레벨로 구현할 때 참고할 점

- **고수준 API vs 저수준 API**  
  - Artec도 Capturing API(저수준)와 Scanning Procedure(고수준)를 분리한다. [web:2]  
  - Photoneo도 PhoXi raw frame 레벨과 NBV/registration/fusion이 묶인 상위 파이프라인을 분리하는 것이 좋다.

- **Frame-level callback**  
  - Artec의 `IScanningProcedureObserver`처럼,  
    PhoXi frame 도착마다 callback을 받아 실시간 품질 평가 및 NBV 후보 평가를 할 수 있는 구조가 이상적이다. [web:40][web:2]

- **General Pipeline 순서 준수**  
  - Artec는 “Processing 시 알고리즘은 General Pipeline 순서대로 실행해야 한다”고 명시한다. [web:2][web:47]  
  - Photoneo에서도 alignment → global registration → fusion → postprocessing 순서를 유지해야 단계 간 전제조건이 깨지지 않는다.

---

## 8. 한 줄 요약

Artec의 스캐닝 구조는  
**“Scanning Procedure로 Frame/Scan을 만들고, Algorithms General Pipeline으로 Scan들을 Final Mesh로 가공하는 2단계 구조”**로 요약할 수 있다. [web:2][web:47][web:314]

이 구조를 그대로 PhoXi에 이식하려면:

1. PhoXi용 Scanning Procedure 레이어를 만들고,  
2. PhoXi 데이터를 입력으로 쓰는 Algorithm Pipeline 레이어를 올리면 된다.
```