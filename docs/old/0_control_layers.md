아래가 전체 정리 md야. 그대로 복붙하면 돼.

```markdown
# MMS 좌표계 프레임워크 설계

---

## 1. 프레임 정의

| 기호 | 이름 | 설명 |
|------|------|------|
| B | Base frame | xArm7 로봇 베이스 = World |
| F | Turntable frame | 턴테이블 회전축 중심 |
| O | Object / Internal Global frame | 첫 스캔 기준 내부 글로벌 프레임 (Artec 스타일) |
| E | End-effector frame | 로봇 플랜지(TCP) |
| C | Camera frame | 엔드이펙터에 rigid하게 고정된 카메라 |

### 노테이션 규칙

- `T_A^B` = A 프레임에서 B 프레임으로 가는 4×4 homogeneous transform
- 체인 규칙: `T_A^C = T_A^B · T_B^C` (중간 프레임 B가 약분)

---

## 2. O 프레임 설계 원칙 (시나리오 A)

O는 **물리 턴테이블에 rigid하게 묶인 프레임이 아니다.**  
첫 스캔/등록 결과를 기준으로 정의하는 **내부 글로벌 프레임(Internal Global Frame)**이다.

- Artec Studio의 "프로젝트 글로벌 좌표계"와 동일한 역할
- 별도의 O 캘리브레이션 불필요
- 첫 스캔 시점에 O↔F 관계(`^O T_F^(0)`)를 한 번 정의하고 고정

---

## 3. 레이어 분리

### 3.1 위층 — NBV / 재구성 레이어

- **O–C 관계만 본다.**
- 입력: 현재까지의 모델(포인트/메쉬) + 과거 뷰들의 `^O T_C^(k)`
- 출력: 다음에 가야 할 카메라 포즈 `^O T_C_des`
- **로봇, 턴테이블, θ, B/F/E를 전혀 모른다.**

### 3.2 아래층 — 하드웨어 레이어

- NBV가 요구한 `^O T_C_des`를 만족하는 θ, `^B T_E`를 찾는다.
- 사용 프레임: B, F, O, E, C

---

## 4. 트랜스폼 관리

### 4.1 상수 (캘리브레이션/설계로 결정, 고정)

| 트랜스폼 | 의미 | 결정 방법 |
|----------|------|-----------|
| `^E T_C` | E → C (hand–eye) | hand–eye 캘리브레이션 |
| `^B T_F^(0)` | B → F 초기 설치 자세 | 지그/측정으로 고정 |
| `^O T_F^(0)` | O → F 초기 관계 | 첫 스캔 시점에 정의 |

### 4.2 가변값 (매 스텝 결정)

| 값 | 의미 | 결정 방법 |
|----|------|-----------|
| θ | 턴테이블 각도 | 엔코더 읽기 or NBV policy |
| `^B T_E` | B → E (엔드이펙터 포즈) | 로봇 IK 결과 |

---

## 5. `^B T_F(θ)` 유도

턴테이블은 **F 자신의 z축**으로 θ만큼 회전한다.  
→ 초기 설치 자세에 오른쪽에 Rz(θ)를 곱한다 (로컬 축 회전 = 오른쪽 곱).

```
^B T_F(θ) = ^B T_F^(0) · Rz(θ)
```

4×4 형태:

```
^B T_F(θ) = [ R0 · Rz(θ)   t0 ]
             [      0        1  ]
```

- R0, t0: 초기 설치 자세/위치 (고정)
- Rz(θ): z축 회전 행렬

역행렬:

```
^F T_B(θ) = inv(^B T_F(θ))
           = [ Rz(θ)^T · R0^T    -Rz(θ)^T · R0^T · t0 ]
             [       0                      1            ]
```

코드:
```python
def T_B_F(theta, T_B_F0):
    Rz = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
       , [ppl-ai-file-upload.s3.amazonaws](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/images/163947877/3e571135-98ce-44da-9e12-9471904350c7/image.jpg)
       , [ppl-ai-file-upload.s3.amazonaws](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/images/163947877/3e571135-98ce-44da-9e12-9471904350c7/image.jpg)
    ])
    return T_B_F0 @ Rz
```

---

## 6. 핵심 체인: O 기준 카메라 포즈

O에서 C까지 경로: **O → F → B → E → C**

```
^O T_C(θ, ^B T_E) = ^O T_F · ^F T_B(θ) · ^B T_E · ^E T_C
```

각 항의 역할:

| 항 | 역할 | 상수/가변 |
|----|------|-----------|
| `^O T_F` | O↔F 기준 관계 | 상수 |
| `^F T_B(θ) = inv(^B T_F(θ))` | 턴테이블 회전 포함 | θ에 따라 가변 |
| `^B T_E` | 로봇 IK 결과 | 가변 |
| `^E T_C` | hand–eye | 상수 |

코드:
```python
def T_O_C(theta, T_B_E, T_O_F0, T_B_F0, T_E_C):
    T_B_F_theta = T_B_F0 @ Rz(theta)
    T_F_B = np.linalg.inv(T_B_F_theta)
    return T_O_F0 @ T_F_B @ T_B_E @ T_E_C
```

---

## 7. NBV 목표 방정식

NBV가 `^O T_C_des`를 주면, 이를 만족하는 `^B T_E`를 구한다:

```
^O T_C_des = ^O T_F · ^F T_B(θ) · ^B T_E · ^E T_C

→ ^B T_E(θ) = inv(^O T_F · ^F T_B(θ)) · ^O T_C_des · inv(^E T_C)
             = ^B T_F(θ) · inv(^O T_F) · ^O T_C_des · inv(^E T_C)
```

코드:
```python
def solve_T_B_E(theta, T_O_C_des, T_O_F0, T_B_F0, T_E_C):
    T_B_F_theta = T_B_F0 @ Rz(theta)
    T_B_O = T_B_F_theta @ np.linalg.inv(T_O_F0)
    return T_B_O @ T_O_C_des @ np.linalg.inv(T_E_C)
```

θ는 아래 중 하나로 결정:
- NBV policy (예: frontier 법선이 로봇 정면을 향하는 각도)
- 충돌 회피 조건
- IK 가능 여부로 후보 iterate

---

## 8. 한 스텝 사이클 (구현 관점)

```
① NBV 레이어
   입력: 모델 + 과거 ^O T_C^(k)
   출력: ^O T_C_des

② 하드웨어 레이어
   1) θ 후보 결정 (policy / iterate)
   2) 각 θ에 대해 ^B T_E(θ) 계산
   3) IK + collision check → 실행 가능한 (θ, ^B T_E) 선택
   4) 로봇 / 턴테이블에 명령

③ 실제 스캔 후
   1) 실제 θ_act, ^B T_E_act 읽기
   2) ^O T_C_act = T_O_C(θ_act, ^B T_E_act) 계산
   3) ^O T_C_act + 포인트클라우드를 NBV/모델링 레이어에 저장
```

---

## 9. 설계 요약

```
[NBV 레이어]          O–C 관계만 다룸
      ↕  ^O T_C_des / ^O T_C_act
[하드웨어 레이어]      B/F/O/E/C 전체 체인 처리
      ↕  θ, ^B T_E
[로봇 / 턴테이블]
```

- **상수**: `^E T_C`, `^B T_F^(0)`, `^O T_F^(0)`
- **가변**: θ (엔코더), `^B T_E` (IK)
- **인터페이스**: `^O T_C` (두 레이어를 연결하는 유일한 포즈 표현)
```