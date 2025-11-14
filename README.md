# pgv_tracing_mode (라인 트레이싱 / 정렬 패키지)

`pgv_tracing_mode` 는 PGV(R4 등) 라인 센서가 퍼블리시하는 `PoseStamped (x,y,yaw)` 데이터를 이용하여
로봇을 라인 위에서 정렬 / 주행 / 미세 위치 보정하는 ROS 2 (Humble) Python 패키지입니다.
홀로노믹(측면 이동 가능)과 비홀로노믹(차동/조향) 플랫폼을 모두 지원하며, 서비스·토픽 기반의 간단한 인터페이스로
상대/절대 이동, yaw 정렬, Y축 단독 정렬, 수동 hold-to-run 제어를 제공합니다.

---

## 🧭 개요 (Overview)

주요 특징:
* Yaw 정렬 (ALIGN_YAW) + 선택적 Slow Start 전진
* 절대/상대 X 목표 추종 (RUNNING) 과 Y=0 유지
* 홀로노믹 전용 Y축 정렬 (ALIGN_Y_ONLY) + Yaw 히스테리시스 (미세 도리도리 억제)
* Hold-to-run 수동 전/후진 (teleop_speed / manual_timeout 기반)
* Pose 입력 타임아웃 감지 + 재인식 후 Grace(유예) 정지 (`post_timeout_grace_sec`)
* Reverse heading 허용(±180°를 정렬로 인정) 옵션 (`allow_reverse_heading`)
* ±π 경계 진동 억제 wrap band (`yaw_wrap_band_deg`)로 yaw 부호 안정화
* Slew rate(가속) 제한 + 속도/각속도 클램프
* YAML 파라미터 / launch 파일 제공
* Dummy PGV 시뮬레이터(`sim_dummy_pgv.py`) 포함 → 실제 센서 없이 폐루프 테스트 가능

상태(Phase) 요약:
| Phase | 설명 |
|-------|------|
| IDLE | 목표/정렬 요청 없음 (정지) |
| ALIGN_YAW | yaw 오차를 임계값 이하로 회전 보정 |
| SLOW_START | 짧은 저속 전진 (기계/센서 워밍업) |
| RUNNING | X 목표 추종 + Y, yaw 안정화 |
| ALIGN_Y_ONLY | 홀로노믹 Y축 정렬 (yaw 히스테리시스 적용) |
| DONE | 완료 후 정지 유지 |

히스테리시스:
* |yaw_err| > tol_yaw * yaw_hysteresis_factor → yaw 보정 활성
* |yaw_err| <= tol_yaw → 보정 비활성 + 작은 deadband 로 미세 진동 억제

---

## ⚙️ 의존성 (Dependencies)

- **ROS 2 Humble** (>= 2022.05)
- **Python 3.10**
- **C++17 toolchain**
- ROS 2 packages:
  - `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`

---

## 🧩 포함된 구성 요소

| 파일 / 노드 | 설명 |
|-------------|------|
| `pgv_tracing_mode/line_tracing_node.py` | 핵심 제어 노드 (LineDriveNode) |
| `pgv_tracing_mode/sim_dummy_pgv.py` | `/cmd_vel` 적분 후 PGV PoseStamped 더미 발행 |
| `pgv_tracing_mode/one_shot_goal.py` | 테스트용 단발 목표(상대/절대) 및 서비스 호출 예제 |
| `launch/line_drive.launch.py` | 기본 제어 노드 런치 |
| `launch/test_line_drive_sim.launch.py` | 더미 + 제어 + 목표 통합 테스트 런치 |
| `config/line_drive.params.yaml` | 제어 파라미터 |
| `config/sim.params.yaml` | 시뮬레이터 파라미터 |

---

## 🛠️ 설치 (Installation)

```bash
# 워크스페이스 생성 및 현재 패키지 clone (예시)
mkdir -p ~/pgv_ws/src && cd ~/pgv_ws/src
git clone https://github.com/<your-org-or-user>/pgv_tracing_mode.git

cd ~/pgv_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 사용법 (Usage)

### ▶ 기본 실행 (YAML 파라미터 기반)

```bash
ros2 launch pgv_tracing_mode line_drive.launch.py
```

### ▶ 통합 시뮬레이션 (Dummy PGV + Controller + Goal)

```bash
ros2 launch pgv_tracing_mode test_line_drive_sim.launch.py
```

### ▶ 주요 Launch Arguments

| 이름                | 기본값                             | 설명 |
|--------------------|----------------------------------|------|
| `holonomic`        | `true`                           | true=홀로노믹, false=비홀로노믹 |
| `params_file`      | `config/line_drive.params.yaml`  | 제어 파라미터 YAML |
| `sim_params`       | `config/sim.params.yaml`         | 시뮬레이터 파라미터 YAML |
| `use_relative`     | `true`                           | 상대(Δx) 목표 사용 여부 |
| `relative_goal`    | `1.5`                            | 상대 이동 거리 (m) |
| `absolute_goal`    | `5.0`                            | 절대 목표 x (m) |
| `goal_delay_sec`   | `1.0`                            | 목표 발행 전 지연 (s) |
| `call_align_first` | `true`                           | 시작 시 yaw 정렬 먼저 수행 |

---

## 📦 주요 파라미터 (Parameters)

`config/line_drive.params.yaml` 참고. (일부만 발췌)

| 파라미터 | 단위 | 설명 |
|----------|------|------|
| `pose_topic` | str | PGV PoseStamped 입력 토픽 |
| `cmd_topic` | str | Twist 출력 토픽 |
| `holonomic` | bool | 제어 모드 (홀로/비홀로) |
| `yaw_align_threshold_deg` | deg | ALIGN_YAW 종료 기준 |
| `tolerance_yaw_deg` | deg | 안정 yaw 허용 오차 |
| `yaw_hysteresis_factor` | - | Y-only yaw 히스테리시스 외측 배수 (>|tol_yaw|*factor 시 활성, ≤|tol_yaw| 시 비활성) |
| `tolerance_xy` | m | 목표/정렬 X,Y 허용 오차 |
| `pose_timeout_sec` | s | Pose 수신 타임아웃 (안전 정지) |
| `post_timeout_grace_sec` | s | 타임아웃 후 Pose 회복 시 추가 정지 유지 유예시간 |
| `control_rate` | Hz | 제어 루프 주파수 |
| `max_lin_vel` / `max_ang_vel` | m/s, rad/s | 속도 상한 |
| `accel_lin` / `accel_ang` | m/s², rad/s² | Slew (가속) 제한 |
| `slow_start_duration` / `slow_start_speed` | s, m/s | Slow start 설정 |
| `teleop_speed` | m/s | 수동 hold-to-run 속도 |
| `manual_timeout` | s | 수동 TRUE 유지 시간 |
| `allow_reverse_heading` | bool | ±180° 방향도 정렬 허용 (최소 회전 목표 선택) |
| `yaw_wrap_band_deg` | deg | ±π 경계 부근 부호 고정 밴드 폭 (진동 억제) |
| `align_y_only_max_ang_vel` | rad/s | Y-only 단계 yaw 미세 보정 soft 상한 (주석 처리 시 비활성) |
| `sensor_x_offset` / `sensor_y_offset` | m | 센서 장착 위치 보정 |
| `sensor_yaw_offset_deg` | deg | 센서 yaw 오프셋 |
| `kp_x`, `kp_y`, `kp_yaw` | - | P 게인 |

> 센서 중심을 라인 중앙에 맞추는 것이 기본 가정. 오프셋을 넣으면 로봇(또는 Base) 기준 정렬로 전환.
### Yaw 제어 및 Reverse Heading

`_yaw_err()` 는 `(yaw_meas - target)` 형태의 오차를 반환합니다.

1. 기본 target = 0 rad (+X).
2. `allow_reverse_heading=true` 이고 `consider_reverse=True` 로 호출된 문맥에서는 후보 {0, ±π(측정 yaw 부호 유지)} 중 |yaw - candidate|가 더 작은 목표를 선택 → 최소 회전량.
3. 회전 제어는 음의 피드백 `w = -kp_yaw * yaw_err` 형태. yaw_err>0 (좌측(CCW)로 기울었을 때) → 음수 각속도(w<0) → 시계방향 회전으로 0쪽 수렴.
4. ±π 경계 근처(| |yaw| - π | < yaw_wrap_band_deg)에서는 sign flip을 억제하기 위해 기존 yaw 부호를 고정(`yaw_sign_fixed`).

Holonomic 모드에서는 정렬 이후 일반적으로 w=0을 유지하며 필요 시 미세 보정(현재 기본 설정에서는 비활성화). Non-holonomic은 y 오차와 yaw 오차를 각속도로 동시에 보정합니다.

### 타임아웃 & Grace 복구

1. `pose_timeout_sec` 동안 새로운 Pose 미수신 → 즉시 STOP, `timeout_active=True`.
2. Pose가 다시 들어오는 첫 틱 → `post_timeout_grace_sec` 동안 추가 정지 (센서 튀는 값/재정렬 안정 확보).
3. Grace 진행 중 수동 hold-to-run 입력도 무시되고 주기적으로 남은 시간이 로그로 출력.
4. Grace 종료 후 상태 머신 정상 재개.

로그 예시:
```
[safety] pose timeout detected -> STOP
[safety] pose recovered; holding still for 1.00s grace
[safety] grace active: 0.76s remaining
[safety] grace ended; resuming control
```

### Y-only 정렬 히스테리시스

ALIGN_Y_ONLY 단계에서 yaw 보정은 |yaw_err|가 `tolerance_yaw_deg * yaw_hysteresis_factor` 보다 클 때만 활성화하고, 다시 |yaw_err| ≤ tolerance_yaw_deg 로 작아지면 비활성화하여 미세 흔들림을 줄입니다. 기본 코드에서는 w_cmd 보정 블록이 주석 처리되어 vy만 사용한 순수 Y 정렬을 수행합니다. 필요 시 주석을 해제하고 `align_y_only_max_ang_vel` 로 soft 포화를 적용할 수 있습니다.

### Hold-to-run 전략

`/line_drive/go_forward` 또는 `/line_drive/go_backward` 에 TRUE 펄스를 계속 보내면 `manual_timeout` 내에서 해당 방향 속도(`teleop_speed`)로 유지됩니다. 양 방향이 동시에 활성이면 안전을 위해 정지합니다.

### 파라미터 튜닝 가이드 (요약)

| 목적 | 권장 조정 파라미터 | 메모 |
|------|--------------------|------|
| 정렬 빠르게 | kp_yaw ↑, yaw_align_threshold_deg ↑ (조금) | 과도하면 진동 ↑ |
| 정렬 안정 | kp_yaw ↓, yaw_wrap_band_deg ↑ | wrap 너무 크면 반응 둔화 |
| 센서 튐 대비 | post_timeout_grace_sec ↑ | 너무 길면 재개 지연 |
| 측면 수렴 속도 | kp_y ↑ | 과도하면 oscillation |
| Holonomic 미세 yaw 보정 | align_y_only_max_ang_vel 설정 + w_cmd 블록 활성 | 기본은 비활성 |
| Reverse 허용 여부 | allow_reverse_heading toggle | 물리 방향성 없는 경우만 true 추천 |


> 💡 **기본 설계 원칙**
> 센서(R4)가 라인 중앙에 위치하도록 제어하는 것이 목표이므로
> 일반적으로 `sensor_*_offset = 0` 그대로 사용하면 됩니다.
> 오프셋을 지정하면 “로봇 중심”이 라인 중앙에 오도록 제어 기준이 바뀝니다.

---

## 🎯 인터페이스 (Topics & Services)

### 구독 (Subscribe)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `pose_topic` (기본: `/amr1/bcd_pose`) | PoseStamped | 센서 포즈 입력 |
| `/line_drive/relative_x_goal` | Float64 | 상대 Δx 목표 |
| `/line_drive/absolute_x_goal` | Float64 | 절대 x 목표 |
| `/line_drive/go_forward` | Bool | 수동 전진 펄스 (hold) |
| `/line_drive/go_backward` | Bool | 수동 후진 펄스 (hold) |

### 퍼블리시 (Publish)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `cmd_topic` (기본: `/cmd_vel`) | Twist | 속도 명령 (vx, vy, wz) |

### 서비스 (Services)
| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/line_drive/align_to_line` | Trigger | Yaw 정렬 시작 (ALIGN_YAW 흐름) |
| `/line_drive/nudge_forward` | Trigger | Slow start 전진 단발 (마이크로) |
| `/line_drive/align_y_only` | Trigger | (홀로노믹) Y-only 정렬 (필요시 yaw → y) |

### 토픽 명령 (Goal 예시)

```bash
# (상대) +1.2 m 전진
ros2 topic pub /line_drive/relative_x_goal std_msgs/Float64 '{data: 1.2}'

# (절대) x = 5.0 m 위치까지 이동
ros2 topic pub /line_drive/absolute_x_goal std_msgs/Float64 '{data: 5.0}'
```

### 서비스 명령

```bash
# Yaw 정렬만 다시 실행
ros2 service call /line_drive/align_to_line std_srvs/srv/Trigger {}

# 미세 +x 전진 (slow start 속도/시간으로 단발 수행)
ros2 service call /line_drive/nudge_forward std_srvs/srv/Trigger {}
```

---

## 🧪 시뮬레이션 흐름 예시

1. **Dummy PGV 시뮬레이터 실행** → `/cmd_vel` 명령을 받아 PoseStamped를 퍼블리시
2. **Line Drive Controller** → PoseStamped를 받아 제어(정렬 → 전진)
3. **One-shot Goal Node** → 1 초 후 `/line_drive/relative_x_goal` 발행

```bash
ros2 launch pgv_tracing_mode test_line_drive_sim.launch.py \
  holonomic:=false use_relative:=true relative_goal:=2.0
```

실행 후 콘솔에서 다음 순서를 볼 수 있습니다:

* `[align] yaw aligned; entering slow-start`
* `[slow-start] complete; entering RUNNING`
* `[goal] reached`

---

## 📚 디렉토리 구조

```
pgv_tracing_mode/
├── pgv_tracing_mode/
│   ├── line_tracing_node.py      # 핵심 LineDriveNode 구현
│   ├── sim_dummy_pgv.py          # 더미 PGV 시뮬레이터
│   ├── one_shot_goal.py          # 단발 목표/서비스 테스트 노드
│   └── __init__.py
├── launch/
│   ├── line_drive.launch.py
│   └── test_line_drive_sim.launch.py
├── config/
│   ├── line_drive.params.yaml
│   └── sim.params.yaml
└── test/ (lint / style 테스트)
```

---

## 🧠 설계 메모 (Design Notes)

* **센서 기준 제어 (Default)**
  센서가 라인 중앙에 오도록 제어 → PGV 데이터를 직접 사용하므로 반응 빠름, 설정 단순.
  오프셋 0으로 두면 로봇은 “센서 중심이 라인 위”가 되는 위치에서 멈춤.

* **로봇 중심 기준 제어 (Optional)**
  만약 “로봇 중심(base_link)”을 라인 중앙에 정렬해야 한다면,
  센서의 실제 장착 위치를 `sensor_x_offset`, `sensor_y_offset`에 입력해 보정.

* **Holonomic vs Non-holonomic**
  Holonomic → 측면 이동 가능 플랫폼(예: Mecanum, Omni).
  Non-holonomic → 차동·조향형(예: Double Steering Drive).
  동일한 제어 구조로 모두 대응.

* **Reverse Heading 허용**
  라인 방향성이 없는 경우 `allow_reverse_heading=true` 로 설정하면 +X 또는 -X(±π) 중 더 가까운 방향을 정렬 목표로 삼아 최소 회전량만 수행합니다. 긴 회전 시간을 줄이고 ±π 근처 불안정한 flip을 wrap band 로 억제합니다.

* **Timeout Grace 복구**
  센서 일시 끊김 후 바로 폭주하는 상황을 방지하기 위해 pose 회복 후 짧은 유예시간을 두어 제어 입력 안정화/재평가 시간을 확보합니다.

* **Yaw Error 음의 피드백**
  yaw_err = (yaw_meas - target) 정의 후 w = -kp_yaw * yaw_err 로 단순/직관 음의 피드백을 적용하여 부호 혼동을 제거했습니다.

---

## 📞 문의 (Contact)

**Maintainer:** 손재락 (Jaerak Son)  
📧 **jr@pitin-ev.com**

이슈 / 개선 제안은 GitHub Issues 로 남겨주세요.

---

### 라이선스
특별한 명시가 없다면 패키지 내 소스는 Apache 2.0 호환 라이선스(ROS 2 기본 예시 기반)로 간주됩니다.
프로젝트 요구에 따라 LICENSE 파일을 추가하세요.

### 변경 로그 제안
향후 변경 사항은 `CHANGELOG.rst` (ament 표준) 작성 권장.

---

Happy tracing! 🚀
