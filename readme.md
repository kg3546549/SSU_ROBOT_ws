# SSU Robot Control System (RCS)

**ROS1 Melodic 기반 Yahboom Rosmaster X3 Plus 제어 시스템**

## 프로젝트 개요

순천향대학교 디지털미디어학과 로봇동아리의 Yahboom Rosmaster X3 Plus 로봇을 제어하기 위한 ROS1 기반 제어 시스템입니다. 모드 기반 아키텍처를 통해 다양한 제어 방식(키보드, 웹 조이스틱, 자율주행 등)을 지원하며, 중앙 집중식 제어 노드가 모드에 따라 적절한 입력 소스를 선택하여 로봇을 제어합니다.

### 주요 특징

- **모듈식 모드 관리**: 7가지 동작 모드 지원 (IDLE, AUTO_PATROL, AUTO_NAVIGATION, KEYBOARD, WEB_JOY, JOY, EMERGENCY)
- **중앙 집중식 제어**: Central Node가 모드에 따라 입력 소스를 선택적으로 처리
- **JSON 기반 서비스 통신**: ROS Service를 통한 모드 변경 및 조회
- **자율 순찰 기능**: PID 제어 기반 벽 추종 알고리즘 구현
- **웹 기반 원격 제어**: ROS Bridge WebSocket을 통한 웹 인터페이스 지원
- **실시간 센서 통합**: IMU, LiDAR, Odometry 데이터 처리

---

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     Mode Manager Node                        │
│  - 모드 상태 관리 (RobotMode Enum)                             │
│  - ROS Service (/mode/req) 제공                              │
│  - 모드 변경 브로드캐스트 (/mode/broad)                         │
└────────────────────┬────────────────────────────────────────┘
                     │ Mode Broadcast
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    Central Control Node                      │
│  - 입력 소스 라우팅 (모드별 선택적 처리)                          │
│  - 속도 명령 통합 및 발행 (/cmd_vel)                            │
│  - 긴급 정지 처리                                              │
└─────┬───────┬───────┬──────────┬──────────┬────────────────┘
      │       │       │          │          │
   [키보드] [웹조이] [조이스틱] [순찰노드] [네비게이션]
      │       │       │          │          │
      ▼       ▼       ▼          ▼          ▼
   /keyboard  /web/   /joy    /patrol/   /navigation/
              cmd_vel        cmd_vel     cmd_vel
```

### 데이터 흐름

1. **모드 변경 요청** → Mode Manager Service 호출
2. **모드 브로드캐스트** → Central Node가 현재 모드 업데이트
3. **입력 수신** → 각 입력 소스(키보드, 웹, 조이스틱 등)에서 명령 수신
4. **선택적 처리** → Central Node가 현재 모드에 해당하는 입력만 처리
5. **속도 명령 발행** → `/cmd_vel` 토픽으로 로봇 하드웨어 제어

---

## 패키지 구조

### 1. **mode_manager**
모드 상태 관리 및 서비스 제공

**주요 기능:**
- JSON 기반 ROS Service 인터페이스 (`/mode/req`)
- 모드 변경 브로드캐스트 (`/mode/broad`)
- 7가지 로봇 동작 모드 관리

**제공 서비스:**
```bash
# 현재 모드 조회
rosservice call /mode/req "{request_data: '{\"command\": \"get\"}'}"

# 모드 변경 (예: 자동 순찰 모드로 변경)
rosservice call /mode/req "{request_data: '{\"command\": \"set\", \"mode\": 1, \"speed\": 0.5}'}"

# 사용 가능한 모드 목록 조회
rosservice call /mode/req "{request_data: '{\"command\": \"list\"}'}"
```

**모드 정의:**
| 모드 값 | 이름 | 설명 |
|--------|------|------|
| 0 | IDLE | 대기 상태 |
| 1 | AUTO_PATROL | 자동 순찰 (벽 추종) |
| 2 | AUTO_NAVIGATION | 자동 네비게이션 |
| 3 | KEYBOARD | 키보드 제어 |
| 4 | WEB_JOY | 웹 조이스틱 제어 |
| 5 | JOY | 물리 조이스틱 제어 |
| 99 | EMERGENCY | 비상 정지 |

---

### 2. **central_control**
중앙 집중식 제어 및 입력 라우팅

**주요 기능:**
- 모드별 입력 소스 선택적 처리
- 통합 속도 명령 발행 (`/cmd_vel`)
- 긴급 정지 즉시 처리

**입력 소스:**
- `/keyboard` (String) - 키보드 키 입력
- `/web/cmd_vel` (Twist) - 웹 조이스틱 명령
- `/joy` (Joy) - 물리 조이스틱 명령
- `/patrol/cmd_vel` (Twist) - 자율 순찰 명령

**키보드 제어 키맵:**
```
    i       u       o
    ↑     ↺ ↑ ↻     ↑

j ← • → l   k   , (후진)
               ↓
```
- `i`: 전진, `,`: 후진
- `j`: 왼쪽 이동, `l`: 오른쪽 이동
- `u`: 좌회전, `o`: 우회전
- `k` / `Space`: 정지

---

### 3. **patrol_node**
자율 순찰 및 벽 추종 알고리즘

**주요 기능:**
- PID 제어 기반 벽 추종
- 상태 머신 기반 순찰 로직
- LiDAR 기반 장애물 회피

**상태 머신:**
```
INIT → FIND_WALL → FOLLOW_WALL ⇄ TURN_LEFT
                         ↓              ↓
                    TURN_RIGHT ← OBSTACLE_WAIT
```

**PID 파라미터:**
- Kp: 1.0 (비례 게인)
- Ki: 0.05 (적분 게인)
- Kd: 0.3 (미분 게인)
- 목표 벽 거리: 0.5m
- 전진 속도: 0.2 m/s

**알고리즘 특징:**
1. **벽 추종**: 우측 벽과 일정 거리 유지하며 전진
2. **장애물 회피**: 전방 장애물 감지 시 2초 대기 후 좌회전
3. **모퉁이 감지**: 우측 빈 공간 감지 시 우회전
4. **적응형 회전**: 센서 데이터 기반 회전 각도 자동 조정

---

### 4. **keyboard_control**
키보드 입력 수신 및 발행

**주요 기능:**
- 논블로킹 키 입력 처리
- `/keyboard` 토픽으로 키 발행
- Terminal raw 모드 설정/복원

---

### 5. **yahboomcar_bringup**
Yahboom 로봇 하드웨어 인터페이스

**주요 기능:**
- 로봇 하드웨어 드라이버 (Rosmaster X3 Plus)
- IMU, Odometry, 모터 제어
- 센서 데이터 발행 (LiDAR, IMU)

---

### 6. **common**
공통 라이브러리 및 유틸리티

**제공 모듈:**
- `modes.py`: RobotMode Enum 정의 및 유틸리티 함수
- `constants.py`: Topic/Service 이름 상수
- `utils.py`: JSON 응답 생성 등 공통 함수

---

## 설치 및 실행

### 시스템 요구사항
- Ubuntu 18.04 (Bionic)
- ROS Melodic
- Python 3.6+
- Yahboom Rosmaster X3 Plus

### 의존성 설치
```bash
# ROS Melodic 설치 (Ubuntu 18.04)
sudo apt update
sudo apt install ros-melodic-desktop-full

# 필수 패키지 설치
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install ros-melodic-rosbridge-server
sudo apt install ros-melodic-robot-state-publisher
sudo apt install ros-melodic-joint-state-publisher

# rosdep 초기화
sudo rosdep init
rosdep update
```

### 빌드
```bash
cd ~/SSU_ROBOT_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

### 실행 방법

#### 1. 전체 시스템 실행 (로봇 하드웨어 + 제어 시스템)
```bash
# 환경 변수 설정
export ROBOT_TYPE=X3plus

# 로봇 드라이버 + 중앙 제어 + 모드 관리자 + ROS Bridge 실행
roslaunch central_control centralNode.launch
```

#### 2. 키보드 제어 모드
```bash
# Terminal 1: 전체 시스템 실행
roslaunch central_control centralNode.launch

# Terminal 2: 키보드 노드 실행
rosrun keyboard_control keyboard.py

# Terminal 3: 키보드 모드로 변경
rosservice call /mode/req "{request_data: '{\"command\": \"set\", \"mode\": 3}'}"
```

#### 3. 자율 순찰 모드
```bash
# Terminal 1: 전체 시스템 실행
roslaunch central_control centralNode.launch

# Terminal 2: 순찰 노드 실행
rosrun patrol_node patrol_node.py

# Terminal 3: 자동 순찰 모드로 변경
rosservice call /mode/req "{request_data: '{\"command\": \"set\", \"mode\": 1, \"speed\": 0.3}'}"
```

#### 4. 웹 조이스틱 모드
```bash
# Terminal 1: 전체 시스템 실행 (ROS Bridge 포함)
roslaunch central_control centralNode.launch

# Terminal 2: 웹 조이스틱 모드로 변경
rosservice call /mode/req "{request_data: '{\"command\": \"set\", \"mode\": 4}'}"

# 웹 브라우저에서 ws://localhost:9090 접속
```

---

## 주요 토픽 및 서비스

### 토픽

| 토픽 이름 | 타입 | 설명 |
|----------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 로봇 속도 명령 (최종 출력) |
| `/mode/broad` | std_msgs/String | 현재 모드 브로드캐스트 (JSON) |
| `/keyboard` | std_msgs/String | 키보드 키 입력 |
| `/web/cmd_vel` | geometry_msgs/Twist | 웹 조이스틱 명령 |
| `/joy` | sensor_msgs/Joy | 물리 조이스틱 명령 |
| `/patrol/cmd_vel` | geometry_msgs/Twist | 자율 순찰 명령 |
| `/scan` | sensor_msgs/LaserScan | LiDAR 스캔 데이터 |
| `/imu/imu_raw` | sensor_msgs/Imu | IMU 센서 데이터 |

### 서비스

| 서비스 이름 | 타입 | 설명 |
|-----------|------|------|
| `/mode/req` | mode_manager/ModeRequest | 모드 관리 서비스 (get/set/list) |

---

## 기술 스택

### ROS & 로보틱스
- **ROS Melodic**: 로봇 운영 체제
- **Catkin**: 빌드 시스템
- **TF**: 좌표 변환
- **ROS Bridge**: WebSocket 기반 웹 인터페이스

### 제어 알고리즘
- **PID 제어**: 벽 추종을 위한 비례-적분-미분 제어
- **상태 머신**: 순찰 로직 구현
- **센서 융합**: LiDAR + IMU + Odometry

### 프로그래밍
- **Python 3**: 주 개발 언어
- **JSON**: 서비스 통신 데이터 포맷
- **Enum**: 타입 안전한 모드 관리

---

## 아키텍처 설계 철학

### 1. 모듈화 (Modularity)
각 기능을 독립적인 ROS 패키지로 분리하여 유지보수성과 확장성을 확보

### 2. 단일 책임 원칙 (Single Responsibility)
- Mode Manager: 모드 관리만 담당
- Central Control: 입력 라우팅만 담당
- Patrol Node: 순찰 알고리즘만 담당

### 3. 느슨한 결합 (Loose Coupling)
ROS 토픽과 서비스를 통한 간접 통신으로 모듈 간 의존성 최소화

### 4. 확장성 (Extensibility)
새로운 모드나 입력 소스 추가 시 기존 코드 수정 최소화

---

## 트러블슈팅

### 1. 모드가 변경되지 않음
```bash
# Mode Manager 노드 상태 확인
rosnode info /mode_manager

# 토픽 수신 확인
rostopic echo /mode/broad
```

### 2. 로봇이 움직이지 않음
```bash
# cmd_vel 토픽 발행 확인
rostopic echo /cmd_vel

# 현재 모드 확인
rosservice call /mode/req "{request_data: '{\"command\": \"get\"}'}"

# IDLE 모드면 다른 모드로 변경
rosservice call /mode/req "{request_data: '{\"command\": \"set\", \"mode\": 3}'}"
```

### 3. 순찰 중 벽에 부딪힘
- PID 파라미터 조정 필요 (patrol_node.py의 PIDController 인자)
- 목표 벽 거리 증가 (wall_distance 파라미터)
- 장애물 감지 거리 증가 (obstacle_threshold 파라미터)

---

## 향후 개선 계획

- [ ] SLAM 기반 자율 네비게이션 구현
- [ ] 장애물 회피 알고리즘 고도화 (DWA, TEB)
- [ ] 웹 대시보드 개발 (React + Chakra UI)
- [ ] 다중 로봇 협업 시스템
- [ ] ROS2 포팅

---

## 라이선스

이 프로젝트는 교육 목적으로 개발되었습니다.

---

## 개발자

**순천향대학교 디지털미디어학과 로봇동아리**

---

## 참고 자료

- [ROS Melodic Documentation](http://wiki.ros.org/melodic)
- [Yahboom Rosmaster X3 Plus](http://www.yahboom.net/study/ROS-X3-plus)
- [ROS Bridge Suite](http://wiki.ros.org/rosbridge_suite)
- [PID Control Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
