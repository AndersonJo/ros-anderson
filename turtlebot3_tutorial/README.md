# ROS2 + TurtleBot3 + Reinforcement Learning Development Environment

이 프로젝트는 **ROS2 Humble + TurtleBot3 + Reinforcement Learning**을 위한 완전한 개발 환경을 제공합니다.

## 🚀 빠른 시작

### 1. DevContainer로 환경 설정

```bash
# 1. 프로젝트 클론
git clone <your-repo-url>
cd tutorial

# 2. VS Code/Cursor에서 열기
code .  # 또는 cursor .

# 3. DevContainer에서 열기
# Ctrl+Shift+P → "Dev Containers: Rebuild and Reopen in Container"
```

### 2. 워크스페이스 초기화

DevContainer가 시작되면 자동으로 환경이 설정됩니다.

### 3. TurtleBot3 테스트

```bash
# 환경 소스 (DevContainer 내에서)
source setup_env.sh

# 또는 수동으로
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# TurtleBot3 Gazebo 시뮬레이션 시작
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 또는 간단한 테스트 런치 (별도 터미널)
ros2 launch test_turtlebot3_simple.launch.py

# 키보드로 조작 (별도 터미널)
ros2 run turtlebot3_teleop teleop_keyboard
```

## 🔧 문제 해결

### DevContainer 빌드 실패 시

1. **Python distutils 오류**:
   - Dockerfile에서 `python3-distutils` 패키지가 설치되어 있는지 확인
   - Python 3.12 호환성 문제일 수 있음

2. **네트워크 연결 문제**:
   - DevContainer가 `--network=bridge` 설정을 사용
   - DNS 서버가 8.8.8.8, 8.8.4.4로 설정됨

3. **ROS2 환경 문제**:
   ```bash
   # 호스트 시스템에서 간단한 테스트
   python3 simple_test.py
   ```

### 호스트 시스템에서 개발하기

DevContainer에 문제가 있는 경우:

```bash
# 1. 기본 환경 테스트
python3 simple_test.py

# 2. ROS2 설치 (Ubuntu 22.04)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# 3. TurtleBot3 패키지 설치
sudo apt install ros-humble-turtlebot3*
```

## 📁 프로젝트 구조

```
tutorial/
├── .devcontainer/           # DevContainer 설정
│   ├── devcontainer.json    # 메인 설정
│   ├── Dockerfile          # 커스텀 Docker 이미지
│   ├── setup-turtlebot3-rl.sh  # 환경 설정 스크립트
│   └── turtlebot3_rl.repos # TurtleBot3 패키지 목록
├── src/                     # ROS2 패키지들
│   └── turtlebot3_rl/      # 강화학습 환경 패키지
├── build/                   # 빌드 파일들 (gitignore됨)
├── install/                 # 설치 파일들 (gitignore됨)
├── log/                     # 로그 파일들 (gitignore됨)
├── requirements_rl.txt      # Python RL 패키지 목록
├── test_turtlebot3_simple.launch.py  # 간단한 테스트 런치
└── README.md               # 이 파일
```

## 🛠️ 포함된 패키지들

### 🤖 TurtleBot3
- TurtleBot3 robot description
- Gazebo 시뮬레이션
- Navigation2 스택
- Cartographer SLAM
- Teleop 제어

### 🧠 Reinforcement Learning
- **PyTorch**: 딥러닝 프레임워크
- **Stable-Baselines3**: RL 알고리즘 라이브러리
- **Gymnasium**: RL 환경 인터페이스
- **TensorBoard**: 학습 모니터링
- **Weights & Biases**: 실험 추적

### 🌍 Navigation & Control
- Navigation2 framework
- SLAM (Cartographer)
- Path planning
- Obstacle avoidance
- 센서 처리 (LiDAR, IMU)

### 🎮 ROS2 Communication
- Topics, Services, Actions
- tf2 transforms
- Sensor message handling
- Geometry utilities

## 🔧 유용한 명령어들

### 빌드 관련
```bash
cb                    # colcon build (별칭)
cbp <package_name>    # 특정 패키지만 빌드
cbt                   # colcon test
```

### 환경 설정
```bash
source_ros           # ROS2 환경 소스
source_ws            # 워크스페이스 환경 소스
export TURTLEBOT3_MODEL=burger  # TurtleBot3 모델 설정
```

### TurtleBot3 관련
```bash
tb3_sim              # TurtleBot3 Gazebo 시뮬레이션 (별칭)
tb3_nav              # Navigation2 시작 (별칭)

# 수동 명령어들
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
ros2 run turtlebot3_teleop teleop_keyboard
```

### 강화학습 개발
```bash
# Python 환경 확인
python3 -c "import torch, stable_baselines3, gymnasium; print('RL packages OK')"

# TensorBoard 시작
tensorboard --logdir=./logs

# 강화학습 환경 노드 실행 (개발 예정)
ros2 run turtlebot3_rl rl_environment
```

### 디버깅
```bash
ros2 node list       # 실행 중인 노드 확인
ros2 topic list      # 토픽 목록
ros2 topic echo /scan # LiDAR 데이터 확인
ros2 topic echo /odom # 오도메트리 확인
rviz2               # RViz 시각화
```

## 🧠 강화학습 개발 가이드

### 1. 기본 환경 구조
```python
# turtlebot3_rl/rl_environment.py
import rclpy
import gymnasium as gym
from stable_baselines3 import PPO

class TurtleBot3RLEnv(gym.Env):
    def __init__(self):
        # ROS2 노드 초기화
        # 액션/관측 공간 정의
        # 시뮬레이션 환경 설정
        pass
    
    def step(self, action):
        # 액션 실행 (/cmd_vel 퍼블리시)
        # 관측 데이터 수집 (/scan, /odom)
        # 보상 계산
        # 종료 조건 확인
        return observation, reward, done, info
```

### 2. 학습 목표 예시
- **목표 도달**: 특정 위치로 이동
- **장애물 회피**: LiDAR 데이터로 충돌 방지
- **최단 경로**: 효율적인 내비게이션
- **배터리 최적화**: 에너지 효율적인 이동

### 3. 개발 단계
1. **환경 설정**: Gazebo + TurtleBot3 동작 확인
2. **센서 데이터**: /scan, /odom 토픽 구독
3. **제어 인터페이스**: /cmd_vel 토픽 퍼블리시
4. **보상 함수**: 목표 달성도 기반 설계
5. **학습 알고리즘**: PPO, SAC, DQN 등 적용

## 📦 패키지 추가하기

### 새로운 TurtleBot3 패키지
`.devcontainer/turtlebot3_rl.repos`에 추가:

```yaml
repositories:
  my_rl_package:
    type: git
    url: https://github.com/user/my_rl_package.git
    version: humble
```

### 강화학습 Python 패키지
`requirements_rl.txt`에 추가:
```
new-rl-package>=1.0.0
```

### 직접 개발한 패키지
```bash
cd src
ros2 pkg create --build-type ament_python my_rl_package
```

## 🐳 DevContainer 설정

### 주요 특징
- **베이스 이미지**: `osrf/ros:humble-desktop-full`
- **TurtleBot3**: 모든 관련 패키지 사전 설치
- **강화학습**: PyTorch, Stable-Baselines3, Gymnasium
- **Python 지원**: Python 3.12 호환, 최신 ML 라이브러리
- **GUI 지원**: X11 전달, Gazebo 시각화

### 환경 변수
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

## 🎯 다음 단계

1. ✅ **환경 설정 완료**: DevContainer + TurtleBot3
2. 🔄 **TurtleBot3 테스트**: 시뮬레이션 동작 확인  
3. 🚧 **RL 환경 개발**: Gymnasium 환경 구현
4. 🧠 **알고리즘 적용**: PPO, SAC 등으로 학습
5. 📊 **성능 평가**: 학습 결과 분석

## 🐛 문제 해결

### TurtleBot3 모델 에러
```bash
export TURTLEBOT3_MODEL=burger
# 또는 waffle, waffle_pi
```

### Gazebo 실행 안됨
```bash
# GPU 드라이버 확인
nvidia-smi  # NVIDIA GPU인 경우

# Gazebo 모델 경로 확인
echo $GAZEBO_MODEL_PATH
```

### 강화학습 패키지 에러
```bash
# 패키지 재설치
pip3 install --upgrade stable-baselines3[extra] --break-system-packages
```

Ready for TurtleBot3 + Reinforcement Learning! 🤖🧠