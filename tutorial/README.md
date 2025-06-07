# ROS2 + MoveIt2 + Gazebo + Franka Development Environment

이 프로젝트는 **ROS2 Humble + MoveIt2 + Gazebo Garden + Franka Emika** 로봇을 위한 완전한 개발 환경을 제공합니다.

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

```bash
# DevContainer 내에서 실행
./setup_workspace.sh
```

### 3. 환경 테스트

```bash
# 기본 테스트
source install/setup.bash
ros2 launch launch/test_environment.launch.py

# Panda MoveIt 데모
panda_demo

# Panda + Gazebo 시뮬레이션
panda_gazebo
```

## 📁 프로젝트 구조

```
tutorial/
├── .devcontainer/           # DevContainer 설정
│   ├── devcontainer.json    # 메인 설정
│   ├── setup-full-environment.sh  # 환경 설정 스크립트
│   ├── franka.repos         # Franka 패키지 목록
│   └── tutorial.repos       # 튜토리얼 패키지 목록
├── launch/                  # 런치 파일들
├── src/                     # ROS2 패키지들 (자동 다운로드됨)
├── build/                   # 빌드 파일들 (gitignore됨)
├── install/                 # 설치 파일들 (gitignore됨)
├── log/                     # 로그 파일들 (gitignore됨)
├── setup_workspace.sh       # 워크스페이스 초기화 스크립트
└── README.md               # 이 파일
```

## 🛠️ 포함된 패키지들

### 🤖 MoveIt2
- MoveIt planning framework
- OMPL motion planners
- Visualization tools
- Servo (real-time control)

### 🌍 Gazebo Garden
- 최신 Gazebo 시뮬레이터
- ROS2-Gazebo 브리지
- 물리 시뮬레이션

### 🦾 Franka Emika
- Franka ROS2 패키지
- Panda robot description
- MoveIt 설정
- 예제 컨트롤러

### 🎮 ros2_control
- Hardware interface
- Controllers (position, effort, velocity)
- Controller manager

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
```

### Panda 로봇
```bash
panda_demo           # MoveIt 데모 (RViz만)
panda_gazebo         # Gazebo + MoveIt
panda_gazebo_headless # Headless Gazebo
```

### 디버깅
```bash
ros2 node list       # 실행 중인 노드 확인
ros2 topic list      # 토픽 목록
ros2 service list    # 서비스 목록
rviz2               # RViz 시각화
gz sim              # Gazebo 시뮬레이터
```

## 📦 패키지 추가하기

### 새로운 외부 패키지 추가
`.devcontainer/franka.repos` 또는 `.devcontainer/tutorial.repos`에 추가:

```yaml
repositories:
  my_new_package:
    type: git
    url: https://github.com/user/my_package.git
    version: humble
```

### 직접 개발한 패키지 추가
```bash
cd src
ros2 pkg create --build-type ament_cmake my_package
# 또는
ros2 pkg create --build-type ament_python my_package
```

그리고 `.gitignore`에서 해당 패키지를 제외:
```bash
# .gitignore에 추가
!src/my_package/
```

## 🐛 문제 해결

### DevContainer 빌드 실패
```bash
# Docker 캐시 정리
docker system prune -f
docker builder prune -af

# DevContainer 재빌드
# Ctrl+Shift+P → "Dev Containers: Rebuild Container"
```

### 네트워크 연결 문제
DevContainer 설정에 DNS 서버가 명시되어 있습니다:
```json
"runArgs": [
  "--dns=8.8.8.8",
  "--dns=8.8.4.4"
]
```

### 빌드 의존성 문제
```bash
# 의존성 업데이트
rosdep update
rosdep install --rosdistro humble --from-paths src --ignore-src -r -y

# 클린 빌드
rm -rf build install log
cb
```

## 🤝 기여하기

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 라이선스

이 프로젝트는 [MIT License](LICENSE)를 따릅니다.

## 🆘 도움말

문제가 있으시면 Issues에 등록해주세요:
- [프로젝트 Issues](https://github.com/your-username/your-repo/issues)

---

**Happy Coding with ROS2! 🤖🚀**