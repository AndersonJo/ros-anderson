# ROS2 + Gazebo + MoveIt2 + Franka Docker 환경 테스트 가이드

## 사전 검증 완료! ✅

빠른 검증 테스트가 성공적으로 완료되었습니다. 모든 설정 파일이 올바르게 구성되어 있습니다.

## 테스트 절차

### 1단계: X11 포워딩 설정 (Linux)

```bash
# Docker 컨테이너가 GUI 애플리케이션을 표시할 수 있도록 허용
xhost +local:docker
```

### 2단계: Dev Container 빌드 및 실행

1. **VS Code 열기**: 프로젝트 폴더를 VS Code에서 열기
2. **Command Palette**: `Ctrl+Shift+P` (또는 `Cmd+Shift+P` on Mac)
3. **Container 명령어**: "Dev Containers: Rebuild and Reopen in Container" 선택
4. **대기**: 첫 빌드는 30-60분 정도 소요됩니다

### 3단계: 컨테이너 내에서 환경 테스트

컨테이너가 시작되면 터미널에서 다음을 실행:

```bash
# 테스트 스크립트 실행
./test_environment.sh
```

## 예상 테스트 결과

성공적인 설치 시 다음과 같은 결과를 볼 수 있습니다:

```
=== ROS2 + Gazebo + MoveIt2 + Franka 환경 테스트 ===

1. ROS2 기본 환경 테스트
=========================
테스트: ROS2 설치 확인 ✓ 성공
테스트: Colcon 빌드 도구 확인 ✓ 성공
테스트: ROS2 Humble 환경 확인 ✓ 성공

2. Gazebo 설치 확인
===================
테스트: Gazebo 실행파일 확인 ✓ 성공
테스트: Gazebo 버전 확인 ✓ 성공

... (모든 테스트 통과)
```

## 주요 기능 테스트

### 기본 ROS2 명령어 테스트

```bash
# ROS2 버전 확인
ros2 --version

# 사용 가능한 패키지 목록
ros2 pkg list | grep moveit
ros2 pkg list | grep franka

# 토픽 및 서비스 확인
ros2 topic list
ros2 service list
```

### Gazebo 테스트

```bash
# Gazebo 실행 (GUI)
gz sim

# Gazebo 빈 월드로 실행
gz sim empty.sdf
```

### MoveIt2 테스트

```bash
# MoveIt2 데모 실행 (Panda 로봇)
ros2 launch moveit2_tutorials demo.launch.py

# RViz2 실행
rviz2
```

### Franka 관련 패키지 확인

```bash
# Franka 관련 패키지 목록
ros2 pkg list | grep franka

# Franka description 확인
ros2 pkg list | grep franka_description
```

## GUI 애플리케이션 테스트

### RViz2 테스트
```bash
rviz2
```
- 3D 시각화 창이 표시되어야 함
- 메뉴와 패널들이 정상적으로 보여야 함

### Gazebo 테스트
```bash
gz sim
```
- Gazebo 시뮬레이터 창이 표시되어야 함
- 3D 렌더링이 정상적으로 작동해야 함

### Joint State Publisher GUI
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
- 슬라이더가 있는 제어 창이 표시되어야 함

## 문제 해결

### 1. GUI가 표시되지 않는 경우

**원인**: X11 포워딩 문제
**해결책**:
```bash
# 호스트에서 실행
echo $DISPLAY  # 출력 확인
xhost +local:docker

# 컨테이너 내에서 실행
echo $DISPLAY  # :0 이어야 함
export DISPLAY=:0  # 필요시 수동 설정
```

### 2. 빌드 에러가 발생하는 경우

**원인**: 의존성 문제 또는 네트워크 문제
**해결책**:
```bash
# 컨테이너 내에서 의존성 재설치
cd /ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# 클린 빌드
rm -rf build install log
colcon build
```

### 3. 패키지를 찾을 수 없는 경우

**원인**: 워크스페이스가 소싱되지 않음
**해결책**:
```bash
source /ws/install/setup.bash
```

### 4. Gazebo가 실행되지 않는 경우

**원인**: GPU 드라이버 또는 환경 변수 문제
**해결책**:
```bash
# 환경 변수 확인
env | grep GZ
env | grep DISPLAY

# GPU 정보 확인 (호스트에서)
lspci | grep VGA
```

## 성능 벤치마크

### 예상 빌드 시간
- **첫 빌드**: 30-60분 (인터넷 속도에 따라)
- **재빌드**: 5-15분 (변경사항에 따라)

### 시스템 요구사항
- **RAM**: 최소 8GB, 권장 16GB
- **디스크**: 최소 10GB 여유 공간
- **CPU**: 멀티코어 권장 (빌드 시간 단축)

### 디스크 사용량
- **Base Image**: ~2GB
- **빌드된 워크스페이스**: ~3-5GB
- **전체**: ~7-8GB

## 성공 지표

테스트가 성공적으로 완료되면 다음이 가능해야 합니다:

1. ✅ **ROS2 명령어 실행**: `ros2 --version` 등
2. ✅ **Gazebo 실행**: GUI와 시뮬레이션 환경
3. ✅ **RViz2 실행**: 3D 시각화 도구
4. ✅ **MoveIt2 데모**: 로봇 모션 플래닝
5. ✅ **Franka 패키지**: 소스에서 빌드됨
6. ✅ **ros2_control**: 제어 인터페이스 준비
7. ✅ **GUI 애플리케이션**: X11 포워딩 작동

## 다음 단계

환경이 성공적으로 구축되면 다음과 같은 작업을 진행할 수 있습니다:

1. **커스텀 로봇 모델 추가**
2. **MoveIt2 설정 커스터마이징**
3. **Gazebo 시뮬레이션 월드 생성**
4. **Franka 로봇 프로그래밍**
5. **실제 응용 프로그램 개발**

---

**참고**: 이 테스트는 Docker 컨테이너 환경이 올바르게 구성되었는지 확인하는 것입니다. 실제 로봇 하드웨어와의 연결은 별도의 설정이 필요합니다. 