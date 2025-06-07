#!/bin/bash

echo "=== 빠른 설정 검증 ==="
echo "Docker 빌드 전에 설정 파일들을 검증합니다."
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

check_file() {
    local file="$1"
    local description="$2"
    
    echo -n "확인: $description ... "
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗ 파일이 없습니다: $file${NC}"
        return 1
    fi
}

check_content() {
    local file="$1"
    local pattern="$2"
    local description="$3"
    
    echo -n "확인: $description ... "
    if [ -f "$file" ] && grep -q "$pattern" "$file"; then
        echo -e "${GREEN}✓${NC}"
        return 0
    else
        echo -e "${RED}✗${NC}"
        return 1
    fi
}

echo "1. 필수 파일 존재 확인"
echo "===================="
check_file ".devcontainer/Dockerfile" "Dockerfile"
check_file ".devcontainer/devcontainer.json" "devcontainer.json"
check_file ".devcontainer/setup.sh" "setup.sh"
check_file ".devcontainer/tutorial.repos" "tutorial.repos"
echo ""

echo "2. Dockerfile 내용 확인"
echo "====================="
check_content ".devcontainer/Dockerfile" "ros:humble-desktop-full" "ROS2 Humble Desktop 베이스 이미지"
check_content ".devcontainer/Dockerfile" "gz-garden" "Gazebo Garden 설치"
check_content ".devcontainer/Dockerfile" "ros-humble-moveit" "MoveIt2 패키지"
check_content ".devcontainer/Dockerfile" "ros-humble-ros2-control" "ros2_control 패키지"
echo ""

echo "3. tutorial.repos 내용 확인"
echo "=========================="
check_content ".devcontainer/tutorial.repos" "franka_ros2" "Franka ROS2 패키지"
check_content ".devcontainer/tutorial.repos" "moveit2" "MoveIt2 소스"
check_content ".devcontainer/tutorial.repos" "ros_gz" "ROS-Gazebo 브리지"
echo ""

echo "4. devcontainer.json 설정 확인"
echo "=============================="
check_content ".devcontainer/devcontainer.json" "X11" "X11 포워딩 설정"
check_content ".devcontainer/devcontainer.json" "privileged" "권한 설정"
check_content ".devcontainer/devcontainer.json" "/ws" "워크스페이스 마운트"
echo ""

echo "5. setup.sh 스크립트 확인"
echo "======================="
check_content ".devcontainer/setup.sh" "/ws" "올바른 워크스페이스 경로"
check_content ".devcontainer/setup.sh" "franka" "Franka 빌드 단계"
check_content ".devcontainer/setup.sh" "gazebo_ros2_control" "Gazebo control 빌드"
echo ""

echo "6. 실행 권한 확인"
echo "================"
if [ -x ".devcontainer/setup.sh" ]; then
    echo -e "setup.sh 실행 권한: ${GREEN}✓${NC}"
else
    echo -e "setup.sh 실행 권한: ${RED}✗${NC}"
    echo "  다음 명령어로 권한을 설정하세요: chmod +x .devcontainer/setup.sh"
fi

if [ -f "test_environment.sh" ]; then
    if [ -x "test_environment.sh" ]; then
        echo -e "test_environment.sh 실행 권한: ${GREEN}✓${NC}"
    else
        echo -e "test_environment.sh 실행 권한: ${RED}✗${NC}"
        echo "  다음 명령어로 권한을 설정하세요: chmod +x test_environment.sh"
    fi
fi
echo ""

echo "=== 검증 완료 ==="
echo ""
echo -e "${GREEN}다음 단계:${NC}"
echo "1. Linux에서 X11 허용: xhost +local:docker"
echo "2. VS Code에서 Dev Container 열기:"
echo "   - Ctrl+Shift+P"
echo "   - 'Dev Containers: Rebuild and Reopen in Container' 선택"
echo "3. 컨테이너 내에서 테스트 실행:"
echo "   - chmod +x test_environment.sh"
echo "   - ./test_environment.sh"
echo ""
echo -e "${YELLOW}주의사항:${NC}"
echo "- 첫 빌드는 30-60분 정도 소요될 수 있습니다"
echo "- 인터넷 연결이 안정적이어야 합니다"
echo "- 충분한 디스크 공간(최소 10GB)이 필요합니다" 