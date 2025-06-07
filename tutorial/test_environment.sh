#!/bin/bash

echo "🧪 Testing ROS2 + MoveIt2 + Gazebo + Franka Environment..."
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test function
test_command() {
    local cmd="$1"
    local description="$2"
    
    echo -e "\n🔍 Testing: $description"
    echo "   Command: $cmd"
    
    if eval "$cmd" >/dev/null 2>&1; then
        echo -e "   ${GREEN}✅ PASS${NC}"
        return 0
    else
        echo -e "   ${RED}❌ FAIL${NC}"
        return 1
    fi
}

# Test function with output
test_command_with_output() {
    local cmd="$1"
    local description="$2"
    
    echo -e "\n🔍 Testing: $description"
    echo "   Command: $cmd"
    
    output=$(eval "$cmd" 2>&1)
    if [ $? -eq 0 ]; then
        echo -e "   ${GREEN}✅ PASS${NC}"
        echo "   Output: $output"
        return 0
    else
        echo -e "   ${RED}❌ FAIL${NC}"
        echo "   Output: $output"
        return 1
    fi
}

# Initialize counters
total_tests=0
passed_tests=0

# Source ROS2 environment
echo "🔧 Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Test 1: ROS2 installation
((total_tests++))
if test_command "ros2 --version" "ROS2 installation"; then
    ((passed_tests++))
fi

# Test 2: Gazebo installation
((total_tests++))
if test_command "gz sim --version" "Gazebo installation"; then
    ((passed_tests++))
fi

# Test 3: MoveIt2 packages
((total_tests++))
if test_command "ros2 pkg list | grep moveit" "MoveIt2 packages"; then
    ((passed_tests++))
fi

# Test 4: ros2_control packages
((total_tests++))
if test_command "ros2 pkg list | grep controller" "ros2_control packages"; then
    ((passed_tests++))
fi

# Test 5: Gazebo-ROS2 integration
((total_tests++))
if test_command "ros2 pkg list | grep ros_gz" "Gazebo-ROS2 integration"; then
    ((passed_tests++))
fi

# Test 6: Panda moveit config
((total_tests++))
if test_command "ros2 pkg list | grep panda" "Panda MoveIt config"; then
    ((passed_tests++))
fi

# Test 7: Check if launch files exist
((total_tests++))
if test_command "ls /workspace/launch/panda_*.launch.py" "Custom launch files"; then
    ((passed_tests++))
fi

# Test 8: Environment variables
((total_tests++))
if test_command_with_output "echo \$ROS_DISTRO" "ROS_DISTRO environment variable"; then
    ((passed_tests++))
fi

# Test 9: Python packages
((total_tests++))
if test_command "python3 -c 'import moveit_configs_utils'" "Python MoveIt configs"; then
    ((passed_tests++))
fi

# Test 10: Workspace structure
((total_tests++))
if test_command "ls /workspace/src" "Workspace source directory"; then
    ((passed_tests++))
fi

# Summary
echo ""
echo "=============================================="
echo "🏁 Test Summary"
echo "=============================================="
echo "Total tests: $total_tests"
echo -e "Passed: ${GREEN}$passed_tests${NC}"
echo -e "Failed: ${RED}$((total_tests - passed_tests))${NC}"

if [ $passed_tests -eq $total_tests ]; then
    echo ""
    echo -e "${GREEN}🎉 All tests passed! Environment is ready!${NC}"
    echo ""
    echo "🚀 Quick start commands:"
    echo "   panda_demo              - Basic MoveIt2 demo"
    echo "   panda_gazebo           - Gazebo simulation"
    echo "   panda_gazebo_headless  - Headless Gazebo"
    echo ""
    exit 0
else
    echo ""
    echo -e "${RED}❌ Some tests failed. Please check the setup.${NC}"
    echo ""
    echo "🔧 Try running the setup script again:"
    echo "   /workspace/.devcontainer/setup-full-environment.sh"
    exit 1
fi 