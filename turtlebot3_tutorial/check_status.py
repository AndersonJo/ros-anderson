#!/usr/bin/env python3
"""
Environment Status Checker
현재 개발 환경의 상태를 확인하는 스크립트
"""
import os
import sys
import subprocess
import shutil

def check_command(cmd):
    """Check if a command is available"""
    return shutil.which(cmd) is not None

def run_command(cmd):
    """Run a command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.returncode == 0, result.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, "Timeout"
    except Exception as e:
        return False, str(e)

def check_python_packages():
    """Check Python packages"""
    packages = ['numpy', 'rclpy', 'torch', 'stable_baselines3', 'gymnasium']
    results = {}
    
    for pkg in packages:
        try:
            __import__(pkg)
            if pkg == 'numpy':
                import numpy as np
                results[pkg] = f"✅ {np.__version__}"
            elif pkg == 'torch':
                import torch
                results[pkg] = f"✅ {torch.__version__}"
            else:
                results[pkg] = "✅ Available"
        except ImportError:
            results[pkg] = "❌ Not available"
    
    return results

def main():
    print("🔍 ROS2 + TurtleBot3 + RL Environment Status Check")
    print("=" * 60)
    
    # Python environment
    print(f"\n🐍 Python Environment:")
    print(f"   Version: {sys.version}")
    print(f"   Executable: {sys.executable}")
    
    # Python packages
    print(f"\n📦 Python Packages:")
    pkg_results = check_python_packages()
    for pkg, status in pkg_results.items():
        print(f"   {pkg}: {status}")
    
    # System commands
    print(f"\n🛠️ System Commands:")
    commands = {
        'ros2': 'ROS2 CLI',
        'colcon': 'Colcon build tool',
        'gazebo': 'Gazebo simulator',
        'rviz2': 'RViz2 visualization',
        'rosdep': 'ROS dependency manager'
    }
    
    for cmd, desc in commands.items():
        if check_command(cmd):
            print(f"   ✅ {cmd} ({desc})")
        else:
            print(f"   ❌ {cmd} ({desc}) - Not found")
    
    # ROS2 environment
    print(f"\n🤖 ROS2 Environment:")
    ros_distro = os.environ.get('ROS_DISTRO', 'Not set')
    print(f"   ROS_DISTRO: {ros_distro}")
    
    if check_command('ros2'):
        success, output = run_command('ros2 --version')
        if success:
            print(f"   Version: {output}")
        else:
            print(f"   Version check failed: {output}")
    
    # TurtleBot3 environment
    print(f"\n🐢 TurtleBot3 Environment:")
    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'Not set')
    print(f"   TURTLEBOT3_MODEL: {tb3_model}")
    
    # Workspace structure
    print(f"\n📁 Workspace Structure:")
    dirs = ['src', 'build', 'install', 'log']
    for d in dirs:
        if os.path.exists(d):
            contents = len(os.listdir(d)) if os.path.isdir(d) else 0
            print(f"   ✅ {d}/ ({contents} items)")
        else:
            print(f"   ❌ {d}/ (missing)")
    
    # DevContainer files
    print(f"\n🐳 DevContainer Files:")
    devcontainer_files = [
        '.devcontainer/devcontainer.json',
        '.devcontainer/Dockerfile',
        '.devcontainer/setup-simple.sh',
        '.devcontainer/setup-x11.sh'
    ]
    
    for f in devcontainer_files:
        if os.path.exists(f):
            print(f"   ✅ {f}")
        else:
            print(f"   ❌ {f} (missing)")
    
    # Recommendations
    print(f"\n💡 Recommendations:")
    
    if ros_distro == 'Not set':
        print("   🔧 ROS2 environment not sourced - run: source /opt/ros/humble/setup.bash")
    
    if tb3_model == 'Not set':
        print("   🔧 TurtleBot3 model not set - run: export TURTLEBOT3_MODEL=burger")
    
    if not check_command('ros2'):
        print("   🔧 ROS2 not installed - consider using DevContainer or install ROS2 Humble")
    
    if not os.path.exists('install/setup.bash'):
        print("   🔧 Workspace not built - run: colcon build")
    
    print(f"\n🎯 Next Steps:")
    if check_command('ros2'):
        print("   1. Source environment: source setup_env.sh (if in DevContainer)")
        print("   2. Build workspace: colcon build")
        print("   3. Test ROS2: ros2 topic list")
        print("   4. Test TurtleBot3: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py")
    else:
        print("   1. Setup DevContainer or install ROS2 Humble")
        print("   2. Run this script again to verify installation")
    
    print("\n" + "=" * 60)
    print("Status check complete! 🚀")

if __name__ == '__main__':
    main() 