#!/usr/bin/env python3
"""
Simple GUI test for TurtleBot3 in Docker container
"""

import subprocess
import sys
import os

def test_display():
    """Test if DISPLAY is available"""
    display = os.environ.get('DISPLAY')
    print(f"DISPLAY environment variable: {display}")
    
    # Check if X11 socket exists
    x11_socket = f"/tmp/.X11-unix/X{display.split(':')[1] if display and ':' in display else '0'}"
    if os.path.exists(x11_socket):
        print(f"✓ X11 socket exists: {x11_socket}")
    else:
        print(f"✗ X11 socket not found: {x11_socket}")
        return False
    
    # Test xrandr
    try:
        result = subprocess.run(['xrandr'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✓ X11 display is working")
            return True
        else:
            print("✗ X11 display not working")
            print(f"Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("✗ X11 display test timed out")
        return False
    except FileNotFoundError:
        print("✗ xrandr command not found - need to install X11 utilities")
        return False

def test_gazebo():
    """Test Gazebo GUI"""
    print("\nTesting Gazebo GUI...")
    try:
        # Test gazebo version
        result = subprocess.run(['gazebo', '--version'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ Gazebo version: {result.stdout.strip()}")
            return True
        else:
            print("✗ Gazebo not working")
            return False
    except Exception as e:
        print(f"✗ Gazebo test failed: {e}")
        return False

def test_rviz():
    """Test RViz2"""
    print("\nTesting RViz2...")
    try:
        result = subprocess.run(['rviz2', '--version'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ RViz2 available")
            return True
        else:
            print("✗ RViz2 not working")
            return False
    except Exception as e:
        print(f"✗ RViz2 test failed: {e}")
        return False

def test_opengl():
    """Test OpenGL"""
    print("\nTesting OpenGL...")
    try:
        result = subprocess.run(['glxinfo'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0 and "OpenGL renderer" in result.stdout:
            print("✓ OpenGL working")
            # Extract renderer info
            for line in result.stdout.split('\n'):
                if 'OpenGL renderer' in line:
                    print(f"  {line.strip()}")
                elif 'OpenGL version' in line:
                    print(f"  {line.strip()}")
                    break
            return True
        else:
            print("✗ OpenGL not working properly")
            return False
    except FileNotFoundError:
        print("⚠ glxinfo not available - installing mesa-utils might help")
        # Try alternative test
        try:
            result = subprocess.run(['glxgears', '-info'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                print("✓ OpenGL basic functionality detected")
                return True
        except:
            pass
        return False
    except Exception as e:
        print(f"✗ OpenGL test failed: {e}")
        return False

def main():
    print("TurtleBot3 GUI Test")
    print("=" * 30)
    
    display_ok = test_display()
    opengl_ok = test_opengl()
    gazebo_ok = test_gazebo()
    rviz_ok = test_rviz()
    
    print("\nSummary:")
    print("=" * 30)
    print(f"Display: {'✓' if display_ok else '✗'}")
    print(f"OpenGL: {'✓' if opengl_ok else '✗'}")
    print(f"Gazebo: {'✓' if gazebo_ok else '✗'}")
    print(f"RViz2: {'✓' if rviz_ok else '✗'}")
    
    if all([display_ok, opengl_ok, gazebo_ok, rviz_ok]):
        print("\n✓ All GUI components working!")
        return 0
    else:
        print("\n✗ Some GUI components need fixing")
        if not display_ok:
            print("  - Run on host: ./host-setup.sh")
            print("  - Then rebuild container")
        if not opengl_ok:
            print("  - Check NVIDIA drivers on host")
            print("  - Ensure nvidia-container-toolkit is installed")
        return 1

if __name__ == "__main__":
    sys.exit(main())