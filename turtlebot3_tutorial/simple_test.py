#!/usr/bin/env python3
"""
Simple test to verify Python environment and basic functionality
"""
import sys
import os

def test_python_environment():
    """Test basic Python environment"""
    print("🐍 Python Environment Test")
    print(f"Python version: {sys.version}")
    print(f"Python executable: {sys.executable}")
    
    # Test numpy (if available)
    try:
        import numpy as np
        print(f"✅ NumPy version: {np.__version__}")
    except ImportError:
        print("⚠️ NumPy not available")
    
    # Test basic math operations
    test_array = [1, 2, 3, 4, 5]
    print(f"✅ Basic operations: sum={sum(test_array)}, max={max(test_array)}")
    
    return True

def test_directory_structure():
    """Test current directory structure"""
    print("\n📁 Directory Structure Test")
    print(f"Current directory: {os.getcwd()}")
    
    # Check for src directory
    if os.path.exists('src'):
        print("✅ src directory exists")
        src_contents = os.listdir('src')
        print(f"📦 src contents: {src_contents}")
    else:
        print("⚠️ src directory not found")
        print("Creating src directory...")
        os.makedirs('src', exist_ok=True)
    
    return True

def create_simple_workspace():
    """Create a simple workspace structure"""
    print("\n🏗️ Creating Simple Workspace")
    
    # Create directories
    dirs_to_create = [
        'src',
        'build',
        'install', 
        'log'
    ]
    
    for dir_name in dirs_to_create:
        os.makedirs(dir_name, exist_ok=True)
        print(f"✅ Created: {dir_name}")
    
    # Create a simple test file
    test_file_content = """#!/usr/bin/env python3
# Simple test script
print("Hello from ROS2 workspace!")
"""
    
    with open('src/test_node.py', 'w') as f:
        f.write(test_file_content)
    
    print("✅ Created test node: src/test_node.py")
    
    return True

def main():
    """Main test function"""
    print("🚀 Simple Environment Test")
    print("=" * 50)
    
    try:
        test_python_environment()
        test_directory_structure()
        create_simple_workspace()
        
        print("\n🎉 Basic Test Complete!")
        print("=" * 50)
        print("📋 Next Steps:")
        print("1. Install ROS2 Humble (if not in container)")
        print("2. Setup DevContainer with proper configuration")
        print("3. Test TurtleBot3 packages")
        print("4. Develop RL environment")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 