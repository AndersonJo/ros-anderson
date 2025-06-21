#!/bin/bash

echo "🖥️ Setting up X11 forwarding for GUI applications..."

# Set proper X11 environment
export DISPLAY=:1
export QT_X11_NO_MITSHM=1

# OpenGL software rendering for Gazebo
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_GL_VERSION_OVERRIDE=3.3

# Create .Xauthority file if it doesn't exist
if [ ! -f ~/.Xauthority ]; then
    touch ~/.Xauthority
    echo "📄 Created .Xauthority file"
fi

# Test X11 connection
if [ -n "$DISPLAY" ]; then
    echo "📱 DISPLAY is set to: $DISPLAY"
    
    # Test with a simple X11 command
    if command -v xdpyinfo >/dev/null 2>&1; then
        if xdpyinfo >/dev/null 2>&1; then
            echo "✅ X11 connection working!"
        else
            echo "❌ X11 connection failed"
            echo "💡 Make sure you ran on host: xhost +local:docker"
        fi
    else
        echo "⚠️ xdpyinfo not available for testing"
    fi
else
    echo "❌ DISPLAY not set"
fi

echo ""
echo "🚀 X11 Setup Complete!"
echo ""
echo "📋 Test GUI applications:"
echo "  xeyes &                    # Simple test"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "  rviz2                      # RViz2"
echo ""
echo "🔧 Prerequisites on host:"
echo "  xhost +local:docker        # Allow X11 forwarding" 