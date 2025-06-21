#!/bin/bash

echo "🖥️ Testing GUI applications in devcontainer..."

# Set environment variables
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_INDIRECT=1

echo "📱 Using DISPLAY: $DISPLAY"

# Test X11 connection
echo ""
echo "🔍 Testing X11 connection..."
if command -v xdpyinfo >/dev/null 2>&1; then
    if xdpyinfo >/dev/null 2>&1; then
        echo "✅ X11 connection working!"
    else
        echo "❌ X11 connection failed"
        echo "💡 Make sure 'xhost +local:docker' was run on host"
        exit 1
    fi
else
    echo "⚠️ xdpyinfo not available, trying direct test..."
fi

# Test simple X11 app
echo ""
echo "👁️ Testing simple X11 app (xeyes)..."
xeyes &
XEYES_PID=$!
sleep 2
if kill -0 $XEYES_PID 2>/dev/null; then
    echo "✅ xeyes is running successfully!"
    kill $XEYES_PID
else
    echo "❌ xeyes failed to start"
    exit 1
fi

echo ""
echo "🤖 Ready to test Gazebo GUI!"
echo ""
echo "🚀 Run these commands:"
echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "  rviz2"
echo ""
echo "✅ GUI setup complete!" 