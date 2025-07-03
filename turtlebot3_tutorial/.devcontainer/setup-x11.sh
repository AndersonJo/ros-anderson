#!/bin/bash
# X11 setup script for TurtleBot3 GUI in Docker with NVIDIA GPU support

echo "Setting up X11 for TurtleBot3 GUI with NVIDIA GPU..."

# Create runtime directory with proper permissions
mkdir -p /tmp/runtime-user
chmod 755 /tmp/runtime-user

# Fix DISPLAY variable if needed
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
    echo "Set DISPLAY to :0"
fi

# Test X11 connection
echo "Testing X11 connection..."
if command -v xrandr &> /dev/null; then
    if xrandr &> /dev/null 2>&1; then
        echo "✓ X11 display working"
        echo "Available displays:"
        xrandr | grep " connected" | head -3
    else
        echo "✗ X11 display not working"
        echo "DISPLAY variable: $DISPLAY"
        echo "X11 socket exists: $(ls -la /tmp/.X11-unix/ 2>/dev/null || echo 'No')"
    fi
else
    echo "xrandr not available, installing..."
fi

# Test OpenGL and NVIDIA GPU
echo "Testing OpenGL and GPU..."
if command -v glxinfo &> /dev/null; then
    if glxinfo | grep -q "OpenGL renderer"; then
        echo "✓ OpenGL working"
        echo "OpenGL Renderer: $(glxinfo | grep "OpenGL renderer" | cut -d: -f2 | xargs)"
        echo "OpenGL Version: $(glxinfo | grep "OpenGL version" | cut -d: -f2 | xargs)"
        
        # Check for NVIDIA GPU
        if glxinfo | grep -q "NVIDIA"; then
            echo "✓ NVIDIA GPU detected"
            if command -v nvidia-smi &> /dev/null; then
                echo "NVIDIA GPU Info:"
                nvidia-smi -L
            fi
        else
            echo "⚠ NVIDIA GPU not detected in OpenGL"
        fi
    else
        echo "✗ OpenGL not working"
        echo "Falling back to software rendering..."
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
        echo "Set software rendering environment variables"
    fi
else
    echo "glxinfo not available"
fi

# Test simple GUI application
echo "Testing basic GUI..."
if command -v xclock &> /dev/null; then
    echo "xclock available for testing"
elif command -v xeyes &> /dev/null; then
    echo "xeyes available for testing"
else
    echo "No basic GUI test apps available"
fi

echo "X11 setup complete!"
echo "Environment variables:"
echo "  DISPLAY=$DISPLAY"
echo "  LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-0}"
echo "  NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-not set}"