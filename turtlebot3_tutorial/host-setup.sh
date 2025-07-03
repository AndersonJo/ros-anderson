#!/bin/bash
# Host setup script for TurtleBot3 GUI in Docker with NVIDIA GPU
# Run this script on your HOST machine before starting the container

echo "Setting up host for TurtleBot3 GUI with NVIDIA GPU..."

# Check if running on host
if [ -f /.dockerenv ]; then
    echo "ERROR: This script should be run on the HOST machine, not inside the container!"
    exit 1
fi

# Allow X11 forwarding
echo "Allowing X11 forwarding..."
xhost +local:docker
xhost +local:root

# Check DISPLAY variable
echo "Current DISPLAY: $DISPLAY"
if [ -z "$DISPLAY" ]; then
    echo "WARNING: DISPLAY variable is not set. Setting to :0"
    export DISPLAY=:0
fi

# Check X11 socket
echo "Checking X11 socket..."
DISPLAY_NUM="${DISPLAY##*:}"
X11_SOCKET="/tmp/.X11-unix/X${DISPLAY_NUM}"

if [ -e "$X11_SOCKET" ]; then
    echo "✓ X11 socket exists: $(ls -la $X11_SOCKET)"
    sudo chmod 666 "$X11_SOCKET" 2>/dev/null || echo "Could not change socket permissions"
else
    echo "✗ X11 socket not found: $X11_SOCKET"
    echo "Available X11 sockets:"
    ls -la /tmp/.X11-unix/ 2>/dev/null || echo "No X11 sockets found"
fi

# Check NVIDIA GPU
echo "Checking NVIDIA GPU..."
if command -v nvidia-smi &> /dev/null; then
    echo "✓ NVIDIA driver found"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader,nounits
    
    # Check nvidia-docker
    if command -v nvidia-docker &> /dev/null; then
        echo "✓ nvidia-docker found"
    else
        echo "⚠ nvidia-docker not found, but nvidia-container-toolkit might be installed"
    fi
    
    # Check nvidia-container-runtime
    if docker info | grep -q nvidia; then
        echo "✓ NVIDIA container runtime detected"
    else
        echo "⚠ NVIDIA container runtime not detected"
        echo "You may need to install nvidia-container-toolkit"
    fi
else
    echo "✗ NVIDIA driver not found"
    echo "Please install NVIDIA drivers first"
fi

# Test basic X11
echo "Testing basic X11 on host..."
if command -v xrandr &> /dev/null; then
    echo "✓ xrandr works on host"
    xrandr | head -2
else
    echo "✗ xrandr not working on host"
fi

# Create .Xauthority if it doesn't exist
if [ ! -f ~/.Xauthority ]; then
    echo "Creating .Xauthority file..."
    touch ~/.Xauthority 2>/dev/null || echo "Could not create .Xauthority"
    xauth generate $DISPLAY . trusted 2>/dev/null || echo "Could not generate xauth"
else
    echo "✓ .Xauthority file exists"
fi

echo "Host setup complete!"
echo ""
echo "Now you can start the container with:"
echo "  1. Open VS Code in the project directory"
echo "  2. Select 'Reopen in Container'"
echo "  3. Wait for container to build"
echo "  4. Run inside container: ./setup_rl_env.sh"