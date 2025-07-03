#!/bin/bash
# Install missing packages that failed during Docker build

echo "Installing missing packages..."

# Try to install glxinfo and other OpenGL utilities
if command -v apt-get &> /dev/null; then
    echo "Installing OpenGL utilities..."
    apt-get update 2>/dev/null || echo "Could not update package list"
    
    # Try different package names for glxinfo
    if ! command -v glxinfo &> /dev/null; then
        echo "Trying to install glxinfo..."
        apt-get install -y mesa-utils 2>/dev/null || echo "mesa-utils already installed or not available"
        apt-get install -y mesa-utils-extra 2>/dev/null || echo "mesa-utils-extra not available"
        apt-get install -y glx-utils 2>/dev/null || echo "glx-utils not available"
    fi
    
    # Try to install xclock
    if ! command -v xclock &> /dev/null; then
        echo "Trying to install xclock..."
        apt-get install -y x11-apps 2>/dev/null || echo "x11-apps already installed"
    fi
    
    # Try to install glxgears
    if ! command -v glxgears &> /dev/null; then
        echo "Trying to install glxgears..."
        apt-get install -y mesa-utils 2>/dev/null || echo "mesa-utils already installed"
    fi
    
    echo "Package installation complete"
else
    echo "apt-get not available"
fi

# Test what we have
echo ""
echo "Available commands:"
echo "  glxinfo: $(command -v glxinfo &> /dev/null && echo 'Available' || echo 'Not available')"
echo "  xclock: $(command -v xclock &> /dev/null && echo 'Available' || echo 'Not available')"
echo "  glxgears: $(command -v glxgears &> /dev/null && echo 'Available' || echo 'Not available')"
echo "  xrandr: $(command -v xrandr &> /dev/null && echo 'Available' || echo 'Not available')"