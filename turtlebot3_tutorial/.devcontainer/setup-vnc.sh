#!/bin/bash

echo "🖥️ Setting up VNC server for GUI applications..."

# Create VNC directory
mkdir -p ~/.vnc

# Set VNC password (you can change this)
echo "vnc123" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Create VNC startup script
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
xrdb $HOME/.Xresources
xsetroot -solid grey
export XKL_XMODMAP_DISABLE=1
/etc/X11/Xsession
fluxbox &
EOF

chmod +x ~/.vnc/xstartup

# Create fluxbox config directory
mkdir -p ~/.fluxbox

# Create simple fluxbox menu
cat > ~/.fluxbox/menu << 'EOF'
[begin] (Fluxbox)
[exec] (Terminal) {xterm -fa 'Monospace' -fs 12}
[exec] (Gazebo) {gazebo}
[exec] (RViz2) {rviz2}
[separator]
[restart] (Restart)
[exit] (Exit)
[end]
EOF

# Set environment for VNC
export DISPLAY=:1

echo "✅ VNC setup complete!"
echo ""
echo "🚀 To start VNC server:"
echo "  start_vnc"
echo ""
echo "🌐 To access GUI:"
echo "  Open browser: http://localhost:6080/vnc.html"
echo "  VNC Password: vnc123"
echo ""
echo "🛑 To stop VNC server:"
echo "  stop_vnc" 