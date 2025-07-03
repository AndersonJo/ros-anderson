# Simple Guide: Gazebo GUI in Docker Container

## Core Concept

Running GUI applications from a Docker container requires **X11 forwarding** - sending the visual output from container to host display.

## Essential Components

### 1. X11 Forwarding (The Magic)
```bash
# Host allows container to use its display
xhost +local:docker

# Container connects to host's X11 socket
-v /tmp/.X11-unix:/tmp/.X11-unix:rw

# Container uses host's display
-e DISPLAY=$DISPLAY
```

### 2. GPU Acceleration (For Performance)
```bash
# Enable GPU in container
--gpus all

# NVIDIA environment variables
-e NVIDIA_VISIBLE_DEVICES=all
-e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
```

### 3. Minimal Docker Setup

**Dockerfile** (simplified):
```dockerfile
FROM osrf/ros:humble-desktop-full

# Install GUI packages
RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install TurtleBot3
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*
```

**devcontainer.json** (essential parts):
```json
{
  "name": "Simple Gazebo GUI",
  "dockerFile": "Dockerfile",
  "runArgs": [
    "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw",
    "--env=DISPLAY=${env:DISPLAY}",
    "--gpus=all"
  ]
}
```

## How It Works

1. **Host Setup**: `xhost +local:docker` allows container access
2. **X11 Socket**: `/tmp/.X11-unix` is shared between host and container
3. **Display Variable**: `DISPLAY=:1` tells apps where to show windows
4. **GPU Access**: `--gpus all` enables hardware acceleration
5. **Gazebo Launch**: GUI appears on host desktop automatically

## Test Steps

```bash
# 1. Test X11 works
xclock

# 2. Test GPU works
glxinfo | grep NVIDIA

# 3. Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Key Learning Points

- **X11 = Linux GUI system** (like Windows Desktop)
- **Socket = Communication channel** between container and host
- **Display forwarding ≠ Remote desktop** (much faster)
- **GPU passthrough** enables hardware acceleration
- **Container isolation** but shared display

This is the **minimal knowledge** needed to run any GUI application in Docker!