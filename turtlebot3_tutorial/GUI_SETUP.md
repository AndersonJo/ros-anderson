# TurtleBot3 GUI Setup in Docker Container

This guide explains how to set up TurtleBot3 with GUI support in a Docker container for reinforcement learning training.

## Prerequisites

1. Docker with GPU support (if using NVIDIA GPU)
2. X11 forwarding enabled on host system
3. VS Code with Dev Containers extension

## Quick Start

### 1. Host System Setup

Before starting the container, run these commands on your host system:

```bash
# Allow X11 forwarding
xhost +local:docker

# Check your DISPLAY variable
echo $DISPLAY
```

### 2. Build and Start Container

Open this project in VS Code and select "Reopen in Container" when prompted.

### 3. Test GUI Setup

Once inside the container, run:

```bash
# Run GUI test
python3 test_gui.py

# Setup environment
./setup_rl_env.sh
```

### 4. Launch TurtleBot3 Simulation

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch RViz2 (optional)
ros2 launch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch.py
```

## Reinforcement Learning Training

### 1. Start Training

```bash
# Make sure Gazebo is running first
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# In another terminal, start training
python3 src/turtlebot3_rl/scripts/train_rl.py
```

### 2. Monitor Training

```bash
# View TensorBoard logs
tensorboard --logdir=./tensorboard_logs/

# Open browser and go to http://localhost:6006
```

### 3. Evaluate Trained Model

```bash
python3 src/turtlebot3_rl/scripts/evaluate_rl.py
```

## Troubleshooting

### GUI Issues

1. **No display**: 
   - Check DISPLAY variable: `echo $DISPLAY`
   - Run: `xhost +local:docker` on host

2. **Gazebo crashes**:
   - Try software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`
   - Check GPU drivers: `nvidia-smi`

3. **X11 permission denied**:
   - Run setup script: `/usr/local/bin/setup-x11.sh`
   - Fix permissions: `sudo chmod 777 /tmp/.X11-unix`

### Performance Issues

1. **Slow training**:
   - Enable GPU: Check NVIDIA drivers
   - Reduce simulation complexity
   - Adjust training parameters

2. **Memory issues**:
   - Reduce batch size in training script
   - Close unnecessary applications

## Configuration Files

- `.devcontainer/devcontainer.json`: VS Code dev container configuration
- `.devcontainer/Dockerfile`: Docker image definition
- `.devcontainer/setup-x11.sh`: X11 setup script

## Key Features

- **GUI Support**: Full X11 forwarding for Gazebo and RViz2
- **GPU Acceleration**: NVIDIA GPU support for PyTorch
- **Reinforcement Learning**: Stable Baselines3 with PyTorch backend
- **TurtleBot3 Integration**: Complete TurtleBot3 simulation environment
- **Development Tools**: VS Code integration with ROS2 extensions

## Environment Variables

The container sets these important variables:

```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
export DISPLAY=${DISPLAY:-:0}
export QT_X11_NO_MITSHM=1
export ROS_DOMAIN_ID=0
```

## Next Steps

1. Customize the RL environment in `src/turtlebot3_rl/scripts/train_rl.py`
2. Modify reward functions for your specific use case
3. Experiment with different RL algorithms
4. Add more complex navigation scenarios