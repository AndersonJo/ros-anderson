# NVIDIA GPU Setup for TurtleBot3 GUI in Docker

This guide specifically addresses setting up TurtleBot3 with GUI support using your NVIDIA RTX 3090 GPU.

## Prerequisites

Your system needs:
1. NVIDIA RTX 3090 with proper drivers
2. nvidia-container-toolkit
3. X11 forwarding support

## Step 1: Host System Setup

**Run this on your HOST machine (not in container):**

```bash
# Make the host setup script executable and run it
chmod +x host-setup.sh
./host-setup.sh
```

This script will:
- Enable X11 forwarding for Docker
- Check NVIDIA GPU and drivers
- Verify X11 is working
- Set up proper permissions

## Step 2: Install nvidia-container-toolkit (if not already installed)

```bash
# On Ubuntu/Debian host
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$(. /etc/os-release; echo $ID$VERSION_ID) /" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Step 3: Verify Host Setup

```bash
# Check NVIDIA driver
nvidia-smi

# Check Docker can access GPU
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Check X11
xrandr
```

## Step 4: Rebuild Container

1. **Close VS Code**
2. **Delete existing container**: `docker container prune`
3. **Reopen project in VS Code**
4. **Select "Reopen in Container"**

## Step 5: Test Inside Container

```bash
# Run environment setup
./setup_rl_env.sh

# Test GUI components
python3 test_gui.py

# Test X11 directly
/usr/local/bin/setup-x11.sh
```

## Step 6: Launch TurtleBot3 GUI

```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch RViz2
ros2 launch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch.py
```

## Common Issues and Solutions

### Issue 1: "xrandr command not found"
**Solution**: Container needs to be rebuilt with updated packages

### Issue 2: "NVIDIA GPU not detected"
**Solution**: 
- Check `nvidia-smi` works on host
- Verify nvidia-container-toolkit is installed
- Restart Docker daemon

### Issue 3: "X11 display not working"
**Solution**:
- Run `xhost +local:docker` on host
- Check DISPLAY variable: `echo $DISPLAY`
- Verify X11 socket exists: `ls -la /tmp/.X11-unix/`

### Issue 4: "OpenGL not working"
**Solution**:
- Check GPU is visible: `nvidia-smi` in container
- Verify OpenGL: `glxinfo | grep "OpenGL renderer"`
- If fails, try software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`

### Issue 5: "Permission denied" errors
**Solution**:
- Check file permissions: `ls -la /tmp/.X11-unix/`
- Run host setup script again
- Restart container

## Environment Variables

The container should have these set:
```bash
DISPLAY=:0                           # X11 display
NVIDIA_VISIBLE_DEVICES=all          # All GPUs visible
NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
LIBGL_ALWAYS_INDIRECT=0             # Hardware acceleration
TURTLEBOT3_MODEL=burger             # TurtleBot3 model
```

## Performance Tips

1. **Use hardware acceleration**: Ensure `LIBGL_ALWAYS_SOFTWARE=0`
2. **Monitor GPU usage**: `nvidia-smi -l 1`
3. **Check GPU memory**: `nvidia-smi --query-gpu=memory.used,memory.total --format=csv`

## Advanced Debugging

### Check GPU in container:
```bash
nvidia-smi
glxinfo | grep -i nvidia
```

### Check X11 forwarding:
```bash
echo $DISPLAY
xrandr
xclock  # Should show a clock window
```

### Check OpenGL rendering:
```bash
glxgears  # Should show spinning gears
```

## Verification Checklist

- [ ] Host has NVIDIA drivers: `nvidia-smi` works
- [ ] nvidia-container-toolkit installed
- [ ] X11 forwarding enabled: `xhost +local:docker`
- [ ] Container rebuilt with new configuration
- [ ] GUI test passes: `python3 test_gui.py`
- [ ] Gazebo launches with GUI
- [ ] RViz2 launches with GUI
- [ ] GPU acceleration working: `glxinfo | grep NVIDIA`