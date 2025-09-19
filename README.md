# ROS2 Humble + Gazebo Harmonic DevContainer

A portable, optimized DevContainer for ROS2 Humble with Gazebo Harmonic and gz_ros2_control integration.

##  Features

- **ROS2 Humble** (LTS) - Long Term Support release
- **Gazebo Harmonic** (v8.9.0) - Latest Gazebo simulator
- **gz_ros2_control** - Built from source for perfect compatibility
- **Custom Mobile Manipulator** - Example robot with diff drive, 3-DOF arm, and gripper
- **Optimized Size** - Minimal dependencies with optional features

##  System Requirements

### Minimum Requirements
- **Docker** 20.10 or newer
- **VSCode** with Dev Containers extension
- **OS**: Linux (Ubuntu 20.04/22.04 recommended), Windows with WSL2, or macOS
- **RAM**: 8GB minimum (16GB recommended)
- **Storage**: 10GB free space

### Display Requirements
- X11 server for GUI applications (Gazebo visualization)
- Linux: Works natively
- Windows: Install VcXsrv or WSL2 with WSLg
- macOS: Install XQuartz

##  Configuration Options

Edit `.devcontainer/devcontainer.json` build args to customize:

```json
"args": {
    "INSTALL_CLAUDE_CODE": "false",    // AI coding assistant
    "INSTALL_DEV_TOOLS": "true",       // vim, nano editors
    "INSTALL_EXTRA_GUI_TOOLS": "false" // x11-apps, xterm
}
```

##  Build Instructions

### Option 1: VSCode Dev Containers (Recommended)
1. Clone repository
2. Open in VSCode
3. Press `Ctrl+Shift+P` → "Dev Containers: Reopen in Container"
4. Wait for build (first time ~10-15 minutes)

### Option 2: Docker Build
```bash
# Build with default options
docker build -t ros2-gazebo-harmonic .devcontainer/

# Build with Claude Code
docker build --build-arg INSTALL_CLAUDE_CODE=true -t ros2-gazebo-harmonic .devcontainer/

# Run container
docker run -it --rm \
    --network=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/workspace/humble_harmonic \
    ros2-gazebo-harmonic
```

##  Quick Start

### 1. Build Custom Robot Package
```bash
cd /workspace/humble_harmonic
colcon build --packages-select custom_robot_gz
source install/setup.bash
```

### 2. Launch Robot in Gazebo
```bash
export GZ_VERSION=harmonic
ros2 launch custom_robot_gz custom_robot_gazebo.launch.py
```

### 3. Control the Robot
```bash
# Mobile base
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mobile_base_controller/cmd_vel_unstamped

# Arm control - See src/custom_robot_gz/README.md for details
```

## 🔧 Troubleshooting

### Display Issues (Linux)
```bash
xhost +local:docker
export DISPLAY=:0
```

### Display Issues (Windows WSL2)
1. Install VcXsrv or use WSLg
2. Set in WSL2: `export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0`

### Display Issues (macOS)
1. Install XQuartz
2. Enable "Allow connections from network clients" in XQuartz settings
3. Run: `xhost +localhost`

### GPU Support
- Comment out `"--gpus", "all"` in devcontainer.json if no NVIDIA GPU
- For AMD/Intel GPUs, the software renderer will be used automatically

### Clean Stale Processes
```bash
ps aux | grep -E "gz|ros2|ruby" | grep -v grep | awk '{print $2}' | xargs -r kill -9
ros2 daemon stop && ros2 daemon start
```

##  Package Contents

```
/workspace/humble_harmonic/
├── .devcontainer/          # DevContainer configuration
│   ├── Dockerfile         # Optimized container definition
│   └── devcontainer.json  # VSCode configuration
├── src/
│   └── custom_robot_gz/   # Example robot package
│       ├── urdf/         # Robot description
│       ├── launch/       # Launch files
│       ├── config/       # Controller configs
│       └── README.md     # Robot control instructions
└── README.md             # This file
```

##  Portability Notes

This DevContainer is designed to work across different systems:

- **Linux**: Native support, best performance
- **Windows**: Use WSL2 for best compatibility
- **macOS**: Works with XQuartz, may have slower GUI performance
- **Cloud/CI**: Can run headless without display for testing

##  Performance Optimization

- Base image uses `--no-install-recommends` to reduce size
- Build artifacts cleaned after gz_ros2_control compilation
- Optional features controlled via build args
- Shared memory optimized with `--ipc=host`