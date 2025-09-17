# ROS 2 Humble + Gazebo Harmonic Robot Simulation

Simple robot simulation package for ROS 2 Humble with Gazebo Harmonic.

## Setup

1. Build the workspace:
```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

2. Launch the simulation:
```bash
ros2 launch simple_robot gazebo.launch.py
```

3. Control the robot (in a new terminal):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Robot Description

The simple_robot package contains:
- A 4-wheeled differential drive robot
- URDF model with visual, collision, and inertial properties
- Lidar sensor mounted on top
- Gazebo plugins for differential drive control
- Simple world with obstacles

## Topics

- `/cmd_vel` - Send velocity commands to the robot
- `/odom` - Robot odometry
- `/robot_description` - Robot URDF