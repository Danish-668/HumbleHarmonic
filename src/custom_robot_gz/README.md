# Custom Mobile Manipulator Robot

A ROS2 Humble robot package featuring a mobile base with differential drive, 3-DOF arm, and gripper, designed for Gazebo Harmonic with gz_ros2_control.

## Quick Commands

### Build Package
```bash
colcon build --packages-select custom_robot_gz
source install/setup.bash
```

### Launch Robot in Gazebo
```bash
export GZ_VERSION=harmonic
source /opt/ros/humble/setup.bash
source /opt/gz_ros2_control_ws/install/setup.bash
source install/setup.bash
ros2 launch custom_robot_gz custom_robot_gazebo.launch.py
```

### Teleop Control (Mobile Base)
```bash
# Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mobile_base_controller/cmd_vel_unstamped

# Manual velocity command
ros2 topic pub /mobile_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}" -r 10
```

### Control 3-DOF Arm
```bash
# Send arm to a specific position (shoulder_pan, shoulder_lift, elbow, wrist)
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint'],
  points: [{
    positions: [0.5, 0.3, -0.5, 0.0],
    velocities: [],
    accelerations: [],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}" --once

# Move arm to home position
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint'],
  points: [{
    positions: [0.0, 0.0, 0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}" --once

# Control gripper (open/close)
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.02, max_effort: 5.0}}"  # Open
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 5.0}}"   # Close

# Monitor joint states
ros2 topic echo /joint_states
```

### Kill Stale Nodes
```bash
# Quick cleanup
ps aux | grep -E "gz|ros2|ruby" | grep -v grep | awk '{print $2}' | xargs -r kill -9

# Restart ROS2 daemon
ros2 daemon stop
ros2 daemon start
```