import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package directory
    pkg_path = os.path.join(get_package_share_directory('simple_robot'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'simple_robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'simple_world.sdf')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch Gazebo Harmonic directly
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v4', world_file],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Spawn entity with a delay
    spawn_entity = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/simple_world/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'sdf_string: "{robot_desc.replace(chr(10), "").replace(chr(34), chr(92) + chr(34))}", name: "simple_robot", pose: {{position: {{x: 0, y: 0, z: 0.2}}}}'
        ],
        output='screen'
    )

    # Bridge for cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen'
    )

    # Bridge for odom
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    # Clock bridge
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # Delayed spawn
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        bridge_cmd_vel,
        bridge_odom,
        bridge_clock,
        delayed_spawn
    ])