import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Get the package directory
    pkg_path = os.path.join(get_package_share_directory('simple_robot'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'simple_robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'simple_world.sdf')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch Gazebo Harmonic with the world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': f'-r -v4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
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

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_robot',
            '-string', robot_desc,
            '-x', '0',
            '-y', '0',
            '-z', '0.2',
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # Bridge for various topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Delayed spawn to ensure Gazebo is ready
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        bridge,
        delayed_spawn
    ])