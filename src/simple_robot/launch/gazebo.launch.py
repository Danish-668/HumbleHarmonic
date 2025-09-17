import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
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

    # Spawn entity using gz service
    spawn_entity = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/simple_world/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'sdf_filename: "{urdf_file}", name: "simple_robot", pose: {{position: {{x: 0, y: 0, z: 0.2}}}}'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])