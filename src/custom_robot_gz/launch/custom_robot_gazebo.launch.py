#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='empty.sdf')

    # Get package paths
    pkg_custom_robot = FindPackageShare('custom_robot_gz')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            'xacro ',
            PathJoinSubstitution([
                pkg_custom_robot,
                'urdf',
                'custom_mobile_manipulator.urdf.xacro'
            ])
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            pkg_custom_robot,
            'config',
            'custom_robot_controllers.yaml'
        ]
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -v 2 ', world_file]}.items(),
    )

    # Spawn Entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_custom_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'custom_mobile_manipulator',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-allow_renaming', 'true'
        ]
    )

    # Bridge for clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    mobile_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mobile_base_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', robot_controllers
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', robot_controllers
        ],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', robot_controllers
        ],
        output='screen',
    )

    # Delay controller spawning after entity spawn
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_mobile_base_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mobile_base_controller_spawner],
        )
    )

    delay_arm_controller_after_mobile_base = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mobile_base_controller_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.sdf',
            description='World file to load in Gazebo'
        ),

        # Nodes
        gz_sim,
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,

        # Event handlers for delayed controller spawning
        delay_joint_state_broadcaster_after_spawn,
        delay_mobile_base_controller_after_joint_state,
        delay_arm_controller_after_mobile_base,
        delay_gripper_controller_after_arm,
    ])