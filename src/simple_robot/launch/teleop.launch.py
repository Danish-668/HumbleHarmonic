from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Teleop twist keyboard node
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',  # Opens in a new terminal
        output='screen',
        remappings=[
            ('cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        teleop
    ])