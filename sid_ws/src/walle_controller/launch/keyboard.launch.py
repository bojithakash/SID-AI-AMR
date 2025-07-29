from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        output='screen',
        remappings=[
            ('/cmd_vel', '/simple_velocity_controller/commands')
        ],
        prefix='xterm -e'  # Optional: opens in a new terminal for key input
    )

    return LaunchDescription([
        keyboard_teleop
    ])
