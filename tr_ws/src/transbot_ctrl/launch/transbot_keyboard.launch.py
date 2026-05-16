from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # transbot_keyboard 노드 설정
    keyboard_node = Node(
        package='transbot_ctrl',
        executable='transbot_keyboard.py',
        name='transbot_keyboard',
        emulate_tty=True,
        parameters=[
            {
                'linear_speed_limit': 0.45,
                'angular_speed_limit': 2.0
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        keyboard_node
    ])
