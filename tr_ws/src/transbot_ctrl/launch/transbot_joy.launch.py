from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. joy_node 설정
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # 2. transbot_joy 노드 설정 (transbot_ctrl 패키지)
    transbot_joy_node = Node(
        package='transbot_ctrl',
        executable='transbot_joy.py',  # .py 확장자는 보통 setup.py의 entry_points에서 제거됨
        name='transbot_joy',
        parameters=[
            {'use_sim_time': False},
            {'linear_speed_limit': 0.45},
            {'angular_speed_limit': 2.0}
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        transbot_joy_node
    ])
