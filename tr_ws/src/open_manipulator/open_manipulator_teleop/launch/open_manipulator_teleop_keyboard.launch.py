import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 인자(Arguments) 설정
    robot_name = LaunchConfiguration('robot_name', default='open_manipulator')
    end_effector = LaunchConfiguration('end_effector', default='gripper')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='open_manipulator',
            description='Name of the robot'
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='gripper',
            description='Name of the end-effector'
        ),

        # 2. 텔레옵 노드 실행 설정
        Node(
            package='open_manipulator_teleop',
            # 수정: CMakeLists.txt의 add_executable 이름과 반드시 일치해야 함
            executable='open_manipulator_teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            emulate_tty=True,
            # 키보드 입력 인터럽트 방지를 위해 추가 권장
            prefix='xterm -e' if os.environ.get('USE_XTERM') else '', 
            remappings=[
                ('/kinematics_pose', [robot_name, '/', end_effector, '/kinematics_pose']),
            ],
            parameters=[
                {'end_effector_name': end_effector}
            ]
        ),
    ])
