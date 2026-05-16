from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 런치 인자(Arguments) 정의
    usb_port = LaunchConfiguration('usb_port')
    baud_rate = LaunchConfiguration('baud_rate')
    control_period = LaunchConfiguration('control_period')
    use_platform = LaunchConfiguration('use_platform')

    declare_usb_port = DeclareLaunchArgument(
        'usb_port',
        default_value='/dev/ttyUSB0',
        description='USB port for DYNAMIXEL'
    )

    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Baud rate for DYNAMIXEL'
    )

    declare_control_period = DeclareLaunchArgument(
        'control_period',
        default_value='0.010',
        description='Control period for the controller'
    )

    declare_use_platform = DeclareLaunchArgument(
        'use_platform',
        default_value='true',
        description='Whether to use the real hardware platform'
    )

    # 2. OpenManipulator 컨트롤러 노드 설정
    # ROS 1의 args="$(arg usb_port) $(arg baud_rate)"는 'arguments' 리스트로 전달합니다.
    controller_node = Node(
        package='open_manipulator_controller',
        executable='open_manipulator_controller', # 보통 실행 파일명이 패키지명과 같습니다.
        name='open_manipulator_controller',
        output='screen',
        arguments=[usb_port, baud_rate],
        parameters=[{
            'control_period': control_period,
            'using_platform': use_platform
        }]
    )

    # 3. LaunchDescription에 인자 및 노드 추가
    return LaunchDescription([
        declare_usb_port,
        declare_baud_rate,
        declare_control_period,
        declare_use_platform,
        controller_node
    ])
