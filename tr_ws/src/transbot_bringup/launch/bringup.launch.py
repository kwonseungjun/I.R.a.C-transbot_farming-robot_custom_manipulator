import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 설정
    bringup_dir = get_package_share_directory('transbot_bringup')
    ctrl_dir = get_package_share_directory('transbot_ctrl')
    description_dir = get_package_share_directory('transbot_description')

    # 인자 설정 (Arguments)
    use_gui = LaunchConfiguration('use_gui')
    robot_model = LaunchConfiguration('robot_model')

    declare_use_gui = DeclareLaunchArgument('use_gui', default_value='false')
    declare_robot_model = DeclareLaunchArgument('robot_model', default_value='astra')

    # 1. DeviceSrv (Python 노드)
    device_srv_node = Node(
        package='transbot_bringup',
        executable='DeviceSrv.py', # 패키지 내 스크립트 실행 권한 확인 필요
        name='DeviceSrv',
        output='screen'
    )

    # 2. Transbot Driver (Python 노드)
    transbot_driver_node = Node(
        package='transbot_bringup',
        executable='transbot_driver.py',
        name='transbot_node',
        parameters=[
            {'imu': '/transbot/imu'},
            {'vel': '/transbot/get_vel'}
        ],
        output='screen'
    )


    # 5. IMU Filter (Madgwick)
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        parameters=[{
            'fixed_frame': 'base_link',
            'use_mag': False,
            'publish_tf': False,
            'world_frame': 'enu',
            'orientation_stddev': 0.05,
            'angular_scale': 1.08
        }],
        output='screen'
    )

    # 6. Static TF Publisher (base_link -> imu_link)
    # ROS 2에서는 args 순서가 다름: x y z yaw pitch roll
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=['0.0', '0', '0.02', '0', '0', '0', 'base_link', 'imu_link']
    )

    # 7. Base Node (C++로 리팩토링한 오도메트리 노드)
    base_node = Node(
        package='transbot_bringup',
        executable='base_node',
        name='base_node',
        parameters=[
            {'linear_scale': 1.2},
            {'is_multi_robot': False}
        ]
    )

    # 8. EKF Localization
    ekf_config = os.path.join(bringup_dir, 'param', 'ekf', 'robot_localization.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node', # ROS 2에서는 ekf_node로 이름 변경됨
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')]
    )

    # 9. 조이스틱 제어 (다른 Launch 포함)
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_dir, 'launch', 'transbot_joy.launch.py')
        )
    )

    # LaunchDescription 구성
    return LaunchDescription([
        declare_use_gui,
        declare_robot_model,
        device_srv_node,
        transbot_driver_node,

        imu_filter_node,
        static_tf_node,
        base_node,
        ekf_node,
        joy_launch

    ])
