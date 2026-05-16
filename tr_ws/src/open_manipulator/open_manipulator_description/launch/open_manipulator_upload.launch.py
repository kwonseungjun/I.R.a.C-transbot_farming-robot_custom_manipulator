import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 패키지 경로 설정
    pkg_path = get_package_share_directory('open_manipulator_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'open_manipulator_robot.urdf.xacro')

    # 2. xacro 명령 실행 (command 대신 직접 파싱)
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    # 3. 파라미터 설정
    params = {'robot_description': robot_description_config}

    # 4. robot_state_publisher 노드 실행 (일반적으로 같이 사용됨)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])
