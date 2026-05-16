cmake_minimum_required(VERSION 3.8)
project(transbot_config_camera)

# 1. ament_cmake 패키지 찾기
find_package(ament_cmake REQUIRED)

# 2. 설치 규칙 (Installation)
# ROS 2에서는 CATKIN_PACKAGE_SHARE_DESTINATION 대신 share/${PROJECT_NAME} 경로를 사용합니다.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 3. 패키지 마무리
ament_package()
