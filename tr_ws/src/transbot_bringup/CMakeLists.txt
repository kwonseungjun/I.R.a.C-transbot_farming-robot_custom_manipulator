cmake_minimum_required(VERSION 3.8)
project(transbot_bringup)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 1. 필수 종속성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(transbot_msgs REQUIRED)

# ROS 2에서는 dynamic_reconfigure가 rclcpp 파라미터 시스템에 통합되어 별도 패키지가 필요 없습니다.

# 2. 헤더 파일 경로 설정
include_directories(
  include
  ${transbot_msgs_INCLUDE_DIRS}
)

# 3. C++ 실행 파일 생성 (base_node)
add_executable(base_node 
  src/base_node.cpp 
  src/base.cpp
)

# 종속성 연결 (Target Dependencies)
ament_target_dependencies(base_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  transbot_msgs
)
install(PROGRAMS
  scripts/calibrate_angular.py
  scripts/calibrate_linear.py
  scripts/DeviceSrv.py
  scripts/transbot_driver.py
  scripts/transbot_patrol.py
  scripts/transform_utils.py
  scripts/transbot_driver.py  # 이번에 에러 난 파일 추가
  DESTINATION lib/${PROJECT_NAME}
)
# 4. 설치 설정 (Install)
install(TARGETS
  base_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 및 설정 파일 설치
install(DIRECTORY launch cfg param
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)

ament_export_libraries(transbot_bringup)
ament_package()
