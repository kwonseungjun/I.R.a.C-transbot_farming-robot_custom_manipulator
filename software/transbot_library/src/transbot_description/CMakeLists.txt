cmake_minimum_required(VERSION 3.8)
project(transbot_description)

# C++ 17 표준 사용 (ROS 2 권장)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 필수 패키지 찾기 (ament_cmake 및 ROS 2 패키지)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. 실행 파일 빌드 (C++ 소스 코드)


# 02_attached_object 빌드
add_executable(02_attached_object src/02_attached_object.cpp)
target_include_directories(02_attached_object PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(02_attached_object
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)

# 3. 설치 규칙 (Installation)
# 실행 파일 설치 (lib/<package_name> 경로로 이동)
install(TARGETS
  02_attached_object
  DESTINATION lib/${PROJECT_NAME}
)

# URDF, Meshes, Launch 파일 등 리소스 설치 (share/<package_name> 경로)
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 4. 패키지 내보내기 및 마무리
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)
ament_package()
