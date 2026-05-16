cmake_minimum_required(VERSION 3.8)
project(open_manipulator_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(open_manipulator_libs REQUIRED)

# --- 추가된 핵심 의존성 패키지들 ---
find_package(robotis_manipulator REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
# -------------------------------

find_package(std_srvs REQUIRED)        
find_package(Eigen3 REQUIRED)           

add_executable(open_manipulator_controller
  src/open_manipulator_controller.cpp
)

target_include_directories(open_manipulator_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(open_manipulator_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  open_manipulator_msgs
  open_manipulator_libs
  robotis_manipulator           # 직접 추가
  dynamixel_workbench_toolbox    # 직접 추가
  std_srvs
)

# target_link_libraries를 명확히 정의합니다.
target_link_libraries(open_manipulator_controller
  Eigen3::Eigen
)

install(TARGETS open_manipulator_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)

ament_export_libraries(open_manipulator_controller)
ament_package()
