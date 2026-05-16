cmake_minimum_required(VERSION 3.8)
project(open_manipulator_libs)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/open_manipulator.cpp
  src/custom_trajectory.cpp
  src/dynamixel.cpp
  src/kinematics.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# remove target_link_libraries entirely

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robotis_manipulator
  dynamixel_sdk
  dynamixel_workbench_toolbox
  std_srvs
)

target_link_libraries(open_manipulator_libs
  Eigen3::Eigen
)


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_open_manipulator_libs HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  robotis_manipulator
  dynamixel_sdk
  dynamixel_workbench_toolbox
  std_srvs
)
ament_export_include_directories(include)

ament_export_libraries(open_manipulator_libs)
ament_package()
