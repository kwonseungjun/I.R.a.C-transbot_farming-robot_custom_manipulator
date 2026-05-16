cmake_minimum_required(VERSION 3.8)
project(robotis_manipulator)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(robotis_manipulator
  src/robotis_manipulator/robotis_manipulator.cpp
  src/robotis_manipulator/robotis_manipulator_common.cpp
  src/robotis_manipulator/robotis_manipulator_log.cpp
  src/robotis_manipulator/robotis_manipulator_trajectory_generator.cpp
  src/robotis_manipulator/robotis_manipulator_manager.cpp
  src/robotis_manipulator/robotis_manipulator_math.cpp
)

target_include_directories(robotis_manipulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robotis_manipulator
  rclcpp
)

target_link_libraries(robotis_manipulator
  Eigen3::Eigen
)

install(TARGETS robotis_manipulator
  EXPORT export_robotis_manipulator  # 수출할 타겟 그룹 이름 지정
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)
# --- 이 부분을 추가하세요 ---

# 1. 헤더 파일 경로를 외부 패키지에 알림
ament_export_include_directories(include)

# 2. 빌드된 라이브러리를 외부 패키지에 알림
ament_export_libraries(robotis_manipulator)

# -----------------------
ament_export_targets(export_robotis_manipulator HAS_LIBRARY_TARGET)

ament_package()
