cmake_minimum_required(VERSION 3.8)
project(open_manipulator_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# 1. 실행 파일 생성 (타겟 이름: open_manipulator_teleop_keyboard)
add_executable(open_manipulator_teleop_keyboard
  src/open_manipulator_teleop_keyboard.cpp
)

# 2. 의존성 연결 (타겟 이름을 위와 동일하게 수정)
ament_target_dependencies(open_manipulator_teleop_keyboard
  rclcpp
  std_msgs
  sensor_msgs
  open_manipulator_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

# 3. 설치 설정
install(TARGETS
  open_manipulator_teleop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 4. 내보내기 설정
ament_export_include_directories(include)
# 라이브러리가 아닌 실행 파일 프로젝트이므로 export_libraries는 생략 가능하거나 
# 필요시 프로젝트 이름을 정확히 기재합니다.
ament_export_libraries(open_manipulator_teleop_keyboard) 

ament_package()
