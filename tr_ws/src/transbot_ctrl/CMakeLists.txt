cmake_minimum_required(VERSION 3.8)
project(transbot_ctrl)

# C++ 표준 설정 (ROS 2는 최소 C++14 또는 C++17을 권장합니다)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 1. 필수 종속성 패키지 찾기 (find_package)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(transbot_msgs REQUIRED)
# rosmsg는 ROS 2에서 사용되지 않으며, 보통 rclcpp 내부 기능으로 대체됩니다.

# 2. C++ 노드 실행파일 생성 (예시: src/transbot_ctrl_node.cpp가 있는 경우)
# 주석을 해제하여 사용하세요.
# add_executable(transbot_ctrl_node src/transbot_ctrl_node.cpp)
# ament_target_dependencies(transbot_ctrl_node rclcpp transbot_msgs)

# 3. Python 스크립트 설치 (예시: scripts 폴더 내 실행 파일이 있는 경우)
install(PROGRAMS
   scripts/transbot_joy.py
   scripts/transbot_keyboard.py
   DESTINATION lib/${PROJECT_NAME}
)

# 4. 헤더 파일 및 기타 리소스 설치 (Launch 파일 등)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_libraries(transbot_ctrl)
ament_package()
