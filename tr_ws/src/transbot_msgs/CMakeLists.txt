cmake_minimum_required(VERSION 3.8)
project(transbot_msgs)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# 1. 필수 종속성 패키지 찾기
find_package(ament_cmake REQUIRED)


# 메시지/서비스 구성에 필요한 기본 종속성 (std_msgs 등)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED) # ROS 2에서는 보통 action_msgs를 사용하지만 호환성을 위해 유지
find_package(rosidl_default_generators REQUIRED)
# 2. 인터페이스 생성 (메시지, 서비스, 액션 통합)
rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/Adjust.msg"
  "msg/SensorState.msg"
  "msg/Position.msg"
  "msg/General.msg"
  "msg/PWMServo.msg"
  "msg/Joint.msg"
  "msg/Arm.msg"
  "msg/LaserAvoid.msg"
  "msg/ImageMsg.msg"
  "msg/JoyState.msg"
  "msg/PatrolWarning.msg"
  "msg/PointArray.msg"
  "msg/Battery.msg"
  "msg/Edition.msg"
  

  "srv/Buzzer.srv"
  "srv/RGBLight.srv"
  "srv/Headlight.srv"
  "srv/RobotArm.srv"
  "srv/CamDevice.srv"
  "srv/Patrol.srv"
  

  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)
ament_export_dependencies(rosidl_default_runtime)
# 3. 빌드 환경 내보내기
ament_package()
