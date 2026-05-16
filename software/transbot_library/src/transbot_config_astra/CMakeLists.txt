cmake_minimum_required(VERSION 3.5)
project(transbot_config_astra)

# ROS 2 빌드 시스템 불러오기
find_package(ament_cmake REQUIRED)

# 설치 규칙 정의
# ROS 2에서는 share/${PROJECT_NAME} 경로에 런치와 설정 파일을 바로 넣습니다.
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ROS 1의 EXCLUDE 패턴이 필요하다면 아래와 같이 작성할 수 있습니다.
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
#   PATTERN "setup_assistant.launch" EXCLUDE
# )

# ament 패키지로 등록 (catkin_package() 대체)
ament_package()
