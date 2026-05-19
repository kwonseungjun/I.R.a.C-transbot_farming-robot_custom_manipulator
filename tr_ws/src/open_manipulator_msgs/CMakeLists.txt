cmake_minimum_required(VERSION 3.5)
project(open_manipulator_msgs)

################################################################################
# Find packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################################################
# Declare messages and services
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPosition.msg"
  "msg/KinematicsPose.msg"
  "msg/OpenManipulatorState.msg"

  "srv/GetJointPosition.srv"
  "srv/GetKinematicsPose.srv"
  "srv/SetJointPosition.srv"
  "srv/SetKinematicsPose.srv"
  "srv/SetDrawingTrajectory.srv"
  "srv/SetActuatorState.srv"
  "srv/Test.srv"

  DEPENDENCIES std_msgs geometry_msgs
)

################################################################################
# Export dependencies
################################################################################
ament_export_dependencies(rosidl_default_runtime)

################################################################################
# Package
################################################################################
ament_package()
