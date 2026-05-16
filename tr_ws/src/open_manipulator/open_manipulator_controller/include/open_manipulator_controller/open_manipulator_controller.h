/*******************************************************************************
* Copyright 2026 ROBOTIS CO., LTD. (Modified for ROS 2)
*******************************************************************************/

#ifndef OPEN_MANIPULATOR_CONTROLLER_H_
#define OPEN_MANIPULATOR_CONTROLLER_H_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"

// ROS 2 Standard Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

// OpenManipulator Messages & Services (ROS 2 버전 경로)
#include "open_manipulator_msgs/msg/open_manipulator_state.hpp"
#include "open_manipulator_msgs/msg/kinematics_pose.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"
#include "open_manipulator_msgs/srv/set_drawing_trajectory.hpp"
#include "open_manipulator_msgs/srv/set_actuator_state.hpp"

// Library
#include "open_manipulator_libs/open_manipulator.h"

namespace open_manipulator_controller
{
// rclcpp::Node를 상속받는 구조로 변경
class OpenManipulatorController : public rclcpp::Node
{
 public:
  OpenManipulatorController(std::string usb_port, std::string baud_rate);
  virtual ~OpenManipulatorController();

  // update
  void startTimerThread();
  void timerThread();
  void process(double time);
  void publishCallback();
  double getControlPeriod(void) { return control_period_; }

 private:
  /*****************************************************************************
  ** ROS 2 Parameters
  *****************************************************************************/
  bool using_platform_;
  double control_period_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  // POSIX thread 대신 std::thread 사용 권장
  std::thread timer_thread_;
  bool timer_thread_state_;

  // Robotis_manipulator related 
  OpenManipulator open_manipulator_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
  ** ROS Publishers & Timer
  *****************************************************************************/
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Publisher<open_manipulator_msgs::msg::OpenManipulatorState>::SharedPtr open_manipulator_states_pub_;
  std::vector<rclcpp::Publisher<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr> open_manipulator_kinematics_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr open_manipulator_joint_states_pub_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gazebo_goal_joint_position_pub_;

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishJointStates();
  void publishGazeboCommand();

  /*****************************************************************************
  ** ROS Subscribers
  *****************************************************************************/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr open_manipulator_option_sub_;

  void openManipulatorOptionCallback(const std_msgs::msg::String::SharedPtr msg);

  /*****************************************************************************
  ** ROS Servers
  *****************************************************************************/
  // SharedPtr를 사용하는 ROS 2 서비스 서버 정의
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_pose_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_position_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_joint_space_path_to_kinematics_orientation_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_orientation_only_server_;
  
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_position_only_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_from_present_orientation_only_server_;
  
  rclcpp::Service<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetActuatorState>::SharedPtr set_actuator_state_server_;
  rclcpp::Service<open_manipulator_msgs::srv::SetDrawingTrajectory>::SharedPtr goal_drawing_trajectory_server_;

  /*****************************************************************************
  ** Service Callback Functions
  *****************************************************************************/
  void goalJointSpacePathCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);

  void goalJointSpacePathToKinematicsPoseCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalJointSpacePathToKinematicsPositionCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalJointSpacePathToKinematicsOrientationCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalTaskSpacePathCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalTaskSpacePathPositionOnlyCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalTaskSpacePathOrientationOnlyCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalJointSpacePathFromPresentCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);

  void goalTaskSpacePathFromPresentCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalTaskSpacePathFromPresentPositionOnlyCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalTaskSpacePathFromPresentOrientationOnlyCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res);

  void goalToolControlCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res);

  void setActuatorStateCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res);

  void goalDrawingTrajectoryCallback(
    const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Request> req,
    std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Response> res);
};
} // namespace open_manipulator_controller
#endif // OPEN_MANIPULATOR_CONTROLLER_H_
