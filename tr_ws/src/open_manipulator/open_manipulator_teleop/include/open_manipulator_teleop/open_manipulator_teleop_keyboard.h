/*******************************************************************************
* Copyright 2026 (Modified for OpenManipulator-X 5-DOF ROS 2 Humble)
*******************************************************************************/

#ifndef OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
#define OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"

// OpenManipulator Messages & Services
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "open_manipulator_msgs/srv/set_kinematics_pose.hpp"
#include "open_manipulator_msgs/msg/kinematics_pose.hpp"

// TF2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <termios.h>
#include <unistd.h>

// 하드웨어 구성에 맞춘 상수 정의
#define NUM_OF_JOINT 5    // ID 11, 12, 13, 16, 14 총 5개
#define DELTA 0.01        // XYZ 이동 증분 (1cm)
#define JOINT_DELTA 0.05  // 조인트 이동 증분 (rad)
#define PATH_TIME 0.5     // 목표 도달 시간

class OpenManipulatorTeleop : public rclcpp::Node
{
public:
  OpenManipulatorTeleop();
  virtual ~OpenManipulatorTeleop();

  void printText();
  void setGoal(char ch);

private:
  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  std::vector<double> goalPose;
  std::vector<double> goalJoint;
  
  std::vector<std::string> joint_names_;
  std::mutex state_mutex_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers
  *****************************************************************************/
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr kinematics_pose_sub_;

  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg);

  /*****************************************************************************
  ** ROS Clients (실제 존재하는 서비스로 최적화)
  *****************************************************************************/
  // 컨트롤러에 존재하는 기본 서비스 클라이언트
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_joint_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr goal_task_space_path_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr goal_tool_control_client_;

  /*****************************************************************************
  ** Service Call Wrappers (Async)
  *****************************************************************************/
  // 현재 위치를 기반으로 계산하여 절대 좌표 서비스를 호출하는 래퍼 함수들
  bool sendJointSpacePath(const std::vector<double>& joint_delta);
  bool sendTaskSpacePath(const std::vector<double>& task_delta);
  bool sendToolControl(double gripper_pos);

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;
  void restoreTerminalSettings(void);
  
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
};

#endif // OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
