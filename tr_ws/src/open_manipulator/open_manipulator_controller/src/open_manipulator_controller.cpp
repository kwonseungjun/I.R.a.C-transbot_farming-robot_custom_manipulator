/*******************************************************************************
* Copyright 2026 ROBOTIS CO., LTD. (Updated for ROS 2)
*******************************************************************************/

#include "../include/open_manipulator_controller/open_manipulator_controller.h"

using namespace robotis_manipulator;
using namespace open_manipulator_controller;
using namespace std::placeholders;

OpenManipulatorController::OpenManipulatorController(std::string usb_port, std::string baud_rate)
: Node("open_manipulator_controller"),
  timer_thread_state_(false)
{
  /************************************************************
  ** Initialize ROS 2 parameters
  ************************************************************/
  this->declare_parameter("control_period", 0.010);
  this->declare_parameter("using_platform", false);

  control_period_ = this->get_parameter("control_period").as_double();
  using_platform_ = this->get_parameter("using_platform").as_bool();

  /************************************************************
  ** Initialize variables
  ************************************************************/
  open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);

  if (using_platform_) 
    RCLCPP_INFO(this->get_logger(), "Succeeded to init %s", this->get_namespace());
  else 
    RCLCPP_INFO(this->get_logger(), "Ready to simulate %s on Gazebo", this->get_namespace());

  /************************************************************
  ** Initialize ROS publishers, subscribers and servers
  ************************************************************/
  initPublisher();
  initSubscriber();
  initServer();
}

OpenManipulatorController::~OpenManipulatorController()
{
  timer_thread_state_ = false;
  if (timer_thread_.joinable()) timer_thread_.join(); 
  
  RCLCPP_INFO(this->get_logger(), "Shutdown OpenManipulator Controller");
  open_manipulator_.disableAllActuator();
}

void OpenManipulatorController::startTimerThread()
{
  timer_thread_state_ = true;
  timer_thread_ = std::thread(&OpenManipulatorController::timerThread, this);
}

void OpenManipulatorController::timerThread()
{
  auto next_time = std::chrono::steady_clock::now();
  auto period = std::chrono::duration<double>(control_period_);

  while (rclcpp::ok() && timer_thread_state_)
  {
    next_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    this->process(this->now().seconds());
    std::this_thread::sleep_until(next_time);
  }
}

/********************************************************************************
** Init Functions
********************************************************************************/
void OpenManipulatorController::initPublisher()
{
  auto om_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  for (auto const& name : om_tools_name)
  {
    auto pb = this->create_publisher<open_manipulator_msgs::msg::KinematicsPose>(name + "/kinematics_pose", 10);
    open_manipulator_kinematics_pose_pub_.push_back(pb);
  }

  open_manipulator_states_pub_ = this->create_publisher<open_manipulator_msgs::msg::OpenManipulatorState>("states", 10);

  if (using_platform_)
  {
    open_manipulator_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  }
  else
  {
    auto gazebo_joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
    // Gazebo는 Joint와 Tool 모두 제어해야 하므로 합쳐서 생성
    for (auto const& name : gazebo_joints_name)
    {
      auto pb = this->create_publisher<std_msgs::msg::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
    for (auto const& name : om_tools_name)
    {
      auto pb = this->create_publisher<std_msgs::msg::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
  }
}

void OpenManipulatorController::initSubscriber()
{
  open_manipulator_option_sub_ = this->create_subscription<std_msgs::msg::String>(
    "option", 10, std::bind(&OpenManipulatorController::openManipulatorOptionCallback, this, std::placeholders::_1));
}

void OpenManipulatorController::initServer()
{
  auto handle_joint_path = std::bind(&OpenManipulatorController::goalJointSpacePathCallback, this, std::placeholders::_1, std::placeholders::_2);
  goal_joint_space_path_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path", handle_joint_path);

  goal_joint_space_path_to_kinematics_pose_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_joint_space_path_to_kinematics_pose", std::bind(&OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

  goal_task_space_path_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path", std::bind(&OpenManipulatorController::goalTaskSpacePathCallback, this, std::placeholders::_1, std::placeholders::_2));

  goal_tool_control_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_tool_control", std::bind(&OpenManipulatorController::goalToolControlCallback, this, std::placeholders::_1, std::placeholders::_2));

  set_actuator_state_server_ = this->create_service<open_manipulator_msgs::srv::SetActuatorState>(
    "set_actuator_state", std::bind(&OpenManipulatorController::setActuatorStateCallback, this, std::placeholders::_1, std::placeholders::_2));

  goal_drawing_trajectory_server_ = this->create_service<open_manipulator_msgs::srv::SetDrawingTrajectory>(
    "goal_drawing_trajectory", std::bind(&OpenManipulatorController::goalDrawingTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));
    
   goal_joint_space_path_from_present_server_ = this->create_service<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path_from_present", std::bind(&OpenManipulatorController::goalJointSpacePathFromPresentCallback, this, _1, _2));

  goal_task_space_path_from_present_server_ = this->create_service<open_manipulator_msgs::srv::SetKinematicsPose>(
    "goal_task_space_path_from_present", std::bind(&OpenManipulatorController::goalTaskSpacePathFromPresentCallback, this, _1, _2)); 
}

/*****************************************************************************
** Callback Functions for ROS Servers
*****************************************************************************/
void OpenManipulatorController::goalJointSpacePathCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  std::vector<double> target_angle;
  for (const auto & pos : req->joint_position.position) target_angle.push_back(pos);

  res->is_planned = open_manipulator_.makeJointTrajectory(target_angle, req->path_time);
}

void OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  robotis_manipulator::KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);
  target_pose.orientation = robotis_manipulator::math::convertQuaternionToRotationMatrix(q);

  res->is_planned = open_manipulator_.makeJointTrajectory(req->end_effector_name, target_pose, req->path_time);
}

void OpenManipulatorController::goalTaskSpacePathCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  robotis_manipulator::KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w,
                       req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y,
                       req->kinematics_pose.pose.orientation.z);
  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  res->is_planned = open_manipulator_.makeTaskTrajectory(req->end_effector_name, target_pose, req->path_time);
}

void OpenManipulatorController::goalJointSpacePathFromPresentCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  std::vector<double> target_angle;
  for (auto const& pos : req->joint_position.position) target_angle.push_back(pos);
  res->is_planned = open_manipulator_.makeJointTrajectoryFromPresentPosition(target_angle, req->path_time);
}

void OpenManipulatorController::goalTaskSpacePathFromPresentCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetKinematicsPose::Response> res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req->kinematics_pose.pose.position.x;
  target_pose.position[1] = req->kinematics_pose.pose.position.y;
  target_pose.position[2] = req->kinematics_pose.pose.position.z;
  Eigen::Quaterniond q(req->kinematics_pose.pose.orientation.w, req->kinematics_pose.pose.orientation.x,
                       req->kinematics_pose.pose.orientation.y, req->kinematics_pose.pose.orientation.z);
  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  res->is_planned = open_manipulator_.makeTaskTrajectoryFromPresentPose(req->planning_group, target_pose, req->path_time);
}

void OpenManipulatorController::goalToolControlCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetJointPosition::Response> res)
{
  bool result = true;
  for (size_t i = 0; i < req->joint_position.joint_name.size(); i++)
  {
    if (!open_manipulator_.makeToolTrajectory(req->joint_position.joint_name.at(i), req->joint_position.position.at(i)))
      result = false;
  }
  res->is_planned = result;
}

void OpenManipulatorController::setActuatorStateCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetActuatorState::Response> res)
{
  if (req->set_actuator_state) {
    RCLCPP_INFO(this->get_logger(), "Wait a second for actuator enable");
    timer_thread_state_ = false;
    if (timer_thread_.joinable()) timer_thread_.join();
    open_manipulator_.enableAllActuator();
    startTimerThread();
  } else {
    RCLCPP_INFO(this->get_logger(), "Wait a second for actuator disable");
    timer_thread_state_ = false;
    if (timer_thread_.joinable()) timer_thread_.join();
    open_manipulator_.disableAllActuator();
    startTimerThread();
  }
  res->is_planned = true;
}

void OpenManipulatorController::goalDrawingTrajectoryCallback(
  const std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Request> req,
  std::shared_ptr<open_manipulator_msgs::srv::SetDrawingTrajectory::Response> res)
{
  try {
    if (req->drawing_trajectory_name == "circle") {
      double arg[3] = {req->param[0], req->param[1], req->param[2]};
      res->is_planned = open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req->end_effector_name, arg, req->path_time);
    } else if (req->drawing_trajectory_name == "line") {

    }
    // ... rhombus, heart 등 필요시 추가
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Custom trajectory failed: %s", e.what());
    res->is_planned = false;
  }
}

void OpenManipulatorController::openManipulatorOptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "print_open_manipulator_setting")
    open_manipulator_.printManipulatorSetting();
}

/********************************************************************************
** Process & Publish
********************************************************************************/
void OpenManipulatorController::process(double time)
{
  open_manipulator_.processOpenManipulator(time);
}

void OpenManipulatorController::publishCallback()
{
  if (using_platform_) publishJointStates();
  else publishGazeboCommand();

  publishOpenManipulatorStates();
  publishKinematicsPose();
}

void OpenManipulatorController::publishOpenManipulatorStates()
{
  open_manipulator_msgs::msg::OpenManipulatorState msg;
  msg.open_manipulator_moving_state = open_manipulator_.getMovingState() ? msg.IS_MOVING : msg.STOPPED;
  msg.open_manipulator_actuator_state = open_manipulator_.getActuatorEnabledState(JOINT_DYNAMIXEL) ? msg.ACTUATOR_ENABLED : msg.ACTUATOR_DISABLED;
  open_manipulator_states_pub_->publish(msg);
}

void OpenManipulatorController::publishKinematicsPose()
{
  auto om_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();
  uint8_t index = 0;
  for (auto const& tools : om_tools_name)
  {
    robotis_manipulator::KinematicPose pose = open_manipulator_.getKinematicPose(tools);
    open_manipulator_msgs::msg::KinematicsPose msg;
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond q = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = q.w();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    open_manipulator_kinematics_pose_pub_.at(index)->publish(msg);
    index++;
  }
}

void OpenManipulatorController::publishJointStates()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();

  auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator_.getManipulator()->getAllToolComponentName();
  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto tool_value = open_manipulator_.getAllToolValue();

  for (size_t i = 0; i < joints_name.size(); i++) {
    msg.name.push_back(joints_name.at(i));
    msg.position.push_back(joint_value.at(i).position);
    msg.velocity.push_back(joint_value.at(i).velocity);
    msg.effort.push_back(joint_value.at(i).effort);
  }
  for (size_t i = 0; i < tool_name.size(); i++) {
    msg.name.push_back(tool_name.at(i));
    msg.position.push_back(tool_value.at(i).position);
    msg.velocity.push_back(0.0);
    msg.effort.push_back(0.0);
  }
  open_manipulator_joint_states_pub_->publish(msg);
}

void OpenManipulatorController::publishGazeboCommand()
{
  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto tool_value = open_manipulator_.getAllToolValue();

  for (size_t i = 0; i < joint_value.size(); i++) {
    std_msgs::msg::Float64 msg;
    msg.data = joint_value.at(i).position;
    gazebo_goal_joint_position_pub_.at(i)->publish(msg);
  }
  for (size_t i = 0; i < tool_value.size(); i++) {
    std_msgs::msg::Float64 msg;
    msg.data = tool_value.at(i).position;
    gazebo_goal_joint_position_pub_.at(joint_value.size() + i)->publish(msg);
  }
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc >= 3) {
    usb_port = argv[1];
    baud_rate = argv[2];
  }

  auto om_controller = std::make_shared<OpenManipulatorController>(usb_port, baud_rate);
  om_controller->startTimerThread();

  auto publish_timer = om_controller->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(om_controller->getControlPeriod() * 1000)),
    std::bind(&OpenManipulatorController::publishCallback, om_controller));

  rclcpp::spin(om_controller);
  rclcpp::shutdown();

  return 0;
}
