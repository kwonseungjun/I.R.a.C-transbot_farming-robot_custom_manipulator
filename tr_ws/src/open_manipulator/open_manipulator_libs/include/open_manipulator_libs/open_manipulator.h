#ifndef OPEN_MANIPULATOR_H_
#define OPEN_MANIPULATOR_H_


#include <vector>
#include <map>
#include <string>

#include "dynamixel.h"
#include "custom_trajectory.h"
#include "kinematics.h"




#define CUSTOM_TRAJECTORY_SIZE 4
#define CUSTOM_TRAJECTORY_LINE    "custom_trajectory_line"
#define CUSTOM_TRAJECTORY_CIRCLE  "custom_trajectory_circle"
#define CUSTOM_TRAJECTORY_RHOMBUS "custom_trajectory_rhombus"
#define CUSTOM_TRAJECTORY_HEART   "custom_trajectory_heart"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL  "tool_dxl"

#define X_AXIS robotis_manipulator::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS robotis_manipulator::math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS robotis_manipulator::math::vector3(0.0, 0.0, 1.0)

class OpenManipulator : public robotis_manipulator::RobotisManipulator
{
private:
  robotis_manipulator::Kinematics *kinematics_;
  robotis_manipulator::JointActuator *actuator_;
  robotis_manipulator::ToolActuator *tool_;
  robotis_manipulator::CustomTaskTrajectory *custom_trajectory_[CUSTOM_TRAJECTORY_SIZE];

public:
  OpenManipulator();
  virtual ~OpenManipulator();

  void initOpenManipulator(
      bool using_actual_robot_state,
      std::string usb_port = "/dev/ttyUSB0",
      std::string baud_rate = "1000000",
      float control_loop_time = 0.010);

  void processOpenManipulator(double present_time);
};

#endif
