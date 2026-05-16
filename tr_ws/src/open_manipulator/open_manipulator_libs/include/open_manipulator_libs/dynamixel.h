#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
  #include <DynamixelWorkbench.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
  #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#endif

// STRING이 정의되지 않았다는 에러를 방지하기 위해 추가
#include <string>
typedef std::string STRING;

namespace dynamixel
{

#define SYNC_WRITE_HANDLER 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132
#define ADDR_VELOCITY_TRAJECTORY_2 136
#define ADDR_POSITION_TRAJECTORY_2 140
#define ADDR_PROFILE_ACCELERATION_2 108
#define ADDR_PROFILE_VELOCITY_2 112
#define ADDR_GOAL_POSITION_2 116

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4
#define LENGTH_VELOCITY_TRAJECTORY_2 4
#define LENGTH_POSITION_TRAJECTORY_2 4
#define LENGTH_PROFILE_ACCELERATION_2 4
#define LENGTH_PROFILE_VELOCITY_2 4
#define LENGTH_GOAL_POSITION_2 4

// Protocol 1.0 (매크로 수정 완료)
#define ADDR_PRESENT_CURRENT_1 40
#define ADDR_PRESENT_VELOCITY_1 38
#define ADDR_PRESENT_POSITION_1 36

#define LENGTH_PRESENT_CURRENT_1 2
#define LENGTH_PRESENT_VELOCITY_1 2
#define LENGTH_PRESENT_POSITION_1 2

typedef struct
{
  std::vector<uint8_t> id;
  uint8_t num;
} Joint;

// --- JointDynamixel 클래스 시작 ---
class JointDynamixel : public robotis_manipulator::JointActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

 public:
  JointDynamixel(){}
  virtual ~JointDynamixel(){}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

  bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  // address 기반 함수 오버로딩
  bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, uint16_t address, uint32_t value);
  bool writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector);
  std::vector<robotis_manipulator::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
};

// --- JointDynamixelProfileControl 클래스 시작 ---
class JointDynamixelProfileControl : public robotis_manipulator::JointActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;
  float control_loop_time_; // unit: ms
  std::map<uint8_t, robotis_manipulator::ActuatorValue> previous_goal_value_;

 public:
  JointDynamixelProfileControl(float control_loop_time = 0.010);
  virtual ~JointDynamixelProfileControl(){}

  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);

  bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
  bool writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  std::vector<robotis_manipulator::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
};

// --- GripperDynamixel 클래스 시작 ---
class GripperDynamixel : public robotis_manipulator::ToolActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;
  bool enabled_state_;

 public:
  GripperDynamixel();
  virtual ~GripperDynamixel();

  virtual void init(uint8_t actuator_id, const void *arg);
  virtual void setMode(const void *arg);
  virtual uint8_t getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendToolActuatorValue(robotis_manipulator::ActuatorValue value);
  virtual robotis_manipulator::ActuatorValue receiveToolActuatorValue();

  // 추상 클래스 에러 방지를 위해 명시
  virtual void setOption(const void *arg);

  bool initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(STRING dynamixel_mode = "position_mode");
  bool writeProfileValue(STRING profile_mode, uint32_t value);
  bool setSDKHandler();
  bool writeGoalPosition(double radian);
  double receiveDynamixelValue();
};

} // namespace dynamixel
#endif // DYNAMIXEL_H_
