#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <robotis_manipulator/robotis_manipulator.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>


namespace kinematics
{

/*****************************************************************************
** Chain Rule + Jacobian Solver
*****************************************************************************/
class SolverUsingCRAndJacobian : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name component_name);

  bool inverseSolverUsingJacobian(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);

public:
  SolverUsingCRAndJacobian() {}
  virtual ~SolverUsingCRAndJacobian() {}

  virtual void setOption(const void *arg);
  virtual Eigen::MatrixXd jacobian(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name tool_name);

  virtual void solveForwardKinematics(robotis_manipulator::Manipulator *manipulator);

  virtual bool solveInverseKinematics(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);
};

/*****************************************************************************
** SR Jacobian Solver
*****************************************************************************/
class SolverUsingCRAndSRJacobian : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name component_name);

  bool inverseSolverUsingSRJacobian(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);

public:
  SolverUsingCRAndSRJacobian() {}
  virtual ~SolverUsingCRAndSRJacobian() {}

  virtual void setOption(const void *arg);

  virtual Eigen::MatrixXd jacobian(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name tool_name);

  virtual void solveForwardKinematics(robotis_manipulator::Manipulator *manipulator);

  virtual bool solveInverseKinematics(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);
};

/*****************************************************************************
** Position-only SR Jacobian
*****************************************************************************/
class SolverUsingCRAndSRPositionOnlyJacobian : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name component_name);

  bool inverseSolverUsingPositionOnlySRJacobian(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);

public:
  SolverUsingCRAndSRPositionOnlyJacobian() {}
  virtual ~SolverUsingCRAndSRPositionOnlyJacobian() {}

  virtual void setOption(const void *arg);

  virtual Eigen::MatrixXd jacobian(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name tool_name);

  virtual void solveForwardKinematics(robotis_manipulator::Manipulator *manipulator);

  virtual bool solveInverseKinematics(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);
};

/*****************************************************************************
** OpenManipulator Chain Solver
*****************************************************************************/
class SolverCustomizedforOMChain : public robotis_manipulator::Kinematics
{
private:
  void forwardSolverUsingChainRule(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name component_name);

  bool chainCustomInverseKinematics(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);

public:
  SolverCustomizedforOMChain() {}
  virtual ~SolverCustomizedforOMChain() {}

  virtual void setOption(const void *arg);

  virtual Eigen::MatrixXd jacobian(robotis_manipulator::Manipulator *manipulator,
                                    robotis_manipulator::Name tool_name);

  virtual void solveForwardKinematics(robotis_manipulator::Manipulator *manipulator);

  virtual bool solveInverseKinematics(
      robotis_manipulator::Manipulator *manipulator,
      robotis_manipulator::Name tool_name,
      robotis_manipulator::Pose target_pose,
      std::vector<robotis_manipulator::JointValue>* goal_joint_value);
};

} // namespace kinematics

#endif
