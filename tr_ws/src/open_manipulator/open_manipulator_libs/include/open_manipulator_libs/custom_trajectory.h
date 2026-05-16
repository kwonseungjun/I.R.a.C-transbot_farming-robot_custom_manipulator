#ifndef CUSTOM_TRAJECTORY_H_
#define CUSTOM_TRAJECTORY_H_

#include <eigen3/Eigen/Eigen>
#include <vector>
#include "robotis_manipulator/robotis_manipulator.h"

namespace custom_trajectory
{
class Line : public robotis_manipulator::CustomTaskTrajectory
{
public:
  Line() {}
  virtual ~Line() {}

  // 부모 클래스의 순수 가상 함수를 정확히 오버라이드
  void makeTaskTrajectory(double move_time, robotis_manipulator::TaskWaypoint start, const void *arg) override;
  void setOption(const void *arg) override;
  robotis_manipulator::TaskWaypoint getTaskWaypoint(double tick) override;

private:
  double move_time_;
  double acc_dec_time_;
  std::vector<double> vel_max_;
  robotis_manipulator::TaskWaypoint start_pose_;
  robotis_manipulator::TaskWaypoint goal_pose_;
};

class Circle : public robotis_manipulator::CustomTaskTrajectory
{
public:
  Circle() {}
  virtual ~Circle() {}
  void makeTaskTrajectory(double move_time, robotis_manipulator::TaskWaypoint start, const void *arg) override;
  void setOption(const void *arg) override;
  robotis_manipulator::TaskWaypoint getTaskWaypoint(double tick) override;
};

class Rhombus : public robotis_manipulator::CustomTaskTrajectory
{
public:
  Rhombus() {}
  virtual ~Rhombus() {}
  void makeTaskTrajectory(double move_time, robotis_manipulator::TaskWaypoint start, const void *arg) override;
  void setOption(const void *arg) override;
  robotis_manipulator::TaskWaypoint getTaskWaypoint(double tick) override;
};

class Heart : public robotis_manipulator::CustomTaskTrajectory
{
public:
  Heart() {}
  virtual ~Heart() {}
  void makeTaskTrajectory(double move_time, robotis_manipulator::TaskWaypoint start, const void *arg) override;
  void setOption(const void *arg) override;
  robotis_manipulator::TaskWaypoint getTaskWaypoint(double tick) override;
};
}
#endif
