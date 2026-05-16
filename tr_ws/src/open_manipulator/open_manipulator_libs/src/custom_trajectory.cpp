#include "open_manipulator_libs/custom_trajectory.h"

using namespace custom_trajectory;
using namespace robotis_manipulator;

/*****************************************************************************
** Line
*****************************************************************************/
void Line::makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg)
{
  move_time_ = move_time;
  acc_dec_time_ = move_time_ * 0.2;
  vel_max_.resize(3);

  start_pose_ = start;

  // arg가 TaskWaypoint 형태의 delta 값이라고 가정
  TaskWaypoint *delta = (TaskWaypoint *)arg;
  goal_pose_.kinematic.orientation = start_pose_.kinematic.orientation;
  goal_pose_.kinematic.position = start.kinematic.position + delta->kinematic.position;

  vel_max_.at(0) = delta->kinematic.position(0) / (move_time_ - acc_dec_time_);
  vel_max_.at(1) = delta->kinematic.position(1) / (move_time_ - acc_dec_time_);
  vel_max_.at(2) = delta->kinematic.position(2) / (move_time_ - acc_dec_time_);
}

void Line::setOption(const void *arg) { (void)arg; }

TaskWaypoint Line::getTaskWaypoint(double tick)
{
  TaskWaypoint pose;
  double time_var = tick;

  if(acc_dec_time_ >= time_var)
  {
    pose.kinematic.position(0) = 0.5 * vel_max_.at(0) * std::pow(time_var, 2) / acc_dec_time_ + start_pose_.kinematic.position(0);
    pose.kinematic.position(1) = 0.5 * vel_max_.at(1) * std::pow(time_var, 2) / acc_dec_time_ + start_pose_.kinematic.position(1);
    pose.kinematic.position(2) = 0.5 * vel_max_.at(2) * std::pow(time_var, 2) / acc_dec_time_ + start_pose_.kinematic.position(2);
  }
  else if(time_var > acc_dec_time_ && (move_time_ - acc_dec_time_) >= time_var)
  {
    pose.kinematic.position(0) = vel_max_.at(0) * (time_var - (acc_dec_time_ * 0.5)) + start_pose_.kinematic.position(0);
    pose.kinematic.position(1) = vel_max_.at(1) * (time_var - (acc_dec_time_ * 0.5)) + start_pose_.kinematic.position(1);
    pose.kinematic.position(2) = vel_max_.at(2) * (time_var - (acc_dec_time_ * 0.5)) + start_pose_.kinematic.position(2);
  }
  else if(time_var > (move_time_ - acc_dec_time_) && (time_var < move_time_))
  {
    pose.kinematic.position(0) = goal_pose_.kinematic.position(0) - vel_max_.at(0) * 0.5 / acc_dec_time_ * (std::pow((move_time_ - time_var), 2));
    pose.kinematic.position(1) = goal_pose_.kinematic.position(1) - vel_max_.at(1) * 0.5 / acc_dec_time_ * (std::pow((move_time_ - time_var), 2));
    pose.kinematic.position(2) = goal_pose_.kinematic.position(2) - vel_max_.at(2) * 0.5 / acc_dec_time_ * (std::pow((move_time_ - time_var), 2));
  }
  else
  {
    pose.kinematic = goal_pose_.kinematic;
  }
  pose.kinematic.orientation = start_pose_.kinematic.orientation;
  return pose;
}

/*****************************************************************************
** Circle, Rhombus, Heart (빈 함수라도 규격에 맞게 구현)
*****************************************************************************/
void Circle::makeTaskTrajectory(double m, TaskWaypoint s, const void *a) { (void)m; (void)s; (void)a; }
void Circle::setOption(const void *a) { (void)a; }
TaskWaypoint Circle::getTaskWaypoint(double t) { (void)t; return TaskWaypoint(); }

void Rhombus::makeTaskTrajectory(double m, TaskWaypoint s, const void *a) { (void)m; (void)s; (void)a; }
void Rhombus::setOption(const void *a) { (void)a; }
TaskWaypoint Rhombus::getTaskWaypoint(double t) { (void)t; return TaskWaypoint(); }

void Heart::makeTaskTrajectory(double m, TaskWaypoint s, const void *a) { (void)m; (void)s; (void)a; }
void Heart::setOption(const void *a) { (void)a; }
TaskWaypoint Heart::getTaskWaypoint(double t) { (void)t; return TaskWaypoint(); }
