#ifndef RIKI_BASE_H
#define RIKI_BASE_H

#include <iostream>
#include <string>
#include <memory>

// ROS 2 필수 헤더
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class RobotBase {
public:
    // ROS 2에서는 Node 객체의 포인터를 받아야 하므로 생성자를 수정합니다.
    RobotBase(rclcpp::Node::SharedPtr nh);

    // 콜백 함수의 인자 타입은 SharedPtr 방식을 사용하는 것이 ROS 2 표준입니다.
    void velCallback(const geometry_msgs::msg::Twist::SharedPtr twist);

private:
    // ROS 1의 ros::NodeHandle 대신 rclcpp::Node의 SharedPtr를 멤버로 가집니다.
    rclcpp::Node::SharedPtr nh_;

    // Publisher와 Subscription 타입도 템플릿 기반 SharedPtr로 변경합니다.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;

    // 시간 타입 변경
    rclcpp::Time last_vel_time_;

    // TF2 Broadcaster (기존 tf2_ros 사용 유지)
    tf2_ros::TransformBroadcaster odom_broadcaster_;

    float linear_scale_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
    std::string namespace_;
    bool is_multi_robot_;
};

#endif
