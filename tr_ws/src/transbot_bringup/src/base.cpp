#include "../include/transbot_bringup/base.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// 전역 변수는 유지하되, 가급적 클래스 멤버를 사용하는 것이 좋습니다.
std::string str1;

RobotBase::RobotBase(rclcpp::Node::SharedPtr nh) : 
    nh_(nh),
    linear_velocity_x_(0.0),
    linear_velocity_y_(0.0),
    angular_velocity_z_(0.0),
    vel_dt_(0.0),
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0),
    odom_broadcaster_(nh) {

    // 1. 초기 시간 설정
    last_vel_time_ = nh_->get_clock()->now();

    // 2. 파라미터 선언 및 가져오기 (ROS 2 필수 단계)
    nh_->declare_parameter<double>("linear_scale", 1.0);
    nh_->declare_parameter<std::string>("is_namespace", "");
    nh_->declare_parameter<bool>("is_multi_robot", false);

    nh_->get_parameter("linear_scale", linear_scale_);
    nh_->get_parameter("is_namespace", namespace_);
    nh_->get_parameter("is_multi_robot", is_multi_robot_);

    str1 = namespace_;

    // 3. Publisher 설정
    odom_publisher_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 50);

    // 4. Subscriber 설정 (람다 또는 std::bind 사용)
    velocity_subscriber_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
        "/transbot/get_vel", 50, 
        std::bind(&RobotBase::velCallback, this, std::placeholders::_1));
}

void RobotBase::velCallback(const geometry_msgs::msg::Twist::SharedPtr twist) {
    // ROS 2 현재 시간 및 시간 차 계산
    rclcpp::Time current_time = nh_->get_clock()->now();
    
    linear_velocity_x_ = twist->linear.x * linear_scale_;
    linear_velocity_y_ = twist->linear.y * linear_scale_;
    angular_velocity_z_ = twist->angular.z;

    // rclcpp::Duration을 초 단위(double)로 변환
    vel_dt_ = (current_time - last_vel_time_).seconds();
    last_vel_time_ = current_time;

    // Odometry 계산 로직
    double delta_heading = angular_velocity_z_ * vel_dt_;
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_;
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_;

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // 5. Yaw를 Quaternion으로 변환 (TF2 활용)
    tf2::Quaternion q;
    q.setRPY(0, 0, heading_);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;

    // 프레임 ID 설정
    if (!is_multi_robot_) {
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
    } else {
        odom.header.frame_id = str1 + "/odom";
        odom.child_frame_id = str1 + "/base_footprint";
    }

    // 위치 데이터 입력
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Pose 공분산 설정
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    // 속도 데이터 입력
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.angular.z = angular_velocity_z_;

    // Twist 공분산 설정
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    // 로그 출력 및 발행
    RCLCPP_INFO(nh_->get_logger(), "ODOM PUBLISH");
    odom_publisher_->publish(odom);
}
