#include <rclcpp/rclcpp.hpp>
#include "../include/transbot_bringup/base.h"

int main(int argc, char **argv) {
    // 1. ROS 2 시스템 초기화
    rclcpp::init(argc, argv);

    // 2. 노드 생성 (노드 이름: "base_node")
    // 이 노드 객체가 파라미터 읽기, Publisher/Subscriber 생성 등에 사용됩니다.
    auto node = std::make_shared<rclcpp::Node>("base_node");

    // 3. RobotBase 객체 생성 
    // 수정된 RobotBase 생성자에 노드 포인터(SharedPtr)를 전달합니다.
    RobotBase Robot(node);

    // 4. 노드 실행 (이벤트 루프 시작)
    // rclcpp::spin은 노드 내의 콜백 함수(velCallback 등)를 처리합니다.
    rclcpp::spin(node);

    // 5. 종료 처리
    rclcpp::shutdown();
    
    return 0;
}
