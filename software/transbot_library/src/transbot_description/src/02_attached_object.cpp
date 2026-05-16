#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <memory>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv) {
    // 1. ROS 2 초기화
    rclcpp::init(argc, argv);
    
    // 2. 노드 옵션 설정 (MoveGroupInterface를 위해 필요)
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("attached_object_cpp", node_options);

    // 3. Executor 설정 (별도 스레드에서 노드 실행)
    // MoveGroup은 조인트 상태 업데이트 등을 위해 백그라운드 스핀이 필수입니다.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 4. MoveGroupInterface 초기화 ("arm" 그룹)
    moveit::planning_interface::MoveGroupInterface transbot(node, "arm");
    
    // 5. 계획 파라미터 설정
    transbot.allowReplanning(true);
    transbot.setPlanningTime(5.0);
    transbot.setNumPlanningAttempts(10);
    transbot.setGoalPositionTolerance(0.01);
    transbot.setGoalOrientationTolerance(0.01);
    transbot.setMaxVelocityScalingFactor(1.0);
    transbot.setMaxAccelerationScalingFactor(1.0);

    // 6. 초기 위치 이동 (pose1)
    transbot.setNamedTarget("pose1");
    transbot.move();

    // 7. Planning Scene Interface 및 장애물 설정
    moveit::planning_interface::PlanningSceneInterface scene;
    string planning_frame = transbot.getPlanningFrame();

    // 장애물(Box) 생성
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = planning_frame;
    obj.id = "obj";

    // 첫 번째 박스 (primitive)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.1, 0.01};

    // 두 번째 박스 (floor)
    shape_msgs::msg::SolidPrimitive floor;
    floor.type = floor.BOX;
    floor.dimensions = {1.0, 0.5, 0.01};

    obj.primitives.push_back(primitive);
    obj.primitives.push_back(floor);

    // 장애물 포즈 설정
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.36;
    pose.position.y = 0.0;
    pose.position.z = 0.125;

    tf2::Quaternion q;
    q.setRPY(0, 0, 90.0 * M_PI / 180.0);
    pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Pose pose1; // 기본값 (0,0,0)

    obj.primitive_poses.push_back(pose);
    obj.primitive_poses.push_back(pose1);
    obj.operation = obj.ADD;

    // 장애물 색상 설정
    moveit_msgs::msg::ObjectColor color;
    color.id = "obj";
    color.color.r = 0.0;
    color.color.g = 1.0;
    color.color.b = 0.0;
    color.color.a = 0.5;

    // 8. 씬에 장애물 적용
    scene.applyCollisionObject(obj, color.color);

    // 9. 루프 동작 시작
    rclcpp::Rate loop_rate(2);
    while (rclcpp::ok()) {
        // 특정 조인트 값으로 이동 예시
        auto target_joints1 = transbot.getCurrentJointValues();
        if (target_joints1.size() >= 2) {
            target_joints1[0] = -1.35;
            target_joints1[1] = 1.35;
            transbot.setJointValueTarget(target_joints1);
            transbot.move();
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        auto target_joints2 = transbot.getCurrentJointValues();
        if (target_joints2.size() >= 2) {
            target_joints2[0] = -3.0;
            target_joints2[1] = 3.0;
            transbot.setJointValueTarget(target_joints2);
            transbot.move();
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // 랜덤 타겟 이동
        transbot.setRandomTarget();
        transbot.move();
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // 종료 처리
    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
