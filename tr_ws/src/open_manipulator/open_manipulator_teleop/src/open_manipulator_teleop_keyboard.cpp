#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <open_manipulator_msgs/srv/set_kinematics_pose.hpp>
#include <open_manipulator_msgs/msg/kinematics_pose.hpp>

#include <open_manipulator_msgs/srv/test.hpp>

#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <mutex>
#include <memory>

#define NUM_OF_JOINT 6 // joint1, 2, 3, 3_5, 4

using namespace std::placeholders;

class OpenManipulatorTeleop : public rclcpp::Node
{
public:
  OpenManipulatorTeleop() : Node("open_manipulator_teleop_keyboard")
  {
    // 파라미터 설정
    this->declare_parameter("delta", 0.01);
    this->declare_parameter("joint_delta", 0.1);
    this->declare_parameter("path_time", 1.0);

    delta_ = this->get_parameter("delta").as_double();
    joint_delta_ = this->get_parameter("joint_delta").as_double();
    path_time_ = this->get_parameter("path_time").as_double();

    // 조인트 이름 (사용자 설정 반영)
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    present_joint_angle_.assign(NUM_OF_JOINT, 0.0);
    present_pose_.assign(6, 0.0);

    // Subscribers
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&OpenManipulatorTeleop::jointCb, this, _1));

    pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
      "/gripper/kinematics_pose", 10, std::bind(&OpenManipulatorTeleop::poseCb, this, _1));

    


    // Clients (상대 좌표 제어용 서비스)
    joint_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path_from_present");
    joint_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_joint_space_path");
    task_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path_from_present");
    task_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("goal_task_space_path");

    tool_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");
    
    
    test_client_ = this->create_client<open_manipulator_msgs::srv::Test>("send_num");
    
    

    RCLCPP_INFO(this->get_logger(), "OpenManipulator ROS2 Teleop Started (5-DOF)");
  }

  void setGoal(char c)
  {
    std::vector<double> goalPose(3, 0.0);
    std::vector<double> goalJoint(NUM_OF_JOINT, 0.0);

    // 372줄 방식: goalPose.at(0) = DELTA_;
    if (c == 'w' || c == 'W') { goalPose.at(0) = delta_;  sendTask(goalPose); }
    else if (c == 's' || c == 'S') { goalPose.at(0) = -delta_; sendTask(goalPose); }
    else if (c == 'a' || c == 'A') { goalPose.at(1) = delta_;  sendTask(goalPose); }
    else if (c == 'd' || c == 'D') { goalPose.at(1) = -delta_; sendTask(goalPose); }
    else if (c == 'z' || c == 'Z') { goalPose.at(2) = delta_;  sendTask(goalPose); }
    else if (c == 'x' || c == 'X') { goalPose.at(2) = -delta_; sendTask(goalPose); }

    // Joint 제어
    else if (c == '6') { goalJoint.at(0) = joint_delta_;  sendJoint(goalJoint); }
    else if (c == 'y') { goalJoint.at(0) = -joint_delta_; sendJoint(goalJoint); }
    else if (c == '7') { goalJoint.at(1) = joint_delta_;  sendJoint(goalJoint); }
    else if (c == 'u') { goalJoint.at(1) = -joint_delta_; sendJoint(goalJoint); }
    else if (c == '8') { goalJoint.at(2) = joint_delta_;  sendJoint(goalJoint); }
    else if (c == 'i') { goalJoint.at(2) = -joint_delta_; sendJoint(goalJoint); }
    else if (c == '9') { goalJoint.at(3) = joint_delta_;  sendJoint(goalJoint); }
    else if (c == 'o') { goalJoint.at(3) = -joint_delta_; sendJoint(goalJoint); }
    else if (c == '0') { goalJoint.at(4) = joint_delta_;  sendJoint(goalJoint); }
    else if (c == 'p') { goalJoint.at(4) = -joint_delta_; sendJoint(goalJoint); }
    else if (c == '-') { goalJoint.at(5) = joint_delta_;  sendJoint(goalJoint); }
    else if (c == '[') { goalJoint.at(5) = -joint_delta_; sendJoint(goalJoint); }
    
    
    else if (c == '1') {
    auto req = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    req->joint_position.joint_name = joint_names_;
    req->joint_position.position = goalJoint; // 컨트롤러가 현재값에 더함
    req->path_time = path_time_;
    joint_client_->async_send_request(req);
    }
    
    else if (c=='n') {
    double num1, num2, num3, num4;
    
    printf("input num1 : \n");
    if (scanf("%lf", &num1) != 1) return;
    
    printf("input num2 : \n");
    if (scanf("%lf", &num2) != 1) return;
    
    printf("input num3 : \n");
    if (scanf("%lf", &num3) != 1) return;
    
    printf("input num4 : \n");
    if (scanf("%lf", &num4) != 1) return;   
     
    auto req = std::make_shared<open_manipulator_msgs::srv::Test::Request>();
    
    req->number1 = num1;
    req->number2 = num2;
    req->number3 = num3;
    req->number4 = num4;
    
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF);

    RCLCPP_INFO(this->get_logger(), "Sent absolute Pose Task with Orientation.");
    
    
    }

    else if (c == 'm') {
    double input_x, input_y, input_z;
    double roll, pitch, yaw;

    // 1. 위치(XYZ) 입력 받기 (미터 단위)
    printf("\n[Task Space Control with Orientation]\n");
    printf("input X (meter): \n");
    if (scanf("%lf", &input_x) != 1) return;

    printf("input Y (meter): \n");
    if (scanf("%lf", &input_y) != 1) return;

    printf("input Z (meter): \n");
    if (scanf("%lf", &input_z) != 1) return;

    // 2. 방향(RPY) 입력 받기 (도 단위 -> 라디안 변환 예정)
    printf("input Roll (degree): \n");
    if (scanf("%lf", &roll) != 1) return;

    printf("input Pitch (degree): \n");
    if (scanf("%lf", &pitch) != 1) return;

    printf("input Yaw (degree): \n");
    if (scanf("%lf", &yaw) != 1) return;

    // 3. 오일러 각도(Degree)를 라디안(Radian)으로 변환
    double r_rad = roll * M_PI / 180.0;
    double p_rad = pitch * M_PI / 180.0;
    double y_rad = yaw * M_PI / 180.0;

    // 4. 오일러 각도 -> 쿼터니언(Quaternion) 변환 공식 적용
    double cy = cos(y_rad * 0.5);
    double sy = sin(y_rad * 0.5);
    double cp = cos(p_rad * 0.5);
    double sp = sin(p_rad * 0.5);
    double cr = cos(r_rad * 0.5);
    double sr = sin(r_rad * 0.5);

    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
    double qw = cr * cp * cy + sr * sp * sy;

    // 5. 서비스 요청 메시지 작성 및 전송
    auto req = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();

    req->planning_group = "gripper"; 
    req->end_effector_name = "gripper"; 
    
    // 위치 대입
    req->kinematics_pose.pose.position.x = input_x;
    req->kinematics_pose.pose.position.y = input_y;
    req->kinematics_pose.pose.position.z = input_z;
    
    // 계산된 쿼터니언 방향 대입
    req->kinematics_pose.pose.orientation.x = qx;
    req->kinematics_pose.pose.orientation.y = qy;
    req->kinematics_pose.pose.orientation.z = qz;
    req->kinematics_pose.pose.orientation.w = qw;
    
    req->path_time = 4.0;
    
    // 전송 후 입력 버퍼 비우기 (중요: 키보드 인터럽트 오작동 방지)
    int ch;
    while ((ch = getchar()) != '\n' && ch != EOF);

    task_client_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Sent absolute Pose Task with Orientation.");
    }
 
 

    // Gripper
    else if (c == 'g') sendGripper(0.01);
    else if (c == 'f') sendGripper(-0.01);
  }

  void printUI()
  {
    auto j = getJoints();
    auto p = getPose();
    printf("\n--- OpenManipulator Teleop ---\n");
    printf("WASD/ZX: move task space (XYZ)\n");
    printf("Y/H, U/J, I/K, O/L, P/; : Joint 1~5\n");
    printf("G/F: gripper open/close | Q: quit\n");
    printf("------------------------------\n");
    printf("Joint State: ");
    for(int i=0; i<NUM_OF_JOINT; i++) printf("j%d: %.2f ", i+1, j[i]);
    printf("\nPose: X:%.3f Y:%.3f Z:%.3f\n", p[0], p[1], p[2]);
    printf("\nOrientation: X:%.3f Y:%.3f Z:%.3f\n", p[3], p[4], p[5]);
  }

private:
  void jointCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < msg->name.size(); i++) {
      for (size_t j = 0; j < joint_names_.size(); j++) {
        if (msg->name[i] == joint_names_[j]) present_joint_angle_[j] = msg->position[i];
      }
    }
  }

  void poseCb(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    present_pose_[0] = msg->pose.position.x;
    present_pose_[1] = msg->pose.position.y;
    present_pose_[2] = msg->pose.position.z;
    present_pose_[3] = msg->pose.orientation.x;
    present_pose_[4] = msg->pose.orientation.y;
    present_pose_[5] = msg->pose.orientation.z;
  }

  std::vector<double> getPose() { std::lock_guard<std::mutex> lock(mtx_); return present_pose_; }
  std::vector<double> getJoints() { std::lock_guard<std::mutex> lock(mtx_); 
  return present_joint_angle_; }


  void sendJoint(const std::vector<double>& delta_joints) {
    if (!joint_from_present_client_->wait_for_service(std::chrono::milliseconds(100))) return;
    auto req = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    req->joint_position.joint_name = joint_names_;
    req->joint_position.position = delta_joints; // 컨트롤러가 현재값에 더함
    req->path_time = path_time_;
    joint_from_present_client_->async_send_request(req);
  }

  void sendTask(const std::vector<double>& delta_pose) {
    if (!task_from_present_client_->wait_for_service(std::chrono::milliseconds(100))) return;
    auto req = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
    
    req->planning_group = "gripper"; 
    req->end_effector_name = "gripper"; 
    req->kinematics_pose.pose.position.x = delta_pose[0];
    req->kinematics_pose.pose.position.y = delta_pose[1];
    req->kinematics_pose.pose.position.z = delta_pose[2];
    req->path_time = path_time_;
    task_from_present_client_->async_send_request(req);
  }

  void sendGripper(double val) {
    if (!tool_client_->wait_for_service(std::chrono::milliseconds(100))) return;
    auto req = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    req->joint_position.joint_name = {"gripper"};
    req->joint_position.position = {val};
    tool_client_->async_send_request(req);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<open_manipulator_msgs::msg::KinematicsPose>::SharedPtr pose_sub_;
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr joint_from_present_client_, joint_client_, tool_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr task_from_present_client_;
  rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr task_client_;
  
  rclcpp::Client<open_manipulator_msgs::srv::Test>::SharedPtr test_client_;
    
  std::vector<double> present_joint_angle_, present_pose_;
  std::vector<std::string> joint_names_;
  std::mutex mtx_;
  double delta_, joint_delta_, path_time_;
};


static struct termios oldt;
void initTerm() {
  struct termios newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}
void restoreTerm() { tcsetattr(STDIN_FILENO, TCSANOW, &oldt); }



// 키보드 입력 처리를 위한 메인 함수
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenManipulatorTeleop>();

  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  node->printUI();

  fd_set set;
  struct timeval timeout;

  while (rclcpp::ok()) {
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    if (select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout) > 0) {
      char c = getchar();
      if (c == 'q' || c == 'Q') break;
      node->setGoal(c);
      node->printUI();
    }
    rclcpp::spin_some(node);
  }
  restoreTerm();
  rclcpp::shutdown();
  return 0;
}

