#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import getpass
import threading
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
# [주의] transbot_msgs가 ROS 2용으로 빌드되어 있어야 합니다.
from transbot_msgs.srv import RobotArm, RGBLight, Buzzer, CamDevice, Headlight
from transbot_msgs.msg import PWMServo, Arm, Joint, JoyState, Adjust

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('transbot_joy')
        
        # 1. 파라미터 선언 및 가져오기
        self.declare_parameter('linear_speed_limit', 0.45)
        self.declare_parameter('angular_speed_limit', 2.0)
        self.linear_speed_limit = self.get_parameter('linear_speed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

        # 2. 초기 변수 설정
        self.user_name = getpass.getuser()
        self.linear_Gear = 1.0
        self.angular_Gear = 1.0
        self.Buzzer_value = False
        self.Headlight_value = False
        self.CameraDevice = "Astra"
        self.Joy_active = False
        self.cancel_time = time.time()
        self.RGBLight_value = 0
        
        self.PWMServo_X = 90
        self.PWMServo_Y = 90
        self.joint1, self.joint2, self.joint3 = 225, 45, 90
        self.joint_key1, self.joint_key2, self.joint_key3, self.joint_key4 = 0, 0, 0, 0
        self.Servo_leftX, self.Servo_rightB, self.Servo_downA, self.Servo_upY = 0, 0, 0, 0

        # 3. 통신 설정 (Callback Group을 사용하여 병렬 처리 허용)
        self.cb_group = ReentrantCallbackGroup()

        # 발행자(Publishers)
        self.cmdVelPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_PWMServo = self.create_publisher(PWMServo, '/PWMServo', 10)
        self.pub_Arm = self.create_publisher(Arm, '/TargetAngle', 10)
        self.pub_JoyState = self.create_publisher(JoyState, '/JoyState', 10)
        self.pub_Adjust = self.create_publisher(Adjust, '/Adjust', 10)

        # 서비스 클라이언트(Service Clients)
        self.RobotArm_client = self.create_client(RobotArm, '/CurrentAngle', callback_group=self.cb_group)
        self.RGBLight_client = self.create_client(RGBLight, '/RGBLight', callback_group=self.cb_group)
        self.Buzzer_client = self.create_client(Buzzer, '/Buzzer', callback_group=self.cb_group)
        self.CamDevice_client = self.create_client(CamDevice, '/CamDevice', callback_group=self.cb_group)
        self.Headlight_client = self.create_client(Headlight, '/Headlight', callback_group=self.cb_group)

        # 구독자(Subscriber)
        self.sub_Joy = self.create_subscription(Joy, '/joy', self.buttonCallback, 10, callback_group=self.cb_group)

        # 4. 타이머 기반 루프 (기존 threading 대체 권장)
        self.timer_period = 0.05  # 20Hz
        self.arm_timer = self.create_timer(self.timer_period, self.analyse_PWM, callback_group=self.cb_group)
        
        # 초기 장치 정보 가져오기
        self.init_device_info()
        self.get_logger().info("NavSatTransform (ROS 2 Humble) 정식 가동 준비 완료")

    def init_device_info(self):
        # 서비스 호출 스레드 분리
        threading.Thread(target=self.call_initial_services).start()

    def call_initial_services(self):
        # 로봇 팔 및 카메라 정보 초기화
        self.call_service_sync(self.RobotArm_client, RobotArm.Request(apply="getJoint"))
        self.call_service_sync(self.CamDevice_client, CamDevice.Request(get_gev="GetCamDevice"))

    def call_service_sync(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service {client.srv_name} not available")
            return None
        future = client.call_async(request)
        return future

    def buttonCallback(self, joy_data):
        if self.user_name == "pi": self.user_pi(joy_data)
        else: self.user_jetson(joy_data)

    def user_jetson(self, joy_data):
        # 기존 로직 유지하되 발행 방식 변경
        linear_speed = joy_data.axes[1] * self.linear_speed_limit * self.linear_Gear
        angular_speed = joy_data.axes[2] * self.angular_speed_limit * self.angular_Gear
        
        # 정지 시 보정 명령
        adjust_msg = Adjust()
        adjust_msg.adjust = (linear_speed != 0)
        self.pub_Adjust.publish(adjust_msg)

        twist = Twist()
        twist.linear.x = float(linear_speed)
        twist.angular.z = float(angular_speed)
        self.cmdVelPublisher.publish(twist)

        # 버튼 이벤트 (기존 인덱스 유지)
        if joy_data.buttons[13] == 1: # Gear control
            self.update_linear_gear()
        if joy_data.buttons[14] == 1:
            self.update_angular_gear()
            
        # 헬퍼 변수 업데이트 (analyse_PWM에서 사용)
        self.joint_key1 = joy_data.axes[6]
        self.joint_key2 = joy_data.axes[7]
        self.joint_key3 = float(joy_data.buttons[6])
        self.joint_key4 = float(joy_data.buttons[8])
        
        self.Servo_leftX = joy_data.buttons[1]
        self.Servo_rightB = joy_data.buttons[3]
        self.Servo_downA = joy_data.buttons[4]
        self.Servo_upY = joy_data.buttons[0]

        # 서비스 기반 제어 (Buzzer, Light 등)
        if joy_data.buttons[11] != 0:
            self.Buzzer_value = not self.Buzzer_value
            self.Buzzer_client.call_async(Buzzer.Request(buzzer=int(self.Buzzer_value)))
            
        if joy_data.buttons[10] != 0:
            self.Headlight_value = not self.Headlight_value
            val = 100 if self.Headlight_value else 0
            self.Headlight_client.call_async(Headlight.Request(headlight=val))

    def update_linear_gear(self):
        if self.linear_Gear >= 1.0: self.linear_Gear = 1.0/3.0
        elif self.linear_Gear <= 1.0/3.0: self.linear_Gear = 2.0/3.0
        else: self.linear_Gear = 1.0

    def update_angular_gear(self):
        if self.angular_Gear >= 1.0: self.angular_Gear = 1.0/4.0
        elif self.angular_Gear <= 1.0/4.0: self.angular_Gear = 1.0/2.0
        elif self.angular_Gear <= 1.0/2.0: self.angular_Gear = 3.0/4.0
        else: self.angular_Gear = 1.0

    def analyse_PWM(self):
        # 타이머 루프에서 실행되는 제어 로직
        if any([self.joint_key1, self.joint_key2, self.joint_key3, self.joint_key4]):
            self.joint1 = max(0, min(225, self.joint1 + self.joint_key1))
            self.joint2 = max(30, min(270, self.joint2 + self.joint_key2))
            self.joint3 = max(60, min(180, self.joint3 + self.joint_key3 - self.joint_key4))
            
            arm_msg = Arm()
            for i, ang in enumerate([self.joint1, self.joint2, self.joint3]):
                joint = Joint()
                joint.id = 7 + i
                joint.angle = float(ang)
                joint.run_time = 50 # 50ms
                arm_msg.joint.append(joint)
            self.pub_Arm.publish(arm_msg)

        # 팬/틸트(PTZ) 제어
        if any([self.Servo_leftX, self.Servo_rightB, self.Servo_downA, self.Servo_upY]):
            self.PWMServo_X = max(60, min(120, self.PWMServo_X - self.Servo_leftX + self.Servo_rightB))
            # Astra 카메라 외의 경우 Y축도 제어
            if self.CameraDevice != "Astra":
                self.PWMServo_Y = max(0, min(180, self.PWMServo_Y - self.Servo_downA + self.Servo_upY))
            
            pwm_msg = PWMServo()
            pwm_msg.id = 1
            pwm_msg.angle = int(self.PWMServo_X)
            self.pub_PWMServo.publish(pwm_msg)
            
            if self.CameraDevice != "Astra":
                pwm_msg.id = 2
                pwm_msg.angle = int(self.PWMServo_Y)
                self.pub_PWMServo.publish(pwm_msg)

    # user_pi 로직은 user_jetson과 유사하게 인덱스만 맞춰 구현 가능 (지면상 생략)

def main(args=None):
    rclpy.init(args=args)
    joy_teleop = JoyTeleop()
    
    # 멀티스레드 실행기를 사용하여 서비스 대기 중에도 조이스틱 입력을 처리
    executor = MultiThreadedExecutor()
    executor.add_node(joy_teleop)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        joy_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
