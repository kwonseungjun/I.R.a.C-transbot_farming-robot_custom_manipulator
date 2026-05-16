#!/usr/bin/env python3
# encoding: utf-8

import sys
import os
# 하드웨어 라이브러리 경로 유지
sys.path.append("/home/irac/Transbot/transbot")
sys.path.append("/home/irac/Transbot/py_install")


import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import threading
from math import pi
from time import sleep

# ROS 2 표준 메시지 및 서비스
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
# 사용자 정의 메시지 (ROS 2용으로 빌드되어 있어야 함)
from transbot_msgs.msg import Arm, Joint, Adjust, Edition, Battery, PWMServo
from transbot_msgs.srv import RobotArm, RGBLight, Buzzer, Headlight

# 하드웨어 제어 라이브러리
from Transbot_Lib import Transbot
from arm_transbot import Transbot_ARM

class TransbotDriver(Node):
    def __init__(self):
        super().__init__('driver_node')
        
        # 1. 하드웨어 초기화
        self.bot_arm = Transbot_ARM()
        bot_arm_offset = self.bot_arm.get_arm_offset()
        self.bot = Transbot(arm_offset=bot_arm_offset)
        
        self.RA2DE = 180 / pi
        
        # 2. 파라미터 선언 및 초기값 설정 (Dynamic Reconfigure 대체)
        self.declare_parameter('imu_topic', '/transbot/imu')
        self.declare_parameter('vel_topic', '/transbot/get_vel')
        self.declare_parameter('CameraDevice', 'astra')
        self.declare_parameter('linear_speed_limit', 0.4)
        self.declare_parameter('angular_speed_limit', 2.0)
        self.declare_parameter('Kp', 1.0)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.0)

        # 파라미터 값 가져오기
        self.imu_topic = self.get_parameter('imu_topic').value
        self.vel_topic = self.get_parameter('vel_topic').value
        self.CameraDevice = self.get_parameter('CameraDevice').value
        self.linear_max = self.get_parameter('linear_speed_limit').value
        self.linear_min = 0.0
        self.angular_max = self.get_parameter('angular_speed_limit').value
        self.angular_min = 0.0

        # 3. 구독자(Subscribers) 설정
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.sub_TargetAngle = self.create_subscription(Arm, "/TargetAngle", self.sub_arm_callback, 10)
        self.sub_PWMServo = self.create_subscription(PWMServo, "/PWMServo", self.sub_pwm_servo_callback, 10)
        self.sub_Adjust = self.create_subscription(Adjust, "/Adjust", self.sub_adjust_callback, 10)

        # 4. 서비스(Services) 설정
        self.srv_CurrentAngle = self.create_service(RobotArm, "/CurrentAngle", self.robot_arm_callback)
        self.srv_RGBLight = self.create_service(RGBLight, "/RGBLight", self.rgb_light_callback)
        self.srv_Buzzer = self.create_service(Buzzer, "/Buzzer", self.buzzer_callback)
        self.srv_Headlight = self.create_service(Headlight, "/Headlight", self.headlight_callback)

        # 5. 발행자(Publishers) 설정
        self.ediPublisher = self.create_publisher(Edition, '/edition', 10)
        self.volPublisher = self.create_publisher(Battery, "/voltage", 10)
        self.velPublisher = self.create_publisher(Twist, self.vel_topic, 10)
        self.imuPublisher = self.create_publisher(Imu, self.imu_topic, 10)

        # 6. 파라미터 변경 콜백 (Dynamic Reconfigure 기능)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 7. 데이터 발행 타이머 (ROS 1의 pub_data 루프 대체, 20Hz)
        self.timer = self.create_timer(0.05, self.pub_data_timer_callback)

        self.bot.create_receive_threading()
        self.bot.set_uart_servo_angle(9, 90)
        self.get_logger().info("Transbot Driver ROS 2 Node Started")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'linear_speed_limit':
                self.linear_max = param.value
            elif param.name == 'angular_speed_limit':
                self.angular_max = param.value
            # PID 제어 등 필요한 파라미터 처리
            self.get_logger().info(f"Parameter {param.name} updated to {param.value}")
        return SetParametersResult(successful=True)

    def robot_arm_callback(self, request, response):
        joints = self.bot.get_uart_servo_angle_array()
        for i in range(3):
            if joints[i] < 0: continue
            joint = Joint()
            joint.id = i + 7
            joint.angle = float(joints[i])
            joint.run_time = 500
            response.robot_arm.joint.append(joint) # 필드명 소문자 주의
        return response

    def pub_data_timer_callback(self):
        # 센서 데이터 읽기
        ax, ay, az = self.bot.get_accelerometer_data()
        gx, gy, gz = self.bot.get_gyroscope_data()
        velocity_val, angular_val = self.bot.get_motion_data()
        voltage = self.bot.get_battery_voltage()

        # 전압 및 버전 정보 발행
        battery = Battery()
        battery.voltage = float(voltage)
        self.volPublisher.publish(battery)

        edition = Edition()
        edition.edition = float(self.bot.get_version())
        self.ediPublisher.publish(edition)

        # IMU 데이터 발행
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        self.imuPublisher.publish(imu)

        # 피드백 속도 발행
        twist = Twist()
        twist.linear.x = float(velocity_val)
        twist.angular.z = float(angular_val)
        self.velPublisher.publish(twist)

    def sub_adjust_callback(self, msg):
        self.bot.set_imu_adjust(msg.adjust)

    def sub_arm_callback(self, msg):
        for joint in msg.joint:
            if joint.run_time != 0:
                self.bot.set_uart_servo_angle(joint.id, joint.angle, joint.run_time)

    def sub_pwm_servo_callback(self, msg):
        angle = int(msg.angle)
        if self.CameraDevice == "astra":
            if msg.id == 1:
                angle = max(60, min(120, angle))
        if msg.id == 2:
            angle = min(140, angle)
        self.bot.set_pwm_servo(msg.id, angle)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        az = msg.angular.z

        # 속도 제한 로직
        vx = max(-self.linear_max, min(self.linear_max, vx))
        if 0 < abs(vx) < self.linear_min:
            vx = self.linear_min if vx > 0 else -self.linear_min
            
        az = max(-self.angular_max, min(self.angular_max, az))
        if 0 < abs(az) < self.angular_min:
            az = self.angular_min if az > 0 else -self.angular_min

        self.bot.set_car_motion(vx, az)

    def rgb_light_callback(self, request, response):
        self.bot.set_colorful_effect(request.effect, request.speed, parm=1)
        response.result = True
        return response

    def buzzer_callback(self, request, response):
        self.bot.set_beep(request.buzzer)
        response.result = True
        return response

    def headlight_callback(self, request, response):
        self.bot.set_floodlight(request.headlight) # 필드명 주의
        response.result = True
        return response

def main(args=None):
    rclpy.init(args=args)
    driver = TransbotDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 로봇 정지
        driver.bot.set_car_motion(0.0, 0.0)
        driver.get_logger().info("Stopping Transbot...")
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
