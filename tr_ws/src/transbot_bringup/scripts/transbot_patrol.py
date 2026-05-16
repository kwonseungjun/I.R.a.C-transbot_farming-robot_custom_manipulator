#!/usr/bin/env python3
# coding:utf-8

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import numpy as np
import threading
from time import sleep
from math import radians, copysign, sqrt, pow, atan2

from transbot_msgs.msg import JoyState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion, TransformStamped

class TransbotPatrol(Node):
    def __init__(self):
        super().__init__('TransbotPatrol')
        
        # 1. 파라미터 선언 (기존 Dynamic Reconfigure 대체)
        self.declare_parameter('Linear', 0.3)
        self.declare_parameter('Angular', 1.0)
        self.declare_parameter('Length', 1.0)
        self.declare_parameter('Angle', 360.0)
        self.declare_parameter('ResponseDist', 0.7)
        self.declare_parameter('LaserAngle', 20)
        self.declare_parameter('LineScaling', 1.0)
        self.declare_parameter('RotationScaling', 1.0)
        self.declare_parameter('LineTolerance', 0.1)
        self.declare_parameter('RotationTolerance', 0.3)
        self.declare_parameter('Command', 'finish')
        self.declare_parameter('SetLoop', False)
        self.declare_parameter('Switch', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # 변수 초기화
        self.moving = True
        self.Joy_active = False
        self.warning = 1
        self.command_src = "finish"
        
        # TF2 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 발행자 및 구독자
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 5)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.registerScan, 10)
        self.sub_Joy = self.create_subscription(JoyState, '/JoyState', self.JoyStateCallback, 10)

        # 파라미터 업데이트 콜백 설정
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 초기 파라미터 로드
        self.update_internal_params()

        # 메인 프로세스 스레드 시작
        self.process_thread = threading.Thread(target=self.process)
        self.process_thread.daemon = True
        self.process_thread.start()

        self.get_logger().info("Transbot Patrol Node (ROS2) Started. Use 'ros2 param set' or rqt_reconfigure.")

    def update_internal_params(self):
        self.Linear = self.get_parameter('Linear').value
        self.Angular = self.get_parameter('Angular').value
        self.Length = self.get_parameter('Length').value
        self.Angle = self.get_parameter('Angle').value
        self.ResponseDist = self.get_parameter('ResponseDist').value
        self.LaserAngle = self.get_parameter('LaserAngle').value
        self.LineScaling = self.get_parameter('LineScaling').value + 0.08
        self.RotationScaling = self.get_parameter('RotationScaling').value + 0.2
        self.LineTolerance = self.get_parameter('LineTolerance').value
        self.RotationTolerance = self.get_parameter('RotationTolerance').value
        self.Command = self.get_parameter('Command').value
        self.SetLoop = self.get_parameter('SetLoop').value
        self.Switch = self.get_parameter('Switch').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter {param.name} changed to {param.value}")
        self.update_internal_params()
        return rclpy.parameter.SetParametersResult(successful=True)

    # --- 유틸리티 함수 (기존 transform_utils 대체) ---
    def normalize_angle(self, angle):
        while angle > np.pi: angle -= 2.0 * np.pi
        while angle < -np.pi: angle += 2.0 * np.pi
        return angle

    def quat_to_angle(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)

    def get_position_and_angle(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, now)
            pos = trans.transform.translation
            angle = self.quat_to_angle(trans.transform.rotation)
            return pos, angle
        except Exception as e:
            self.get_logger().info(f"TF Lookup failed: {str(e)}")
            return None, None

    # --- 순찰 로직 ---
    def process(self):
        while rclpy.ok():
            if self.Switch:
                if self.Command == "LengthTest":
                    if self.advancing(self.Length): self.set_param('Command', 'finish')
                elif self.Command == "AngleTest":
                    if self.Spin(self.Angle): self.set_param('Command', 'finish')
                elif self.Command == "Triangle":
                    self.Triangle(0, 135)
                elif self.Command == "Square":
                    self.Square(0, 90)
                # Circle 등 나머지 모양은 기존 로직과 동일하게 추가
                
                if self.Command == "finish":
                    self.pub_cmdVel.publish(Twist())
                    if not self.SetLoop:
                        self.set_param('Switch', False)
                    else:
                        self.set_param('Command', self.command_src)
            sleep(0.05) # 20Hz

    def advancing(self, target_distance):
        start_pos, _ = self.get_position_and_angle()
        if start_pos is None: return False
        
        while rclpy.ok():
            if not self.Switch: return False
            curr_pos, _ = self.get_position_and_angle()
            distance = sqrt(pow(curr_pos.x - start_pos.x, 2) + pow(curr_pos.y - start_pos.y, 2))
            distance *= self.LineScaling
            
            error = distance - target_distance
            if abs(error) <= self.LineTolerance:
                self.pub_cmdVel.publish(Twist())
                return True
            
            if self.Joy_active or self.warning > 10:
                self.pub_cmdVel.publish(Twist())
                self.moving = False
                continue
            
            move_cmd = Twist()
            move_cmd.linear.x = self.Linear
            self.pub_cmdVel.publish(move_cmd)
            self.moving = True
            sleep(0.05)

    def Spin(self, angle_deg):
        target_angle = radians(angle_deg)
        _, last_angle = self.get_position_and_angle()
        turn_angle = 0
        
        while rclpy.ok():
            if not self.Switch: return False
            _, curr_angle = self.get_position_and_angle()
            delta_angle = self.RotationScaling * self.normalize_angle(curr_angle - last_angle)
            turn_angle += delta_angle
            last_angle = curr_angle
            
            error = target_angle - turn_angle
            if abs(error) < self.RotationTolerance:
                self.pub_cmdVel.publish(Twist())
                return True
            
            if self.Joy_active or self.warning > 10:
                self.pub_cmdVel.publish(Twist())
                self.moving = False
                continue
            
            move_cmd = Twist()
            move_cmd.angular.z = copysign(self.Angular, error)
            self.pub_cmdVel.publish(move_cmd)
            self.moving = True
            sleep(0.05)

    # --- 콜백 함수들 ---
    def JoyStateCallback(self, msg):
        self.Joy_active = msg.state
        if not self.Joy_active:
            self.pub_cmdVel.publish(Twist())

    def registerScan(self, scan_data):
        if self.ResponseDist == 0 or not self.moving: return
        ranges = np.array(scan_data.ranges)
        ranges = np.where(ranges == 0, 10.0, ranges) # 0은 무한대로 간주
        
        sorted_indices = np.argsort(ranges)
        self.warning = 1
        
        # 단순화된 장애물 체크 (LaserAngle 범위 내)
        for i in sorted_indices:
            angle_idx = i
            # 360도 레이더 기준 (오린 NX 트랜스봇 기본 사양 가정)
            if angle_idx < self.LaserAngle or angle_idx > (360 - self.LaserAngle):
                if ranges[i] < self.ResponseDist:
                    self.warning += 1

    def set_param(self, name, value):
        new_param = rclpy.parameter.Parameter(name, rclpy.Parameter.Type.BOOL if isinstance(value, bool) else rclpy.Parameter.Type.STRING, value)
        self.set_parameters([new_param])

def main(args=None):
    rclpy.init(args=args)
    patrol = TransbotPatrol()
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        pass
    finally:
        patrol.pub_cmdVel.publish(Twist())
        patrol.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
