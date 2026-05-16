#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, Quaternion
from tf2_ros import Buffer, TransformListener
import threading
import time
from math import radians, copysign, atan2, pi

class CalibrateAngular(Node):
    def __init__(self):
        super().__init__('calibrate_angular')

        # 1. 파라미터 선언 및 초기화
        self.declare_parameter('test_angle', 360.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('tolerance', 1.5)
        self.declare_parameter('odom_angular_scale_correction', 1.05)
        self.declare_parameter('start_test', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate', 50.0)

        self.update_parameters()

        # 2. TF2 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 발행자 설정
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 5)

        # 4. 상태 변수 및 스레드 설정
        self.stop_event = threading.Event()
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 제어 루프를 별도 스레드에서 실행 (블로킹 방지)
        self.test_thread = threading.Thread(target=self.run_test)
        self.test_thread.start()

        self.get_logger().info("회전 각도 교정 노드가 시작되었습니다.")
        self.get_logger().info("rqt_reconfigure 또는 'ros2 param set'으로 start_test를 true로 바꿔 시작하세요.")

    def update_parameters(self):
        self.test_angle_raw = self.get_parameter('test_angle').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = radians(self.get_parameter('tolerance').value)
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').value
        self.start_test = self.get_parameter('start_test').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.rate = self.get_parameter('rate').value

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"파라미터 변경: {param.name} -> {param.value}")
        self.update_parameters()
        return SetParametersResult(successful=True)

    def normalize_angle(self, angle):
        """각도를 -pi ~ pi 범위로 정규화"""
        while angle > pi: angle -= 2.0 * pi
        while angle < -pi: angle += 2.0 * pi
        return angle

    def quat_to_angle(self, q):
        """쿼터니언을 오일러 각(Yaw)으로 변환"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)

    def get_odom_angle(self):
        """TF를 통해 현재 로봇의 Yaw 각도 가져오기"""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, now, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return self.quat_to_angle(trans.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f"TF 변환 대기 중... {str(e)}")
            return None

    def run_test(self):
        reverse = 1
        rate_wait = 1.0 / self.rate

        while rclpy.ok():
            if self.start_test:
                self.get_logger().info("회전 테스트 시작")
                
                target_angle = radians(self.test_angle_raw) * reverse
                odom_angle = self.get_odom_angle()
                if odom_angle is None:
                    time.sleep(1.0)
                    continue

                last_angle = odom_angle
                turn_angle = 0.0
                error = target_angle - turn_angle
                
                # 방향 전환 (다음 테스트를 위해)
                reverse = -reverse

                while abs(error) > self.tolerance and self.start_test and rclpy.ok():
                    # 로봇 회전 명령
                    move_cmd = Twist()
                    move_cmd.angular.z = copysign(self.speed, error)
                    self.cmd_vel.publish(move_cmd)
                    
                    time.sleep(rate_wait)

                    # 현재 각도 업데이트
                    odom_angle = self.get_odom_angle()
                    if odom_angle is None: continue

                    # 이동한 델타 각도 계산 및 보정 계수 적용
                    delta_angle = self.odom_angular_scale_correction * self.normalize_angle(odom_angle - last_angle)
                    turn_angle += delta_angle
                    error = target_angle - turn_angle
                    last_angle = odom_angle

                    self.get_logger().info(
                        f"목표: {target_angle:.2f}, 현재 회전: {turn_angle:.2f}, 오차: {error:.2f}",
                        throttle_duration_sec=1.0
                    )

                # 테스트 종료 및 정지
                self.cmd_vel.publish(Twist())
                self.set_parameters([Parameter('start_test', value=False)])
                self.get_logger().info("회전 테스트가 완료되었습니다.")
            
            time.sleep(0.1)

    def stop_robot(self):
        self.get_logger().info("로봇을 정지합니다.")
        self.cmd_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateAngular()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
