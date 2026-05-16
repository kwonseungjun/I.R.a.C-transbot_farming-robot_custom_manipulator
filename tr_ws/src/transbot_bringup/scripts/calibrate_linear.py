#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, Point
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from math import copysign, sqrt, pow
import time

class CalibrateLinear(Node):
    def __init__(self):
        super().__init__('calibrate_linear')

        # 1. 파라미터 선언 및 초기화 (Dynamic Reconfigure 대체)
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.15)
        self.declare_parameter('tolerance', 0.01)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate', 20.0)

        # 변수 로드
        self.update_parameters()

        # 2. TF2 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. 발행자 설정
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 5)

        # 4. 상태 변수
        self.position = Point()
        self.x_start = 0.0
        self.y_start = 0.0
        self.is_initialized = False

        # 5. 파라미터 업데이트 콜백
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 6. 메인 루프 타이머 (20Hz)
        timer_period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("직선 거리 교정 노드 가동. 'start_test' 파라미터를 true로 변경하여 시작하세요.")

    def update_parameters(self):
        self.test_distance = self.get_parameter('test_distance').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').value
        self.start_test = self.get_parameter('start_test').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start_test' and param.value is True:
                # 테스트 시작 시 현재 위치를 시작점으로 기록
                pos = self.get_position()
                if pos:
                    self.x_start = pos.x
                    self.y_start = pos.y
                    self.get_logger().info(f"테스트 시작! 시작 위치: ({self.x_start:.2f}, {self.y_start:.2f})")
            
            self.get_logger().info(f"파라미터 변경: {param.name} -> {param.value}")
        
        self.update_parameters()
        return SetParametersResult(successful=True)

    def get_position(self):
        try:
            # ROS 2에서는 Time(0) 대신 rclpy.time.Time() 사용
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame, 
                self.base_frame, 
                now, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return trans.transform.translation
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {str(e)}")
            return None

    def control_loop(self):
        move_cmd = Twist()

        if self.start_test:
            curr_pos = self.get_position()
            if curr_pos is None:
                return

            # 이동 거리 계산 (피타고라스)
            distance = sqrt(pow((curr_pos.x - self.x_start), 2) +
                            pow((curr_pos.y - self.y_start), 2))

            # 교정 계수 적용
            distance *= self.odom_linear_scale_correction

            # 오차 계산
            error = distance - self.test_distance

            # 목표 도달 확인
            if abs(error) < self.tolerance:
                self.get_logger().info("목표 거리에 도달했습니다.")
                self.set_parameters([Parameter('start_test', value=False)])
                self.cmd_vel.publish(Twist()) # 정지
            else:
                # 목표 방향으로 속도 설정
                move_cmd.linear.x = copysign(self.speed, -1 * error)
                self.cmd_vel.publish(move_cmd)
                
            self.get_logger().info(f"진행 거리: {distance:.3f}m / 목표: {self.test_distance}m (오차: {error:.3f}m)", throttle_duration_sec=0.5)
        else:
            # 테스트 중이 아닐 때는 정지 명령 발행 (안전)
            # 단, 너무 자주 발행하지 않도록 로직 조절 가능
            pass

    def stop_robot(self):
        self.get_logger().info("로봇을 정지합니다.")
        self.cmd_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear()
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
