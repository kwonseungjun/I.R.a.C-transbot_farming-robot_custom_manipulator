#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

DEBUG_SHOW = True  # 연구실 모니터에서 확인하려면 True

def gstreamer_pipeline(capture_width=640, capture_height=480, 
                       display_width=640, display_height=480, 
                       framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (capture_width, capture_height, framerate, 
           flip_method, display_width, display_height)
    )

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # 1. 발행자(Publishers)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_pub = self.create_publisher(String, '/t2m', 10)
        self.detect_pub = self.create_publisher(String, '/t2d', 10)
        
        # 2. 구독자(Subscriber)
        self.sub_d2t = self.create_subscription(String, '/d2t', self.d2t_callback, 10)
        
        # 3. 상태 및 제어 변수
        self.running = 0
        self.camera_control_active = True
        self.STATE_LINE = "LINE_TRACING"
        self.STATE_STOP = "STOPPED_BY_ORANGE"
        self.state = self.STATE_LINE
        
        self.linear_speed = 0.1
        self.max_angular_speed = 0.7
        self.min_angular_speed = -0.7
        self.Kp = 0.003
        
        # 4. 이미지 처리 변수
        self.W, self.H = 0, 0
        self.M = None
        self.W_ratio, self.H_ratio = 0.60, 0.60
        self.bev_top_y_ratio, self.bev_bot_y_ratio = 0.45, 0.95
        self.bev_top_dx_ratio, self.bev_bot_dx_ratio = 0.25, 0.35
        self.roi_y_ratio = 0.55
        self.MIN_AREA = 1000
        self.ignore_orange_until = 0
        self.insert_init = False

        self.get_logger().info("Line Follower ROS 2 Node Started")

    def d2t_callback(self, msg):
        self.get_logger().info(f"Received mission complete: {msg.data}")
        self.running = 1

    def init_dimensions(self, frame0):
        H0, W0 = frame0.shape[:2]
        self.W = int(W0 * self.W_ratio)
        self.H = int(H0 * self.H_ratio)
        
        top_y = int(self.H * self.bev_top_y_ratio)
        bot_y = int(self.H * self.bev_bot_y_ratio)
        top_dx = int(self.W * self.bev_top_dx_ratio)
        bot_dx = int(self.W * self.bev_bot_dx_ratio)
        
        src = np.float32([
            [self.W/2 - top_dx, top_y], [self.W/2 + top_dx, top_y],
            [self.W/2 + bot_dx, bot_y], [self.W/2 - bot_dx, bot_y],
        ])
        dst = np.float32([[0,0],[self.W,0],[self.W,self.H],[0,self.H]])
        self.M = cv2.getPerspectiveTransform(src, dst)

    def scan_for_orange(self, frame):
        if time.time() < self.ignore_orange_until:
            return False
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([8, 80, 30])
        upper_orange = np.array([28, 255, 230])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            if cv2.contourArea(contour) > self.MIN_AREA:
                return True
        return False

    def preprocess_image(self, frame0):
        proc = cv2.resize(frame0, (self.W, self.H), interpolation=cv2.INTER_AREA)
        bev = cv2.warpPerspective(proc, self.M, (self.W, self.H))
        bev_gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(bev_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        roi_y0 = int(self.H * self.roi_y_ratio)
        binary_img[:roi_y0, :] = 0
        return binary_img, bev, proc

    def calculate_motor_control(self, binary_img):
        twist = Twist()
        moments = cv2.moments(binary_img)
        
        if moments['m00'] > self.MIN_AREA:
            cx = int(moments['m10'] / moments['m00'])
            error = cx - (self.W / 2.0)
            angular_z = -self.Kp * error
            angular_z = np.clip(angular_z, self.min_angular_speed, self.max_angular_speed)
            
            turn_ratio = abs(angular_z) / self.max_angular_speed
            speed_ratio = max(0.3, 1.0 - turn_ratio)
            
            twist.linear.x = self.linear_speed * speed_ratio
            twist.angular.z = float(angular_z)
            return twist, f"Err:{error:.1f}, Turn:{angular_z:.2f}", cx
        else:
            return twist, "LINE NOT FOUND", None

    def run(self):
        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            self.get_logger().error("Could not open camera!")
            return
        
        while rclpy.ok():
            ret, frame0 = cap.read()
            if not ret: break
            
            if self.M is None:
                self.init_dimensions(frame0)
            
            # 초기화 전송 (1회)
            if not self.insert_init:
                self.command_pub.publish(String(data='init'))
                time.sleep(2.0)
                self.insert_init = True

            twist = Twist()
            display_text = ""
            cx = None

            # 1. 주황색 감지 로직 (정지 및 미션 대기)
            if self.scan_for_orange(frame0):
                self.get_logger().warn("Orange Detected! Stopping for mission.")
                self.cmd_pub.publish(Twist()) # 정지 명령
                self.state = self.STATE_STOP
                
                self.command_pub.publish(String(data='home'))
                self.detect_pub.publish(String(data='start'))

                self.running = 0
                # 콜백 처리를 위해 루프 안에서 spin_once 실행
                while self.running == 0 and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                self.get_logger().info("Mission Complete. Resuming...")
                self.command_pub.publish(String(data='init'))
                time.sleep(2.0)
                self.state = self.STATE_LINE
                self.ignore_orange_until = time.time() + 2.0

            # 2. 라인 트레이싱 모드
            elif self.state == self.STATE_LINE:
                binary_img, bev_vis, proc = self.preprocess_image(frame0)
                twist, display_text, cx = self.calculate_motor_control(binary_img)
                
                if self.camera_control_active:
                    self.cmd_pub.publish(twist)

                if DEBUG_SHOW:
                    # 시각화 데이터 생성
                    if cx: cv2.circle(bev_vis, (cx, int(self.H/2)), 5, (0, 0, 255), -1)
                    cv2.putText(bev_vis, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.imshow("BEV View", bev_vis)
                    cv2.imshow("Binary", binary_img)
                    if cv2.waitKey(1) & 0xFF == ord('q'): break

            # 메인 루프에서 구독자 데이터를 주기적으로 확인
            rclpy.spin_once(self, timeout_sec=0.001)

        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist()) # 종료 시 로봇 정지
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
