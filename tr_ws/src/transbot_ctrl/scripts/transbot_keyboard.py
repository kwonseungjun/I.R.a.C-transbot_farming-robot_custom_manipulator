#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from ultralytics import YOLO


DEBUG_SHOW = True


class ObjectFollower(Node):

    def __init__(self):
        super().__init__('object_follower_node')

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # YOLO 모델 로드
        self.model = YOLO("yolov8n.pt")

        # 추적 대상
        self.target_class = "person"

        # 최소 confidence
        self.min_conf = 0.5

        # 회전 제어 gain
        self.Kp = 0.002

        # 최대 회전 속도
        self.max_angular_speed = 0.7

        # 전진 기본 속도
        self.linear_base = 0.12

        self.get_logger().info(
            f"Object Follower Node Started - Target: {self.target_class}"
        )

    def yolo_follow(self, frame):

        # YOLO 추론
        results = self.model(frame, verbose=False)[0]

        best_box = None
        best_conf = 0.0

        H, W = frame.shape[:2]

        # 가장 confidence 높은 target 선택
        for box in results.boxes:

            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            label = self.model.names[cls_id]

            if label != self.target_class:
                continue

            if conf < self.min_conf:
                continue

            if conf > best_conf:
                best_conf = conf
                best_box = box

        twist = Twist()

        # 객체 발견
        if best_box is not None:

            # tensor -> float 변환
            x1, y1, x2, y2 = map(
                float,
                best_box.xyxy[0]
            )

            # 중심 좌표
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0

            # bbox 크기
            area = (x2 - x1) * (y2 - y1)

            # 화면 중심 오차
            error = cx - (W / 2.0)

            # 회전 제어
            angular_z = -self.Kp * error

            angular_z = np.clip(
                angular_z,
                -self.max_angular_speed,
                self.max_angular_speed
            )

            # 객체 크기 기반 속도 제어
            speed = self.linear_base * (80000.0 / (area + 1.0))

            speed = np.clip(
                speed,
                0.05,
                0.2
            )

            # cmd_vel 설정
            twist.linear.x = float(speed)
            twist.angular.z = float(angular_z)

            # 시각화
            if DEBUG_SHOW:

                cv2.rectangle(
                    frame,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )

                cv2.circle(
                    frame,
                    (int(cx), int(cy)),
                    6,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f"{label} {best_conf:.2f}",
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

            return twist, frame, True

        # 객체 미발견
        return Twist(), frame, False

    def run(self):

        # 카메라 열기
        cap = cv2.VideoCapture(6, cv2.CAP_V4L2)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        if not cap.isOpened():
            self.get_logger().error("Camera open failed!")
            return

        while rclpy.ok():

            ret, frame = cap.read()

            if not ret:
                self.get_logger().error("Camera frame read failed!")
                break

            # YOLO 추적
            twist, vis, found = self.yolo_follow(frame)

            # 객체 발견 시 추적
            if found:
                self.cmd_pub.publish(twist)

            # 객체 없으면 정지
            else:
                self.cmd_pub.publish(Twist())

            # 디버그 화면
            if DEBUG_SHOW:

                cv2.imshow(
                    "YOLO Object Tracking",
                    vis
                )

                key = cv2.waitKey(1)

                if key & 0xFF == ord('q'):
                    break

        # 종료 처리
        cap.release()

        cv2.destroyAllWindows()

    def stop_robot(self):

        stop_msg = Twist()

        self.cmd_pub.publish(stop_msg)


def main(args=None):

    rclpy.init(args=args)

    node = ObjectFollower()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
