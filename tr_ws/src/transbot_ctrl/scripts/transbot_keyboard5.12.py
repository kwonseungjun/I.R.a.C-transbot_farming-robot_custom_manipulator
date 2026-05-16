#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Transbot (ROS 2 Humble)!
---------------------------
Moving around:
    u    i    o
    j    k    l
    m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class TeleopNode(Node):
    def __init__(self, settings):
        super().__init__('transbot_keyboard')
        self.settings = settings
        
        # 파라미터 선언 및 가져오기
        self.declare_parameter('linear_limit', 0.45)
        self.declare_parameter('angular_limit', 2.0)
        self.linear_limit = self.get_parameter('linear_limit').value
        self.angular_limit = self.get_parameter('angular_limit').value
        
        # 발행자 설정
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 초기 변수
        self.speed = 0.2
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self):
        return f"currently:\tspeed {self.speed:.2f}\tturn {self.turn:.2f} "

    def run(self):
        try:
            print(msg)
            print(self.vels())
            while rclpy.ok():
                key = self.get_key()
                
                # 이동 키 확인
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    self.count = 0
                # 속도 조절 키 확인
                elif key in speedBindings.keys():
                    self.speed = min(self.linear_limit, self.speed * speedBindings[key][0])
                    self.turn = min(self.angular_limit, self.turn * speedBindings[key][1])
                    self.count = 0
                    print(self.vels())
                    if self.status == 14:
                        print(msg)
                    self.status = (self.status + 1) % 15
                # 정지 키 확인
                elif key == ' ' or key == 'k':
                    self.x = 0
                    self.th = 0
                # 기타 키 입력 시 점진적 정지 (Smooth Stop)
                else:
                    self.count += 1
                    if self.count > 4:
                        self.x = 0
                        self.th = 0
                    if key == '\x03': # CTRL-C
                        break
                
                # 메시지 발행
                twist = Twist()
                twist.linear.x = float(self.speed * self.x)
                twist.angular.z = float(self.turn * self.th)
                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"에러 발생: {e}")
        finally:
            # 종료 시 정지 명령
            self.pub.publish(Twist())

def main(args=None):
    # 키보드 설정 저장
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    teleop_node = TeleopNode(settings)
    
    try:
        teleop_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # 터미널 설정 복구
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
