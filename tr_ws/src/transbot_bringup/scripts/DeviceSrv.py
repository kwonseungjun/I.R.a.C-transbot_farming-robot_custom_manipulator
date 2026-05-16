#!/usr/bin/env python3
# encoding: utf-8
import usb.core
import usb.util
import rclpy
from rclpy.node import Node
from transbot_msgs.srv import CamDevice

class DeviceSrvNode(Node):
    def __init__(self):
        super().__init__('DeviceSrv')
        
        # ROS 2 서비스 생성 (서비스 타입, 서비스 이름, 콜백 함수)
        self.srv = self.create_service(CamDevice, '/CamDevice', self.cam_device_callback)
        
        self.get_logger().info("카메라 장치 확인 서비스 가동 중...")
        
        # 초기 실행 시 장치 확인 및 출력
        device_type = self.get_device()
        self.get_logger().info(f"현재 감지된 카메라: {device_type}")

    def cam_device_callback(self, request, response):
        """
        서비스 콜백 함수
        request: 서비스 요청 데이터 (현재는 사용하지 않음)
        response: 서비스 응답 객체 (반드시 반환해야 함)
        """
        # ROS 2 서비스 응답 객체의 필드명은 .msg/.srv 정의를 따르며 보통 소문자입니다.
        response.cam_device = self.get_device()
        return response

    def get_device(self):
        """
        USB 버스를 스캔하여 Astra 카메라(Vendor ID: 0x2bc5) 유무 확인
        """
        # pyusb의 최신 방식을 권장 (usb.core.find)
        # 모든 장치를 리스트로 가져옴
        devices = usb.core.find(find_all=True)
        
        id_vendors = [dev.idVendor for dev in devices]
        
        # Orbbec Astra의 Vendor ID: 0x2bc5
        if 0x2bc5 in id_vendors:
            return "Astra"
        else:
            return "USBCam"

def main(args=None):
    rclpy.init(args=args)
    
    device_node = DeviceSrvNode()
    
    try:
        rclpy.spin(device_node)
    except KeyboardInterrupt:
        pass
    finally:
        device_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
