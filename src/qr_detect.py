#!/usr/bin/env python3 

import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge 
import cv2
from pyzbar import pyzbar

class QrScan(Node):

    def __init__(self):
        super().__init__("qr_scan")
        self.qr_code_list = set()
        
        self.create_subscription(Image, f"/camera_0/image_raw", self.camera_callback, 10)
        self.create_subscription(Image, f"/camera_1/image_raw", self.camera_callback, 10)
        self.create_subscription(Image, f"/camera_2/image_raw", self.camera_callback, 10)
        self.qr_publisher = self.create_publisher(String, '/check_qr', 10)
        self.cv_bridge = CvBridge()
    
    def camera_callback(self, data):
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)

        qr_codes = pyzbar.decode(current_frame)
        for qr in qr_codes:
            data = qr.data.decode()
            if data not in self.qr_code_list:
                self.qr_code_list.add(data)
                print(f"\nFOUND {len(self.qr_code_list)}/14: {data}\n")
                # self.get_logger.info(f"\nFOUND {len(self.qr_code_list)}/14: {data}\n")
                msg = String()
                msg.data = f"\nFOUND {len(self.qr_code_list)}/14: {data}\n"
                self.qr_publisher.publish(msg)
        if len(self.qr_code_list) == 14:
            print(f"all codes are collected !!!")
        # cv2.imshow("camera", current_frame)   
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    qr_scan = QrScan()
    rclpy.spin(qr_scan)
    qr_scan.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
    main()
