#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest

class CamInf(Node):
    def __init__(self):
        super().__init__('camera_info')
        self.subscription = self.create_subscription(
            Image,
            'rgb',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(CameraInfo, 'camera_info', 10)

    def listener_callback(self, msg):
        cam_inf = CameraInfo()

        width = msg.width
        height = msg.height

        cam_inf.header = msg.header
        cam_inf.height = height
        cam_inf.width = width
        cam_inf.distortion_model = "plumb_bob"
        cam_inf.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        cx = width / 2
        cy = height / 2
        fx = cx
        fy = cy

        cam_inf.k = [fx, 0.0, cx,
                     0.0, fy, cy,
                     0.0,  0.0,  1.0]

        cam_inf.p = [fx, 0.0, cx, 0.0,
                     0.0, fy, cy, 0.0,
                     0.0,  0.0,  1.0, 0.0]

        cam_inf.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_inf.binning_x = 0
        cam_inf.binning_y = 0
        cam_inf.roi = RegionOfInterest()

        self.publisher.publish(cam_inf)

def main(args=None):
    rclpy.init(args=args)

    cam_inf = CamInf()

    rclpy.spin(cam_inf)

    cam_inf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

