#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix

class PoseToIMU(Node):
    def __init__(self):
        super().__init__('pose_to_imu')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'pose',
            self.listener_callback,
            10)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/fix2',
            self.gps_cb,
            10)
        self.publisher = self.create_publisher(Imu, 'gps/pose_as_imu', 10)
        self.publisher_gps = self.create_publisher(NavSatFix, 'gps/fix', 10)

    def listener_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation = msg.pose.pose.orientation
        imu_msg.orientation_covariance = [
            msg.pose.covariance[28], msg.pose.covariance[28], msg.pose.covariance[28],
            msg.pose.covariance[28], msg.pose.covariance[28], msg.pose.covariance[28],
            msg.pose.covariance[28], msg.pose.covariance[28], msg.pose.covariance[35]
        ]
        self.publisher.publish(imu_msg)
    def gps_cb(self, msg):
        pass
#        gps_msg = msg
#        gps_msg.header.stamp = self.get_clock().now().to_msg()
#        self.publisher_gps.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)

    pose_to_imu = PoseToIMU()

    rclpy.spin(pose_to_imu)

    pose_to_imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

