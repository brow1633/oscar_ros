#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Transform, Quaternion
from nav_msgs.msg import Odometry
import tf2_geometry_msgs  # necessary for doTransform operations
import tf2_ros
import tf_transformations as tf_trans
import time

class MapToOdomPublisher(Node):

    def __init__(self):
        super().__init__('map_to_odom_publisher')
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform = TransformStamped()
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.listener_callback_pose,
            10)
        self.subscription_gps = self.create_subscription(
            Odometry,
            '/odometry/gps',
            self.listener_callback_gps,
            10)

    def get_odom_to_base_link(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            return trans
        except:
            self.get_logger().warn('Transform extrapolation error')
        return None

    def listener_callback_pose(self, msg):
        odom_to_base_link = self.get_odom_to_base_link()
        if odom_to_base_link:
            odom_to_base_link = odom_to_base_link.transform
            # Get the difference in orientations
            q1 = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            q2 = [odom_to_base_link.rotation.x, odom_to_base_link.rotation.y,
                  odom_to_base_link.rotation.z, odom_to_base_link.rotation.w]
            q2_conjugate = tf_trans.quaternion_conjugate(q2)
            delta_orientation = tf_trans.quaternion_multiply(q1, q2_conjugate)
            q_out = Quaternion()
            q_out.x = delta_orientation[0]
            q_out.y = delta_orientation[1]
            q_out.z = delta_orientation[2]
            q_out.w = delta_orientation[3]
            self.transform.transform.rotation = q_out

    def listener_callback_gps(self, msg):
        odom_to_base_link = self.get_odom_to_base_link()
        if odom_to_base_link:
            # Get the difference in positions
            delta_position = msg.pose.pose.position
            delta_position.x -= odom_to_base_link.transform.translation.x
            delta_position.y -= odom_to_base_link.transform.translation.y
            delta_position.z = 0.0
            self.transform.transform.translation.x = delta_position.x
            self.transform.transform.translation.y = delta_position.y
            self.transform.transform.translation.z = delta_position.z
            self.transform.header.stamp = self.get_clock().now().to_msg()
            self.transform.header.frame_id = "map"
            self.transform.child_frame_id = "odom"
            self.br.sendTransform(self.transform)

def main(args=None):
    time.sleep(10)
    rclpy.init(args=args)
    map_to_odom_publisher = MapToOdomPublisher()
    rclpy.spin(map_to_odom_publisher)
    map_to_odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

