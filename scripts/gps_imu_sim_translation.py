import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

class GPSToPoseNode(Node):
    def __init__(self):
        super().__init__('gps_to_pose_node')

        # Subscribers
        self.create_subscription(NavSatFix, '/gps', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publisher
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/pose', 10)

        # Initialize pose message
        self.pose_msg = PoseWithCovarianceStamped()

    def gps_callback(self, msg):
        # Update X and Y positions with Latitude and Longitude
        self.pose_msg.pose.pose.position.x = msg.latitude
        self.pose_msg.pose.pose.position.y = msg.longitude

        # Publish the updated pose message
        self.publish_pose()

    def imu_callback(self, msg):
        # Update orientation from IMU
        self.pose_msg.pose.pose.orientation = msg.orientation

        # Publish the updated pose message
        self.publish_pose()

    def publish_pose(self):
        # Set the header timestamp
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the pose message
        self.pose_publisher.publish(self.pose_msg)

def main(args=None):
    rclpy.init(args=args)
    gps_to_pose_node = GPSToPoseNode()

    try:
        rclpy.spin(gps_to_pose_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_to_pose_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

