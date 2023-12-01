import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import yaml
import time
from threading import Thread
from tf_transformations import euler_from_quaternion

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.ns_cb,
            10)
        self.last_saved_time = time.time()

        self.counter = 0

        # Start the save to file thread
        self.save_thread = Thread(target=self.save_to_file)
        self.save_thread.daemon = True
        self.save_thread.start()

    def ns_cb(self, msg):
        lat = msg.longitude
        lon = msg.latitude
        # Assuming the orientation is represented as a quaternion
        # Assuming the heading is the yaw component in radians, East-referenced
        self.waypoint = ([lat, lon])

    def save_to_file(self):
        while True:
            current_time = time.time()
            if current_time - self.last_saved_time >= 0.1:
                # Save to file
                with open('waypoints.yaml', 'a') as file:
                    if self.waypoint is not None:
                        yaml.dump({'wp{}'.format(self.counter): self.waypoint}, file)
                        self.counter += 1
                    self.waypoint = None
                self.last_saved_time = current_time
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    waypoint_recorder = WaypointRecorder()
    rclpy.spin(waypoint_recorder)

    waypoint_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    open('waypoints.yaml', 'w')
    main()
