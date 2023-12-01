import sys
import time
import yaml

from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
import serial

class GpsWaypointPublisher(Node):
    def __init__(self):
        self.feedback = None
        self.result_future = None
        self.status = None
        self.goal_handle = None

        self.waypoint_poses = []

        super().__init__('gps_waypoint_publisher')
        self.cli = self.create_client(FromLL, 'fromLL')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = FromLL.Request()

    def convertLL(self, long, lat) -> Point:
        self.req.ll_point.latitude = lat
        self.req.ll_point.longitude = long
        self.req.ll_point.altitude = 0.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result().map_point

    def loadFile(self, f):
        with open(f, 'r') as file:
            waypoints = yaml.safe_load(file)['waypoints']
            print(waypoints)
        for wp in waypoints:
            lat = waypoints[wp][0]
            long = waypoints[wp][1]
            ang = waypoints[wp][2]

            wp_local_pos: Point = self.convertLL(lat, long)
            wp_quat = quaternion_from_euler(0, 0, ang)

            wp_pose = PoseStamped()
            wp_pose.header.frame_id = 'map'
            wp_pose.header.stamp = self.get_clock().now().to_msg()
            wp_pose.pose.position.x = wp_local_pos.x
            wp_pose.pose.position.y = wp_local_pos.y
            wp_pose.pose.position.z = 0.0
            wp_pose.pose.orientation.x = wp_quat[0]
            wp_pose.pose.orientation.y = wp_quat[1]
            wp_pose.pose.orientation.z = wp_quat[2]
            wp_pose.pose.orientation.w = wp_quat[3]
            self.waypoint_poses.append(wp_pose)
        print(self.waypoint_poses)

    def followWaypoints(self):
        # Sends a `FollowWaypoints` action request
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoint_poses

        self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Following ' + str(len(poses)) + ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True

        rclpy.spin_until_future_complete(self, self.result_future)

        if not self.result_future:
            return True
#        if self.result_future.result():
#            self.status = self.result_future.result().status
#            if self.status != GoalStatus.STATUS_SUCCEEDED:
#                self.debug('Goal with failed with status code: {0}'.format(self.status))
#            return True
#        else:
#            # Timed out, still processing, not complete yet
#            return False


    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return



def main(args=None):
    init_time = time.time()
    avg_bat_volt = None
    sendWaypoints = True
    counter = 0
    rclpy.init(args=args)
    gps_waypoint_publisher = GpsWaypointPublisher()
    gps_waypoint_publisher.loadFile(sys.argv[1])
    #ser = serial.Serial("/dev/ttyUSB2", 115200)
    time.sleep(0.5)

    while(True):
        #bat_volt = eval(ser.readline())
        bat_volt = 30
        if(avg_bat_volt == None):
            avg_bat_volt = bat_volt
        else:
            avg_bat_volt = 0.9*avg_bat_volt + 0.1*bat_volt
        if(avg_bat_volt < 22.5):
            print(f"Battery voltage dropped below threshold.  Cycles: {counter}, Time: {(time.time() - init_time)/60}")
            break
        gps_waypoint_publisher.followWaypoints()
        counter += 1
        print(f"Number of cycles: {counter}, time: {(time.time() - init_time)/60}, battery voltage: {avg_bat_volt}")
        sendWaypoints = gps_waypoint_publisher.isNavComplete()

if __name__ == '__main__':
    main()
