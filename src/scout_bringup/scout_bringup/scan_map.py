import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import json

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
import math
import time

class ScanMap(Node):
    def __init__(self):
        super().__init__("scan_map")
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Retrieves and executes navigation instructions
        self.navigation_subscription = self.create_subscription(
            String,
            '/dfs_result',   # Can replace with bfs_result
            self.navigation_callback,
            qos
        )

        self.current_waypoint_destination = -1
        self.start_time = -1
        self.current_time = -1

    # Instructs the vehicle to follow lists of coordinates
    def navigation_callback(self, navigation):
        navigator = BasicNavigator()

        # Estimates the vehicle's current position based on provided pose
        yaw_deg = float(input("Enter initial vehicle heading: "))
        yaw_rad = math.radians(yaw_deg)
        quaternion = quaternion_from_euler(0,0,yaw_rad)

        estimate_pose = PoseStamped()
        estimate_pose.header.frame_id = 'map'
        estimate_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose_x = float(input("Enter initial x position: "))
        initial_pose_y = float(input("Enter initial y position: "))
        estimate_pose.pose.position.x = initial_pose_x
        estimate_pose.pose.position.y = initial_pose_y
        estimate_pose.pose.orientation.x = quaternion[0]
        estimate_pose.pose.orientation.y = quaternion[1]
        estimate_pose.pose.orientation.z = quaternion[2]
        estimate_pose.pose.orientation.w = quaternion[3]
        navigator.setInitialPose(estimate_pose)

        goal_poses = []
        navigation_data = json.loads(navigation.data)
        list_of_waypoints = navigation_data['centers']
        previous_waypoint = [initial_pose_x, initial_pose_y]

        # Converts each waypoint into a navigation instruction and adds it to a list
        for waypoint in list_of_waypoints:
            current_vehicle_angle_orientation = math.atan2(waypoint[1] - previous_waypoint[1], waypoint[0] - previous_waypoint[0])
            quaternion = quaternion_from_euler(0, 0, current_vehicle_angle_orientation)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]
            goal_poses.append(goal_pose)
            previous_waypoint = waypoint

        # Executes navigation instructions in the order of the waypoints were given
        navigator.followWaypoints(goal_poses)

        # Prints status of the vehicle's progress
        while True: 

            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                current_time = time.time()
                if self.current_waypoint_destination != feedback.current_waypoint:
                    self.start_time = time.time()
                    self.current_waypoint_destination = feedback.current_waypoint
                    print(f"Currently going to waypoint {self.current_waypoint_destination}")
#                if current_time - self.start_time >= 12.0:
#                    print(f"taking too long. Moving on to the next wapoint...")
#                    break
#            else:
#                break
            
#            goal_poses = goal_poses[self.current_waypoint_destination + 1:]
#            navigator.followWaypoints(goal_poses)

            

        # Prints status of the vehicle at the end of the program
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Goal succeeded")
        elif result == TaskResult.CANCELED:
            print("Goal was cancelled")
        elif result == TaskResult.FAILED:
            print(f"Goal failed. {navigator.error()}")

def main(args=None):
    rclpy.init(args=args)
    node = ScanMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
