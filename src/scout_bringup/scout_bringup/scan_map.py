import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import json
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_from_euler
import math
import time

class ScanMap(Node):
    def __init__(self):
        super().__init__("scan_map")
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.pose_sub=self.create_subscription(
            Float64MultiArray,
            "/scout_pose",
            self.pose_callback,
            10
        )

        # Retrieves and executes navigation instructions
        self.navigation_subscription = self.create_subscription(
            String,
            '/dfs_result',   # Can replace with bfs_result
            self.navigation_callback2,
            qos
        )

        self.current_waypoint_destination = -1
        self.start_time = -1
        self.current_time = -1

        self.initial_position_received = False
        self.dfs_result_received = False
        self.navigator = BasicNavigator()

    def pose_callback(self, msg):
        if (self.initial_position_received):
            return

        yaw_rad = math.radians(msg.data[2])
        quaternion = quaternion_from_euler(0,0,yaw_rad)
        estimate_pose = PoseStamped()
        estimate_pose.header.frame_id = 'map'
        estimate_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        estimate_pose.pose.position.x = msg.data[0]
        estimate_pose.pose.position.y = msg.data[1]
        estimate_pose.pose.orientation.x = quaternion[0]
        estimate_pose.pose.orientation.y = quaternion[1]
        estimate_pose.pose.orientation.z = quaternion[2]
        estimate_pose.pose.orientation.w = quaternion[3]
        self.navigator.setInitialPose(estimate_pose)

        self.initial_position_received = True
        self.get_logger().info("Initial pose set")

    # Instructs the vehicle to follow lists of coordinates
    def navigation_callback(self, navigation):
        if (self.dfs_result_received) or (not self.initial_position_received):
            return

        self.dfs_result_received = True
        
        # Estimates the vehicle's current position based on provided pose
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
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]
            goal_poses.append(goal_pose)
            previous_waypoint = waypoint

        # Executes navigation instructions in the order of the waypoints were given
        self.navigator.followWaypoints(goal_poses)

        # Prints status of the vehicle's progress
        while True: 

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
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
#            self.navigator.followWaypoints(goal_poses)

            

        # Prints status of the vehicle at the end of the program
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Goal succeeded")
        elif result == TaskResult.CANCELED:
            print("Goal was cancelled")
        elif result == TaskResult.FAILED:
            print(f"Goal failed. {self.navigator.error()}")


    def navigation_callback2(self, navigation):
        if (self.dfs_result_received) or (not self.initial_position_received):
            return
        self.dfs_result_received = True
        print("Dummy navigation callback")

def main(args=None):
    rclpy.init(args=args)
    node = ScanMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
