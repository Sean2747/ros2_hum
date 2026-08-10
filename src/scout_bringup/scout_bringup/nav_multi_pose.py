import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
import math
import json

class NavMultiPose(Node):
    def __init__(self):
        super().__init__("nav_multi_pose")
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # only need to check initial position once
        self.initial_position_received=False
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.navigator = BasicNavigator()
        self.total_locs = 3
        #self.locs = [[2, 2, 0],[1, 0, 1], [0, 1, 90]]       #hardcoded goal poses
        self.locs = []
        self.map_graph_received = False

        self.map_dict = None

        self.pose_sub=self.create_subscription(
            Float64MultiArray,
            "/scout_pose",
            self.pose_callback,
            10
        )

        self.graph_sub=self.create_subscription(
            String,
            "/map_graph",
            self.graph_callback,
            qos,
        )

    def pose_callback(self, msg):
        if (self.initial_position_received):
            return
        else:
            self.get_logger().info("Subscriber started")
            self.initial_x = msg.data[0]
            self.initial_y = msg.data[1]
            self.initial_yaw = msg.data[2]
            self.initial_position_received = True
            self.get_logger().info("Initial pose set")


    def graph_callback(self, msg):
        if (self.map_graph_received or not self.initial_position_received):
            return
        self.map_graph_received = True

        for i in range(3):
            x = int(input(f"Goal {i + 1} x: "))
            y = int(input(f"Goal{i + 1} y: "))
            z = float(input(f"Goal {i + 1} yaw in degrees: "))
            goal = (x, y, z)
            self.locs.append(goal)
     
        for i in range(self.total_locs):
            print(i)
            initial_pose = self.create_pose(
                self.navigator,
                self.initial_x,
                self.initial_y,
                self.initial_yaw
            )

            self.navigator.setInitialPose(initial_pose)

            self.map_dict = json.loads(msg.data)

            row = str(self.locs[i][0])
            col = str(self.locs[i][1])
            goal_yaw = float(self.locs[i][2])

            key = row + ',' + col

            goal_x = self.map_dict[key]["x_center"]
            goal_y = self.map_dict[key]["y_center"]

            goal_pose = self.create_pose(
                self.navigator,
                goal_x,
                goal_y,
                goal_yaw
            )

            self.get_logger().info(f"initial_x: {self.initial_x} initial_y: {self.initial_y}, initial_yaw: {self.initial_yaw}")
            self.get_logger().info(f"goal_x: {goal_x} goal_y: {goal_y}, goal_yaw: {goal_yaw}")

            self.get_logger().info("Sending robot to goal...")
            #self.navigator.goToPose(goal_pose)                 #uncomment this to allow real navigation

            self.initial_x = goal_x
            self.initial_y = goal_y
            self.initial_yaw = goal_yaw

    def create_pose(self, navigator, x, y, yaw_deg):
        yaw_rad = math.radians(yaw_deg)
        quaternion = quaternion_from_euler(0, 0, yaw_rad)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        return pose

def main(args=None):
    rclpy.init(args=args)
    node = NavMultiPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
