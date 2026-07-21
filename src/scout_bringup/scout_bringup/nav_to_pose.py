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

class NavToPose(Node):
    def __init__(self):
        super().__init__("nav_to_pose")
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # only need to check initial position once
        self.initial_position_received=False
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.pending_graph_msg_received = False
        self.pending_graph_msg = None
        self.navigator = BasicNavigator()

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

            if (self.pending_graph_msg_received):
                self.graph_process(self.pending_graph_msg)

    def graph_callback(self, msg):
        if self.pending_graph_msg_received:
            return
        if (self.initial_position_received == True):
            self.graph_process(msg)
        else:
            self.pending_graph_msg = msg
            self.pending_graph_msg_received = True

    def graph_process(self, msg):
         # Navigate logic
        self.get_logger().info("Processing map_graph...")
        

        #Get initial pose
        initial_pose = self.create_pose(
            self.navigator,
            self.initial_x,
            self.initial_y,
            self.initial_yaw
        )

        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()

        self.map_dict = json.loads(msg.data)
        #print(self.map_dict["3,6"]["x_center"])
        #print(self.map_dict["3,6"]["y_center"])
        row = str(input("Goal x: "))
        col = str(input("Goal y: "))
        key = row + ',' + col

        goal_x = self.map_dict[key]["x_center"]
        goal_y = self.map_dict[key]["y_center"]

        goal_yaw = float(input("Goal yaw in degrees: "))
        
        goal_pose = self.create_pose(
            self.navigator,
            goal_x,
            goal_y,
            goal_yaw
        )

        self.get_logger().info("Sending robot to goal...")
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info(f"Goal failed!")
        else:
            self.get_logger().info("Goal has an invalid return status!")

        self.initial_position_received = False
        self.pending_graph_msg_received = False

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
    node = NavToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
