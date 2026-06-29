import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
import math

class NavToPose(Node):
    def __init__(self):
        super().__init__("nav_to_pose")
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # only need to check initial position once
        self.message_received=False
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None

        self.pose_sub=self.create_subscription(
            Float64MultiArray,
            "/scout_pose",
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        if (self.message_received == True):
            return
        else:
            self.get_logger().info("Subscriber started")
            self.initial_x = msg.data[0]
            self.initial_y = msg.data[1]
            self.initial_yaw = msg.data[2]
            self.message_received = True

            # Navigate logic
            navigator = BasicNavigator()

            initial_pose = self.create_pose(
                navigator,
                self.initial_x,
                self.initial_y,
                self.initial_yaw
            )

            navigator.setInitialPose(initial_pose)

            self.get_logger().info("Waiting for Nav2 to become active...")
            navigator.waitUntilNav2Active()

            print("\nEnter goal pose below")
            goal_x = float(input("Goal x: "))
            goal_y = float(input("Goal y: "))
            goal_yaw = float(input("Goal yaw in degrees: "))

            goal_pose = self.create_pose(
                navigator,
                goal_x,
                goal_y,
                goal_yaw
            )

            self.get_logger().info("Sending robot to goal...")
            navigator.goToPose(goal_pose)

            while not navigator.isTaskComplete():
                pass

            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Goal succeeded!")
            elif result == TaskResult.CANCELED:
                self.get_logger().info("Goal was canceled!")
            elif result == TaskResult.FAILED:
                error_code, error_msg = navigator.getTaskError()
                self.get_logger().info(f"Goal failed! {error_code}: {error_msg}")
            else:
                self.get_logger().info("Goal has an invalid return status!")

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

    # def navigate(self):
    #     if (self.message_received):
    #         navigator = BasicNavigator()
    #
    #         # print("Enter initial vehicle pose below")
    #         # initial_x = float(input("Initial x: "))
    #         # initial_y = float(input("Initial y: "))
    #         # initial_yaw = float(input("Initial yaw in degrees: "))
    #
    #         initial_pose = self.create_pose(
    #             navigator,
    #             self.initial_x,
    #             self.initial_y,
    #             self.initial_yaw
    #         )
    #
    #         navigator.setInitialPose(initial_pose)
    #
    #         self.get_logger().info("Waiting for Nav2 to become active...")
    #         navigator.waitUntilNav2Active()
    #
    #         print("\nEnter goal pose below")
    #         goal_x = float(input("Goal x: "))
    #         goal_y = float(input("Goal y: "))
    #         goal_yaw = float(input("Goal yaw in degrees: "))
    #
    #         goal_pose = self.create_pose(
    #             navigator,
    #             goal_x,
    #             goal_y,
    #             goal_yaw
    #         )
    #
    #         self.get_logger().info("Sending robot to goal...")
    #         navigator.goToPose(goal_pose)
    #
    #         while not navigator.isTaskComplete():
    #             pass
    #
    #         result = navigator.getResult()
    #
    #         if result == TaskResult.SUCCEEDED:
    #             self.get_logger().info("Goal succeeded!")
    #         elif result == TaskResult.CANCELED:
    #             self.get_logger().info("Goal was canceled!")
    #         elif result == TaskResult.FAILED:
    #             error_code, error_msg = navigator.getTaskError()
    #             self.get_logger().info(f"Goal failed! {error_code}: {error_msg}")
    #         else:
    #             self.get_logger().info("Goal has an invalid return status!")

def main(args=None):
    rclpy.init(args=args)
    node = NavToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
