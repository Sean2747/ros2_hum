import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import math
import os
import yaml

class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.scout_pose_msg = Float64MultiArray()
        self.create_timer(0.5, self.publish_pose)

        self.amcl_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.output_amcl_data, 
            qos
        )

        self.scout_pose_publisher = self.create_publisher(
            Float64MultiArray,
            '/scout_pose',
            10
        )

        self.declare_parameter("map_path", "")
        map_path = self.get_parameter("map_path").value

        with open(map_path, "r") as file:
            map_data = yaml.safe_load(file)

        origin = map_data["origin"]
        self.origin_x = origin[0]
        self.origin_y = origin[1]

        #self.get_logger().info(f"{self.origin_x}, {self.origin_y}")



    def output_amcl_data(self, data):
        self.get_logger().info("Hello amcl")
        current_x = data.pose.pose.position.x
        current_y = data.pose.pose.position.y

        quaternion_values = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]

        _, _, yaw_rad = euler_from_quaternion(quaternion_values)
        yaw_deg = math.degrees(yaw_rad)

        row = math.ceil(current_y - self.origin_y) - 1
        col = math.ceil(current_x - self.origin_x) - 1

        self.get_logger().info(f"({current_x}, {current_y}) | degrees: {yaw_deg} | radians: {yaw_rad} | grid index: {row}, {col}")
        
        row = float(row)
        col = float(col) 
        
        self.scout_pose_msg.data = [current_x,current_y,yaw_deg,row,col]

    def publish_pose(self):
            self.scout_pose_publisher.publish(self.scout_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
