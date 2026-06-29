import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import math

class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.scout_pose_msg = Float64MultiArray()
        self.create_timer(0.5, self.publish_pose)

        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.output_amcl_data, qos)

        self.scout_pose_publisher = self.create_publisher(
            Float64MultiArray,
            '/scout_pose',
            10
        )

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

        self.get_logger().info(f"({current_x}, {current_y}) | degrees: {yaw_deg} | radians: {yaw_rad}")
        self.scout_pose_msg.data = [current_x,current_y,yaw_deg]

    def publish_pose(self):
            self.scout_pose_publisher.publish(self.scout_pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
