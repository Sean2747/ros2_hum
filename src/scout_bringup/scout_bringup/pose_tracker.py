import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

class PoseTracker(Node):
    def __init__(self):
        super().__init__('pose_tracker')
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.output_amcl_data, qos)
        #self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.output_map_data, qos)

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

        yaw_deg = euler_from_quaternion(quaternion_values)

        self.get_logger().info(f"({current_x}, {current_y}) | angle: {yaw_deg}")
        #self.get_logger().info(data)

    def output_map_data(self, data):
        print("print map")
        self.get_logger().info("Hello map")
        self.get_logger().info(data)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
