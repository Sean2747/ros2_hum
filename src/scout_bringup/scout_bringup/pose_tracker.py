import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseTracker(Node):
    def __init__(self):
        self.publisher = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.output_data, 10)

    def output_data(self, data):
        current_x = data.pose.position.x
        current_y = data.pose.position.y
        yaw_deg = euler_from_quaternion(data.pose.orientation)

        self.get_logger().info(f"({current_x}, {current_y}) | angle: {yaw_deg}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()