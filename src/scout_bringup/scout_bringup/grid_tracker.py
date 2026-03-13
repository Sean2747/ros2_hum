import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class GridTracker(Node):
    def __init__(self):
        super().__init__("grid_tracker")
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.output_map_data, qos)

    def output_map_data(self, data):
        self.get_logger().info("Hello map")
        x = data.info.width
        y = data.info.height
        self.get_logger().info(f"Cell: ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    node = GridTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()