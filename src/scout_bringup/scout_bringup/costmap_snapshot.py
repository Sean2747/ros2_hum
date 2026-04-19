import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Int8MultiArray
from nav_msgs.msg import OccupancyGrid


class CostmapSnapshot(Node):

    def __init__(self):
        super().__init__('costmap_snapshot')
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            qos
        )

        self.publisher = self.create_publisher(
            Int8MultiArray,
            '/costmap_snapshot',
            qos
        )

        self.timer = self.create_timer(1.0, self.publish_snapshot)
        self.snapshot = None
        self.captured = False

    def costmap_callback(self, msg):
        if not self.captured:
            self.get_logger().info('Taking a snapshot of the current costmap...')
            self.snapshot = list(msg.data)
            self.captured = True
            self.get_logger().info('Snapshot taken successfully.')
           #self.get_logger().info(f"list size: {len(self.snapshot)}")

    def publish_snapshot(self):
        if self.snapshot is not None:
            msg_out = Int8MultiArray()
            msg_out.data = self.snapshot
            self.publisher.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapSnapshot()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

