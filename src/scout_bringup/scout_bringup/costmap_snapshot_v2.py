import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from copy import deepcopy


class CostmapSnapshotV2(Node):

    def __init__(self):
        super().__init__('costmap_snapshot_v2')
        self.qos = QoSProfile(depth=10)
        self.qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            self.qos
        )

        self.publisher = self.create_publisher(
            OccupancyGrid, # modified to OccupancyGrid
            '/snapshot_before',
            self.qos
        )

        # JUST ADDED ___________________________________________

        # Listen for patrol completion signal
        self.patrol_complete_subscription = self.create_subscription(
            Empty,
            '/patrol_complete',
            self.patrol_complete_callback,
            10
        )

        # _______________________________________________________

        self.timer = self.create_timer(1.0, self.publish_snapshot)
        self.snapshot_before = None
        self.captured = False

        # ADDED
        self.snapshot_after = None
        self.captured_after = False

    def costmap_callback(self, msg):
        if not self.captured:
            self.get_logger().info('Taking a snapshot of the current costmap...')
            self.snapshot_before = deepcopy(msg) # modified self.snapshot = list(msg.data)
            self.captured = True
            self.get_logger().info('Snapshot taken successfully.')

        # ADDED
        elif self.captured_after and self.snapshot_after is None:
            self.get_logger().info('Taking a snapshot of the AFTER costmap...')
            self.snapshot_after = deepcopy(msg)
            self.captured_after = False
            self.get_logger().info('AFTER Snapshot taken successfully.')

    def patrol_complete_callback(self, msg):
        if self.snapshot_before is not None and self.snapshot_after is None:
            self.get_logger().info('Patrol completion signal received.')
            self.captured_after = True

        # MOVED
        self.snapshot_after_publisher = self.create_publisher(
            OccupancyGrid,
            '/snapshot_after',
            self.qos
        )

    def publish_snapshot(self):
        if self.snapshot_before is not None:
            self.publisher.publish(self.snapshot_before)

        # ADDED
        if self.snapshot_after is not None:
            self.snapshot_after_publisher.publish(self.snapshot_after)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapSnapshotV2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()