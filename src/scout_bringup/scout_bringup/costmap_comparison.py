import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray

class CostmapComparison(Node):
    def __init__(self):
        super().__init__("costmap_comparison")

        self.raw_costmap_snapshot = None
        self.costmap_matrix_comparison = None
        self.costmap_snapshot = None

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.snapshot_costmap_subscription = self.create_subscription(
            Int8MultiArray,
            '/costmap_snapshot',
            self.store_raw_snapshot,
            qos
        )

        self.live_costmap_subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.compare_raw_costmaps,
            qos
        )

    def store_raw_snapshot(self, snapshot):
        self.raw_costmap_snapshot = snapshot.data
        self.destroy_subscription(self.live_costmap_subscription)

    def compare_raw_costmaps(self, live_costmap):
        raw_live_costmap = live_costmap.data
        list_length = len(raw_live_costmap)
        costmap_differences = list()

        if (self.raw_costmap_snapshot != None) and (raw_live_costmap != self.raw_costmap_snapshot):
            for index in range(list_length):
                if (raw_live_costmap[index] != self.raw_costmap_snapshot[index]) and (index not in costmap_differences):
                    costmap_differences.append(index)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapComparison()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
