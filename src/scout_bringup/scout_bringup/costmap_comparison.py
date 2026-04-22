import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8MultiArray
import math
import numpy as np

class CostmapComparison(Node):
    def __init__(self):
        super().__init__("costmap_comparison")

        self.raw_costmap_snapshot = None
        self.costmap_matrix_comparison = None
        self.costmap_snapshot = None
        self.costmap_differences = list()      

        self.width = None
        self.resolution = None
        self.x_min = None
        self.y_min = None
        self.left_col = None
        self.low_row = None
         

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
        if self.raw_costmap_snapshot is None:
            unique_values = set(snapshot.data)
            self.get_logger().info(f"Costmap values: {sorted(unique_values)}") 
            self.raw_costmap_snapshot = np.array(snapshot.data)
            self.get_logger().info("Saved snapshot")

    def compare_raw_costmaps(self, live_costmap):
        if self.width is None:
            self.width = live_costmap.info.width
            self.resolution = live_costmap.info.resolution
            self.x_min = live_costmap.info.origin.position.x
            self.y_min = live_costmap.info.origin.position.y
            self.left_col = math.ceil(abs(self.x_min))
            self.low_row = math.ceil(abs(self.y_min))
            
        if (self.raw_costmap_snapshot is not None) and (len(self.costmap_differences) == 0):    
            self.get_logger().info("Costmap comparison method is running")
            raw_live_costmap = np.array(live_costmap.data)                              
            #diff_indices = np.where(raw_live_costmap != self.raw_costmap_snapshot)[0]       
            diff_indices = np.where((raw_live_costmap ==100) & (self.raw_costmap_snapshot != 100))[0]     
            self.costmap_differences = diff_indices.tolist()
            self.get_logger().info("Comparison finished")
            #self.get_logger().info(f"{self.costmap_differences}")

            #coarse_cells = set()
            diff = {}

            for index in self.costmap_differences:
                fine_x = index % self.width
                fine_y = index // self.width

                world_x = self.x_min + fine_x * self.resolution
                world_y = self.y_min + fine_y * self.resolution

                col = math.floor(world_x) + self.left_col
                row = math.floor(world_y) + self.low_row

                #coarse_cells.add((row, col))
                key = (row, col)
                if key not in diff:
                    diff[key] = 1
                else:
                    diff[key] += 1

            #self.get_logger().info(f"Changed coarse cells: {list(coarse_cells)}")
            #self.get_logger().info(f"Differences: {diff}")                   
            filtered_diff = {k: v for k, v in diff.items() if v >= 10}      #only copy ones with certain# of different pixels
            self.get_logger().info(f"Differences: {filtered_diff}") 



def main(args=None):
    rclpy.init(args=args)
    node = CostmapComparison()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
