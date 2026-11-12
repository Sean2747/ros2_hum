import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math
import json

class CostmapComparisonV2(Node):
    def __init__(self):
        super().__init__('costmap_comparison_v2')

        self.received = False
        self.cell_size = 0.5  # Define the size of each coarse cell

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Currently working on only subscriber to snapshot_before
        self.snapshot_before_subscriber = self.create_subscription(
            OccupancyGrid,
            '/snapshot_before',
            self.map_before_callback,
            qos
        )

        # Implement logic for handling both snapshots simultaneously here
    
    def map_before_callback(self, msg):
        if self.received:
            return
        
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height

        x_min = msg.info.origin.position.x
        y_min = msg.info.origin.position.y
        x_max = math.floor((x_min + width*resolution)*100)/100
        y_max = math.floor((y_min + height*resolution)*100)/100

        data = msg.data
        left_col = math.ceil(abs(x_min) / self.cell_size)
        right_col = math.ceil((width*resolution - abs(x_min)) / self.cell_size)
        high_row = math.ceil((height*resolution - abs(y_min)) / self.cell_size)
        low_row = math.ceil(abs(y_min) / self.cell_size)
        cols = left_col + right_col
        rows = high_row + low_row

        # generate coarse cell bounds matrix
        cell_bounds = []
        
        for r in range(rows):
            row = []
            for c in range(cols):
                cx = (c - left_col) * self.cell_size
                cy = (r - low_row) * self.cell_size
                x_left = max(cx, x_min)
                x_right = min(cx + self.cell_size, x_max)
                y_bottom = max(cy, y_min)
                y_top = min(cy + self.cell_size, y_max)
                row.append((x_left, x_right, y_bottom, y_top))
            cell_bounds.append(row)

        # generate coarse cell occupancy matrix
        cell_occupancy = []

        for row in cell_bounds:
            occupancy_row = []
            for cell in row:
                x_left = cell[0]
                x_right = cell[1]
                y_bottom = cell[2]
                y_top = cell[3]

                x_mid = (x_left + x_right) / 2
                y_mid = (y_bottom + y_top) / 2

                fine_x = math.floor((x_mid - x_min) / resolution)
                fine_y = math.floor((y_mid - y_min) / resolution)
                index = fine_y * width + fine_x

                occupancy_row.append((data[index-1], math.floor(x_mid*100)/100, math.floor(y_mid*100)/100))
                #occupancy_row.append((fine_x, fine_y, data[index-1]))
            cell_occupancy.append(occupancy_row)

        #generate map_graph
        graph = {}

        directions = [
            # (1,-1),(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1)      #8 directions --> at most 8 neighbors
            (0,-1),(1,0),(0,1),(-1,0)                                   #4 directions --> at most 4 neighbors
        ]

        for row in range(rows):
            for col in range(cols):
                occupancy, x_center, y_center = cell_occupancy[row][col]

                #neighbors = []

                #for dr, dc in directions:
                #    nr = row + dr
                #    nc = col + dc
                #    if 0 <= nr <= (rows-1) and 0 <= nc <= (cols-1):
                #        neighbors.append((nr, nc))

                graph[(row, col)] = {
                    "occupancy": occupancy,
                #    "visited": False,
                    "x_center": x_center,
                    "y_center": y_center,
                #    "neighbors": neighbors
                }

        self.get_logger().info(f"{graph}")

        self.get_logger().info("====== Costmap Info ======")
        self.get_logger().info(f"Width: {width} or {cols} grids")
        self.get_logger().info(f"Height: {height} or {rows} grids")
        self.get_logger().info(f"X Minimum: {x_min}")
        self.get_logger().info(f"X Maximum: {x_max}")
        self.get_logger().info(f"Y Minimum: {y_min}")
        self.get_logger().info(f"Y Maximum: {y_max}")

        self.received = True

def main(args=None):
    rclpy.init(args=args)
    node = CostmapComparisonV2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

