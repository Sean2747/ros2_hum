import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math

class CostmapComparisonV2(Node):
    def __init__(self):
        super().__init__('costmap_comparison_v2')

        self.cell_size = 0.25  # Define the size of each coarse cell
        self.graph_before = None
        self.graph_after = None
        self.snapshot_before_received = False
        self.snapshot_after_received = False
        self.x_min = None
        self.y_min = None

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        #Subscriber to snapshot_before
        self.snapshot_before_subscriber = self.create_subscription(
            OccupancyGrid,
            '/snapshot_before',
            self.snapshot_before_callback,
            qos
        )

        #Subscriber to snapshot_after
        self.snapshot_after_subscriber = self.create_subscription(
            OccupancyGrid,
            '/snapshot_after',
            self.snapshot_after_callback,
            qos
        )
    
    def snapshot_before_callback(self, msg):
        if self.snapshot_before_received:
            return
        self.graph_before = self.grid_to_dict(msg)
        self.snapshot_before_received = True

        if self.snapshot_after_received is not None:    #in case if the receiving order is incorrect
            self.compare_costmaps()

    def snapshot_after_callback(self, msg):
        if self.snapshot_after_received:
            return
        self.graph_after = self.grid_to_dict(msg)
        self.snapshot_after_received = True

        if self.snapshot_before_received is not None:
            self.compare_costmaps()

    def grid_to_dict(self, msg):
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

                #only accept 0, 100, -1 as possible occupancy values
                if data[index-1] > 0 and data[index-1] < 100:
                    occupancy = 0
                else:
                    occupancy = data[index-1]

                occupancy_row.append((occupancy, math.floor(x_mid*100)/100, math.floor(y_mid*100)/100))

            cell_occupancy.append(occupancy_row)

        #generate map_graph
        graph = {}

        #directions = [
        #   (0,-1),(1,0),(0,1),(-1,0)           #4 directions --> at most 4 neighbors
        #]

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

        #self.get_logger().info(f"{graph}")     #uncomment to print the entire dictionary

        self.get_logger().info("====== Costmap Info ======")
        self.get_logger().info(f"Width: {width} or {cols} grids")
        self.get_logger().info(f"Height: {height} or {rows} grids")
        self.get_logger().info(f"X Minimum: {x_min}")
        self.get_logger().info(f"X Maximum: {x_max}")
        self.get_logger().info(f"Y Minimum: {y_min}")
        self.get_logger().info(f"Y Maximum: {y_max}")

        self.x_min = x_min
        self.y_min = y_min

        return graph

    def compare_costmaps(self):
        self.get_logger().info("Comparison begins")
        if self.graph_before is None or self.graph_after is None:
            self.get_logger().info("Cannot compare yet.")
            return

        #Store all fine cells that changed from free (0) to occupied (100)
        changed_cells = set()

        for index in self.graph_after:   #both graphs should have the same set of indices
            before_occupancy = self.graph_before[index]["occupancy"]
            after_occupancy = self.graph_after[index]["occupancy"]
            if before_occupancy == 0 and after_occupancy == 100:
                changed_cells.add(index)

        self.get_logger().info(f"Free-to-occupied cells before filtering: {changed_cells}")

        #Keep only changed cells with at least one changed neighbor
        directions = [
            (-1,-1),
            (-1, 0),
            (-1, 1),

            ( 0,-1),
            ( 0, 1),

            ( 1,-1),
            ( 1, 0),
            ( 1, 1)
        ]

        for row, col in list(changed_cells):
            for row_change, col_change in directions:
                neighbor = (
                row + row_change,
                col + col_change
                )
                if neighbor in changed_cells:
                    break
            else:
                changed_cells.discard((row, col))

        self.get_logger().info(f"Cells after neighbor filtering: {changed_cells}")

        #Convert indices: fine --> coarse
        coarse_cells = self.convert_to_coarse_indices(changed_cells)
        self.get_logger().info(f"Number of changes detected: {len(coarse_cells)}")
        self.get_logger().info(f"Cells after coarse conversion: {coarse_cells}")
  
    def convert_to_coarse_indices(self, changed_cells):
        coarse_cells = set()
        coarse_cell_size = 1.0
        coarse_left_col = math.ceil(abs(self.x_min) / coarse_cell_size)
        coarse_low_row = math.ceil(abs(self.y_min) / coarse_cell_size)

        for row, col in list(changed_cells):
            x_center = self.graph_after[(row, col)]["x_center"]
            y_center = self.graph_after[(row, col)]["y_center"]
            coarse_col = math.floor(x_center / coarse_cell_size) + coarse_left_col
            coarse_row = math.floor(y_center / coarse_cell_size)  + coarse_low_row
            coarse_cells.add((coarse_row, coarse_col))
            changed_cells.discard((row, col))
        return coarse_cells

def main(args=None):
    rclpy.init(args=args)
    node = CostmapComparisonV2()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

