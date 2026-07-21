import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import math
import json

class MapReader(Node):

    def __init__(self):
        super().__init__('map_reader')

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.received = False
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_info,
            qos
        )

        self.graph_publisher = self.create_publisher(
            String,
            '/map_graph',
            qos
        )

        self.msg_out = None
        self.timer = self.create_timer(
            1.0,
            self.publish_graph
        )

        self.get_logger().info("Waiting for /map...")

    def map_info(self, msg):
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
        left_col = math.ceil(abs(x_min))
        right_col = math.ceil(width*resolution - abs(x_min))
        high_row = math.ceil(height*resolution - abs(y_min))
        low_row = math.ceil(abs(y_min))
        cols = left_col + right_col
        rows = high_row + low_row

        #generate coarse cell bounds matrix
        cell_bounds = []
        
        for r in range(rows):
            row = []
            for c in range(cols):
                cx = c - left_col
                cy = r - low_row
                x_left = max(cx, x_min)
                x_right = min(cx + 1, x_max)
                y_bottom = max(cy, y_min)
                y_top = min(cy + 1, y_max)
                row.append((x_left, x_right, y_bottom, y_top))
            cell_bounds.append(row)

        #generate coarse cell occupancy matrix

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

                neighbors = []

                for dr, dc in directions:
                    nr = row + dr
                    nc = col + dc
                    if 0 <= nr <= (rows-1) and 0 <= nc <= (cols-1):
                        neighbors.append((nr, nc))

                graph[(row, col)] = {
                    "occupancy": occupancy,
                    "visited": False,
                    "x_center": x_center,
                    "y_center": y_center,
                    "neighbors": neighbors
                }

        self.get_logger().info("====== Map Info ======")
        self.get_logger().info(f"Width: {width} or {cols} grids")
        self.get_logger().info(f"Height: {height} or {rows} grids")
        self.get_logger().info(f"X Minimum: {x_min}")
        self.get_logger().info(f"X Maximum: {x_max}")
        self.get_logger().info(f"Y Minimum: {y_min}")
        self.get_logger().info(f"Y Maximum: {y_max}")

        graph_json = {}
        for vertex_id, info in graph.items():
            key = f"{vertex_id[0]},{vertex_id[1]}"
            graph_json[key] = {
                "occupancy": info["occupancy"],
                "visited": info["visited"],
                "x_center": info["x_center"],
                "y_center": info["y_center"],
                "neighbors": info["neighbors"]
            }
        
        self.msg_out = String()
        self.msg_out.data = json.dumps(graph_json)
        self.received = True

    def publish_graph(self):
        if self.msg_out is not None:
            self.graph_publisher.publish(self.msg_out)
            #self.get_logger().info(f"Published /map_graph")

def main(args=None):
    rclpy.init(args=args)
    node = MapReader()
    #while rclpy.ok() and not node.received:
    #    rclpy.spin_once(node)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

