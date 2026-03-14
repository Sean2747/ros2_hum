import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import math

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

        self.get_logger().info("Reading /map...")

    def map_info(self, msg):
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

                occupancy_row.append(data[index-1])
                #occupancy_row.append((fine_x, fine_y, data[index-1]))
            cell_occupancy.append(occupancy_row)

        self.get_logger().info("====== Map Info ======")
        self.get_logger().info(f"Width: {cols} grids")
        self.get_logger().info(f"Height: {rows} grids")
        self.get_logger().info(f"{x_max}")
        self.get_logger().info(f"{y_max}")
       #self.get_logger().info(f"{cell_bounds}")
        for row in reversed(cell_occupancy):
            self.get_logger().info(f"{row}")

        self.received = True


def main(args=None):
    rclpy.init(args=args)
    node = MapReader()
    while rclpy.ok() and not node.received:
        rclpy.spin_once(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

