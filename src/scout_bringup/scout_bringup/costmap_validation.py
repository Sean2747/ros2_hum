import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
# no longer need this: from std_msgs.msg import Int8MultiArray
import math
import numpy as np

class CostmapValidation(Node):
    def __init__(self):
        super().__init__("costmap_validation")

        self.raw_costmap_snapshot = None
        self.snapshot_msg = None # ADDED: Store complete OccupancyGrid
        self.costmap_matrix_comparison = None
        self.costmap_snapshot = None
        self.costmap_differences = list()     
        self.comparison_count = 0 

        self.width = None
        self.resolution = None
        self.x_min = None
        self.y_min = None
        self.left_col = None
        self.low_row = None
         

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.snapshot_costmap_subscription = self.create_subscription(
            OccupancyGrid,
            '/costmap_snapshot_v2',  # separate occupancygrid snapshot topic
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
            self.snapshot_msg = snapshot # now preserving snapshot metadata
            self.raw_costmap_snapshot = np.array(snapshot.data)
            self.get_logger().info("Saved snapshot")

    # NEW METHOD
    def validate_geometry(self, snapshot, live):
        snapshot_info = snapshot.info
        live_info = live.info
        matches = True

        # Compare width
        if snapshot_info.width != live_info.width:
            self.get_logger().warning(f"Width mismatch: {snapshot_info.width} vs {live_info.width}")
            matches = False

        # Compare height
        if snapshot_info.height != live_info.height:
            self.get_logger().warning(f"Height mismatch: {snapshot_info.height} vs {live_info.height}")
            matches = False

        # Compare resolution
        if not math.isclose(
            snapshot_info.resolution,
            live_info.resolution,
            rel_tol=0.0,
            abs_tol=1e-6
        ):
            self.get_logger().warning(f"Resolution mismatch: {snapshot_info.resolution} vs {live_info.resolution}")
            matches = False

        # Comapre origin position
        for axis in ("x", "y", "z"):
            snapshot_value = getattr(snapshot_info.origin.position, axis)
            live_value = getattr(live_info.origin.position, axis)

            if not math.isclose(
                snapshot_value,
                live_value,
                rel_tol=0.0,
                abs_tol=1e-6
            ):
                self.get_logger().warning(
                    f"Origin {axis} mismatch: {snapshot_value} vs {live_value}"
                )
                matches = False

        # Compare origin orientation
        for axis in ("x", "y", "z", "w"):
            snapshot_value = getattr(snapshot_info.origin.orientation, axis)
            live_value = getattr(live_info.origin.orientation, axis)

            if not math.isclose(
                snapshot_value,
                live_value,
                rel_tol=0.0,
                abs_tol=1e-6
            ):
                self.get_logger().warning(
                    f"Orientation {axis} mismatch: {snapshot_value} vs {live_value}"
                )
                matches = False

        # Compare coordinate frames
        if snapshot.header.frame_id != live.header.frame_id:
            self.get_logger().warning(
                f"Frame mismatch: {snapshot.header.frame_id} vs {live.header.frame_id}"
            )
            matches = False

        # Check that occupancy array lengths match map dimensions
        if len(snapshot.data) != snapshot_info.width * snapshot_info.height:
            self.get_logger().warning("Invalid snapshot data length")
            matches = False

        if len(live.data) != live_info.width * live_info.height:
            self.get_logger().warning("Invalid live costmap data length")
            matches = False

        if matches:
            self.get_logger().info("Costmap geometry matches!")

        return matches


    def compare_raw_costmaps(self, live_costmap):
        if self.width is None:
            self.width = live_costmap.info.width
            self.resolution = live_costmap.info.resolution
            self.x_min = live_costmap.info.origin.position.x
            self.y_min = live_costmap.info.origin.position.y
            self.left_col = math.ceil(abs(self.x_min))
            self.low_row = math.ceil(abs(self.y_min))
        
        # continue comparing whenever a snapshot is available
        if self.raw_costmap_snapshot is not None:    
            # ADDED: Validate costmap geometry before comparing occupancy values
            if not self.validate_geometry(self.snapshot_msg, live_costmap):
                self.get_logger().warning("Costmap comparison skipped due to geometry mismatch.")
                return
            
            # ADDED: Count every comparison with matching geometry
            self.comparison_count += 1

            self.get_logger().info(
                f"Running costmap comparison #{self.comparison_count}"
            )

            self.get_logger().info("Costmap comparison method is running")
            raw_live_costmap = np.array(live_costmap.data)

            # Prevent comparing incompatible array sizes
            if len(raw_live_costmap) != len(self.raw_costmap_snapshot):
                self.get_logger().warning("Snapshot and live costmap have different array sizes.")
                return

            diff_indices = np.where(
                (raw_live_costmap == 100) & 
                (self.raw_costmap_snapshot == 0)
            )[0]     

            # Diagnostic for obstacles that disappeared
            removed_indices = np.where(
                (self.raw_costmap_snapshot == 100) &
                (raw_live_costmap == 0)
            )[0]

            self.get_logger().info(
                f"Occupied-to-free pixels: {len(removed_indices)}"
            )

            self.costmap_differences = diff_indices.tolist()

            # log the total number of changed pixels before filtering
            self.get_logger().info(f"Raw changed pixels: {len(self.costmap_differences)}")

            self.get_logger().info("Comparison finished")
            #self.get_logger().info(f"{self.costmap_differences}")

            diff = {}

            for index in self.costmap_differences:
                fine_x = index % self.width
                fine_y = index // self.width

                world_x = self.x_min + fine_x * self.resolution
                world_y = self.y_min + fine_y * self.resolution

                col = math.floor(world_x) + self.left_col
                row = math.floor(world_y) + self.low_row

                key = (row, col)
                if key not in diff:
                    diff[key] = 1
                else:
                    diff[key] += 1
                    
            filtered_diff = {k: v for k, v in diff.items() if v >= 15}      #only copy ones with certain# of different pixels
            
            # Show all coarse cells containing changed pixels,
            # including those below 15 pixel threshold
            if len(diff) > 0:
                self.get_logger().info(f"Changed pixels per coarse cell: {diff}")

            # Log how many coarse cells passed the threshold
            self.get_logger().info(f"Cells passing threshold: {len(filtered_diff)}")

            if len(filtered_diff) == 0:
                self.get_logger().info("No differences deteced")
            else:
                self.get_logger().info(f"Differences detected in: {filtered_diff}") 



def main(args=None):
    rclpy.init(args=args)
    node = CostmapValidation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
