
import rclpy
from nav_msgs.msg import OccupancyGrid
from costmap_validation import CostmapValidation


def create_test_map(origin_x):
    msg = OccupancyGrid()

    msg.header.frame_id = "map"

    msg.info.width = 200
    msg.info.height = 200
    msg.info.resolution = 0.05

    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = -5.0
    msg.info.origin.orientation.w = 1.0

    msg.data = [0] * (200 * 200)

    return msg


def main():
    rclpy.init()

    node = CostmapValidation()

    snapshot = create_test_map(-5.0)

    # Test 1: Identical geometry
    live = create_test_map(-5.0)

    print("\nTEST 1: Matching geometry")
    result = node.validate_geometry(snapshot, live)
    print("Result:", result)

    # Test 2: Different origins
    live = create_test_map(-4.5)

    print("\nTEST 2: Origin mismatch")
    result = node.validate_geometry(snapshot, live)
    print("Result:", result)


    # TEST 3: Verify comparison is skipped when geometry differs
    print("\nTEST 3: Comparison blocked by geometry mismatch")

    # Create a snapshot with origin X = -5.0
    snapshot = create_test_map(-5.0)

    # Create a live costmap with origin X = -4.5
    live = create_test_map(-4.5)

    # Set one cell to occupied in the live costmap
    live.data[100] = 100

    # Simulate receiving the snapshot
    node.store_raw_snapshot(snapshot)

    # Simulate receiving the live costmap
    node.compare_raw_costmaps(live)

    # Check whether comparison was skipped
    if len(node.costmap_differences) == 0:
        print("PASS: Comparison was skipped")
    else:
        print("FAIL: Comparison was performed")
    node.destroy_node()

    # TEST 4: Compare occupancy values when geometry matches
    print("\nTEST 4: Matching geometry allows comparison")

    # Create a new node so previous test results don't interfere
    test_node = CostmapValidation()

    snapshot = create_test_map(-5.0)
    live = create_test_map(-5.0)

    # Simulate a new obstacle at index 100
    live.data[100] = 100

    # Save snapshot and compare against live costmap
    test_node.store_raw_snapshot(snapshot)
    test_node.compare_raw_costmaps(live)

    # Check whether the changed cell was detected
    if 100 in test_node.costmap_differences:
        print("PASS: Occupancy change detected at index 100")
    else:
        print("FAIL: Occupancy change was not detected")

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
