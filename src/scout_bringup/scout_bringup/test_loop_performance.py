import rclpy
from rclpy.node import Node

class TestLoopPerformance(Node):
    def __init__(self):
        super().__init__("test_loop_performance")
        self.output_loop()

    def output_loop(self):
        for i in range(0, 20000):
            self.get_logger().info(f"Iteration: {i}")

def main(args=None):
    rclpy.init(args=args)
    node = TestLoopPerformance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()