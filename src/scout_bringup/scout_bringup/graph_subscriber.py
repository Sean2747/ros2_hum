import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import json


class GraphSubscriber(Node):

    def __init__(self):
        super().__init__('graph_subscriber')

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            String,
            '/map_graph',
            self.graph_callback,
            qos
        )

        self.get_logger().info("Waiting for /map_graph ...")

    def graph_callback(self, msg):
        try:
            graph = json.loads(msg.data)

            self.get_logger().info("====== FULL GRAPH ======")

            for vertex_id, info in graph.items():
                occupancy = info["occupancy"]
                visited = info["visited"]
                x_center = info["x_center"]
                y_center = info["y_center"]
                neighbors = info["neighbors"]

                self.get_logger().info(
                    f"ID: ({vertex_id}) | "
                    f"occupancy: {occupancy} | "
                    #f"visited: {visited} | "
                    f"center: ({x_center}, {y_center}) | "
                    #f"neighbors: {neighbors}"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to parse graph JSON: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GraphSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
