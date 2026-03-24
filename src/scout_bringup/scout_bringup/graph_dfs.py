import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import json
from collections import deque

class GraphDFS(Node):
    def __init__(self):
        super().__init__('graph_dfs')

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            String,
            '/map_graph',
            self.dfs_callback,
            qos
        )

        self.result_publisher = self.create_publisher(
            String,
            '/dfs_result',
            qos
        )

        self.received = False
        self.graph = {}
        self.get_logger().info('Waiting for /map_graph...')

    def dfs_callback(self, msg):
        if self.received:
            return

        self.received = True

        try:
            self.graph = self.convert_graph(json.loads(msg.data))       
        except Exception as e:
            self.get_logger().error(f'Failed to parse graph JSON: {e}')
            return

        start_vertex_id = (6,4)   # let user input the starting vertex or automate it
        vertex_ids, centers = self.dfs_graph(self.graph, start_vertex_id)

        self.get_logger().info(f'vertex_ids: {vertex_ids}')
        self.get_logger().info(f'centers: {centers}')

        msg_out = String()
        msg_out.data = json.dumps({'vertex_ids': vertex_ids, 'centers': centers})
        self.result_publisher.publish(msg_out)

    def convert_graph(self, raw_graph):     #converts json-loaded graph into a dict w/ tuple IDs
        graph = {}

        for key, info in raw_graph.items():
            row, col = map(int, key.split(','))
            vertex_id = (row, col)
            graph[vertex_id] = {
                'occupancy': info['occupancy'],
                'visited': info.get('visited', False),
                'x_center': info['x_center'],
                'y_center': info['y_center'],
                'neighbors': [tuple(n) for n in info['neighbors']]
            }
        return graph


    def dfs_graph(self, graph, start_vertex_id):

        stack = deque([start_vertex_id])
        vertex_ids = []
        centers = []
        while stack:
            current_id = stack.pop()
            if graph[current_id]["visited"]:
                continue
            graph[current_id]["visited"] = True
            if graph[current_id]["occupancy"] != 0:
                continue

            vertex_ids.append(current_id)
            centers.append((graph[current_id]["x_center"], graph[current_id]["y_center"]))

            for neighbor_id in graph[current_id]["neighbors"]:
                if not graph[neighbor_id]["visited"]:
                    stack.append(neighbor_id)

        vertex_ids.append(start_vertex_id)                                                          
        centers.append((graph[start_vertex_id]["x_center"], graph[start_vertex_id]["y_center"]))       #go back to where it begins
        vertex_ids, centers = self.filter_path(vertex_ids, centers)
        return vertex_ids, centers


    #simplify vertex_ids[] and centers[]
    def filter_path(self, vertex_ids, centers):
        if len(vertex_ids) <= 2:
            return vertex_ids[:], centers[:]

        filtered_ids = [vertex_ids[0]]
        filtered_centers = [centers[0]]

        for i in range(1, len(vertex_ids) - 1):
            prev_id = vertex_ids[i - 1]
            curr_id = vertex_ids[i]
            next_id = vertex_ids[i + 1]

            dir1 = (
                curr_id[0] - prev_id[0],
                curr_id[1] - prev_id[1]
            )
            dir2 = (
                next_id[0] - curr_id[0],
                next_id[1] - curr_id[1]
            )

            # keep turning points
            if dir1 != dir2:
                filtered_ids.append(curr_id)
                filtered_centers.append(centers[i])

        # always last point
        filtered_ids.append(vertex_ids[-1])
        filtered_centers.append(centers[-1])

        return filtered_ids, filtered_centers




def main(args=None):
    rclpy.init(args=args)
    node = GraphDFS()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()