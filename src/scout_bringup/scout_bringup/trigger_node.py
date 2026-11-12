import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class TriggerNode(Node):
    def __init__(self):
        super().__init__("trigger_node")
        self.publisher=self.create_publisher(
            Empty,
            "patrol_complete",
            10
        )
        self.timer=self.create_timer(1.0, self.publish_signal)
        
    def publish_signal(self):
        msg=Empty()
        self.publisher.publish(msg)
        #self.get_logger().info(f"signal sent")
    
def main(args=None):
        rclpy.init(args=args)
        node = TriggerNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__=="__main__":
    main()
