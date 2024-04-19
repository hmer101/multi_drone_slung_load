import rclpy
from rclpy.node import Node

class LoggingNode(Node):
    def __init__(self):
        super().__init__('logging_node')
        self.get_logger().info('This is an info log message.')
        self.get_logger().debug('This is a debug log message.')

def main(args=None):
    rclpy.init(args=args)
    node = LoggingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()