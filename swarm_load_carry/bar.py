import asyncio, rclpy

from rclpy.node import Node
from std_msgs.msg import Bool

import rclpy.qos as qos
from rclpy.qos import QoSProfile


class Bar(Node):
    def __init__(self):
        super().__init__('foo')

        # Setup test publisher
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.test_pub = self.create_publisher(Bool, '/foo_sub', qos_profile)

    def send_msg(self):
        msg = Bool()
        msg.data = True

        self.test_pub.publish(msg)

    
def main():
    rclpy.init()

    # Create node
    bar = Bar()

    # Send messages
    while True:
        input('Press enter when you want to send msg')
        bar.send_msg()

if __name__ == '__main__':
    asyncio.run(main())