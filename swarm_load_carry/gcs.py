# Contains the GCS (ground control station) class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

class GroundControlStation(Node):

    def __init__(self):
        super().__init__('gcs')

        # Create publishers

        # Create subscribers
        self.sub_pose_drone_actual = self.create_subscription(Pose, 'pose_actual', self.listener_callback, 10)


    def listener_callback(self, msg):
        self.get_logger().info(f'I heard position: {msg.position}')


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs = GroundControlStation()

    # Maintain node
    rclpy.spin(gcs)

    # Destroy node
    gcs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()