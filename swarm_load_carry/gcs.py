# Contains the GCS (ground control station) class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from swarm_load_carry_interfaces.srv import ModeChange

class GroundControlStation(Node):

    def __init__(self):
        super().__init__('gcs')

        ## PUBLISHERS
        ## SUBSCRIBERS
        ## SERVICES

        ## CLIENTS
        self.cli_mode_change = self.create_client(ModeChange,'/px4_1/mode_change')
        while not self.cli_mode_change.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for offboard ROS start service')


    ## CALLBACKS

    ## MISSION CONTROL
    # Change the offboard ROS mode of the specified drones
    def offboard_ros_mode_change(self, mode_desired):
        # Prepare request
        offboard_mode_req = ModeChange.Request()
        offboard_mode_req.mode = mode_desired 

        # Send request and wait for response
        offboard_mode_future = self.cli_mode_change.call_async(offboard_mode_req)
        rclpy.spin_until_future_complete(self, offboard_mode_future)

        return offboard_mode_future.result()


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs = GroundControlStation()

    # Send offboard start
    gcs.offboard_ros_mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_START)

    # Wait for user input
    input('Press enter when you want to end offboard control')

    # Send offboard end
    gcs.offboard_ros_mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_END)

    # Maintain node
    #rclpy.spin(gcs)

    # Destroy node
    gcs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()