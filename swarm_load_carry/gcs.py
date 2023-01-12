# Contains the GCS (ground control station) class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from swarm_load_carry_interfaces.srv import ModeChange

NUM_DRONES=3

class GroundControlStation(Node):

    def __init__(self, drone_id=1):
        super().__init__('gcs')

        ## PUBLISHERS
        ## SUBSCRIBERS
        ## SERVICES

        ## CLIENTS
        self.cli_mode_change_1 = self.create_client(ModeChange,f'/px4_1/mode_change')
        while not self.cli_mode_change_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for offboard ROS start service 1')
        
        self.cli_mode_change_2 = self.create_client(ModeChange,f'/px4_2/mode_change')
        while not self.cli_mode_change_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for offboard ROS start service 2')

        self.cli_mode_change_3 = self.create_client(ModeChange,f'/px4_3/mode_change')
        while not self.cli_mode_change_3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for offboard ROS start service 3')


    ## CALLBACKS

    ## MISSION CONTROL
    # Change the offboard ROS mode of the specified drones
    def mode_change(self, mode_desired):
        # Prepare request
        offboard_mode_req = ModeChange.Request()
        offboard_mode_req.mode = mode_desired 

        # Send request and wait for response
        offboard_mode_future_1 = self.cli_mode_change_1.call_async(offboard_mode_req)
        offboard_mode_future_2 = self.cli_mode_change_2.call_async(offboard_mode_req)
        offboard_mode_future_3 = self.cli_mode_change_3.call_async(offboard_mode_req)

        rclpy.spin_until_future_complete(self, offboard_mode_future_1)
        rclpy.spin_until_future_complete(self, offboard_mode_future_2)
        rclpy.spin_until_future_complete(self, offboard_mode_future_3)

        return offboard_mode_future_1.result()


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs = GroundControlStation()

    # Send takeoff
    input('Press enter when you want to take off')
    gcs.mode_change(ModeChange.Request.MODE_TAKEOFF_MAV_START)

    # Send offboard start
    input('Press enter when you want to start offboard control')
    gcs.mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_START) 

    # Send offboard end
    input('Press enter when you want to end offboard control')
    gcs.mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_END)

    # Send land
    input('Press enter when you want to land at home')
    gcs.mode_change(ModeChange.Request.MODE_LAND_MAV_START)


    # Maintain node
    #rclpy.spin(gcs)

    # Destroy node
    gcs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()