# Contains the GCS (ground control station) user interation class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile

import numpy as np
from rclpy.node import Node

from swarm_load_carry_interfaces.srv import ModeChange
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

class GCSUser(Node):

    def __init__(self):
        super().__init__('gcs_user')

        ## Print information
        self.get_logger().info('GCS USER INTERACTION NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value

        ## PUBLISHERS
        ## SUBSCRIBERS
        ## SERVICES

        ## CLIENTS
        self.cli_mode_change = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            self.cli_mode_change[i-self.first_drone_num] = self.create_client(ModeChange,f'/px4_{i}/mode_change')
            while not self.cli_mode_change[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for offboard ROS start service {i}')

        self.get_logger().info('Setup complete')


    ## CALLBACKS

    ## MISSION CONTROL
    # Change the mode of all drones
    def mode_change(self, mode_desired):
        # Prepare request
        mode_req = ModeChange.Request()
        mode_req.mode = mode_desired

        # Send request
        mode_future = [None] * self.num_drones

        for i in range(self.num_drones):
            mode_future[i] = self.cli_mode_change[i].call_async(mode_req)

        # Wait for response
        for i in range(self.num_drones):
            rclpy.spin_until_future_complete(self, mode_future[i])

    # Take in user commands for the drones
    def user_commands(self):
        cmd = None

        while cmd != 'q':
            cmd = input('Enter command [t = takeoff, o = offboard control, r = return to home, h = hold, k = kill, q= quit]:')

            match(cmd):
                case 't':
                    self.mode_change(ModeChange.Request.MODE_TAKEOFF_MAV_START)
                case 'o':
                    self.mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_START)
                case 'r':
                    self.mode_change(ModeChange.Request.MODE_LAND_MAV_START)
                case 'h':
                    self.mode_change(ModeChange.Request.MODE_HOLD)
                case 'k':
                    self.mode_change(ModeChange.Request.MODE_KILL)


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs_user = GCSUser()

    # Take in user commands (blocking)
    gcs_user.user_commands()

    # # Destroy node
    gcs_user.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()