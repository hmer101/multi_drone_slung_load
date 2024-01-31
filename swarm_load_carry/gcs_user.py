# Contains the GCS (ground control station) user interation class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile

import numpy as np
from rclpy.node import Node

import utils

from swarm_load_carry_interfaces.srv import PhaseChange
from swarm_load_carry_interfaces.msg import Phase
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint


class GCSUser(Node):

    def __init__(self):
        super().__init__('gcs_user')

        ## Print information
        self.get_logger().info('GCS USER INTERACTION NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('first_drone_num', 1)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value

        ## PUBLISHERS
        ## SUBSCRIBERS
        ## SERVICES

        ## CLIENTS
        self.cli_phase_change = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            self.cli_phase_change[i-self.first_drone_num] = self.create_client(PhaseChange,f'/px4_{i}/phase_change')
            while not self.cli_phase_change[i-self.first_drone_num].wait_for_service(timeout_sec=3.0): #1.0
                self.get_logger().info(f'Waiting for phase change service: drone {i}')

        self.get_logger().info('Setup complete')


    ## CALLBACKS

    ## MISSION CONTROL
    # Take in user commands for the drones
    def user_commands(self):
        cmd = None

        while cmd != 'q':
            cmd = input('Enter command [t = takeoff, m = mission start, l = land, h = hold, k = kill, q= quit]:') #r = return to home,

            match(cmd):
                case 't':
                    utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_SETUP_DRONE)
                case 'm':
                    utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_MISSION_START)
                case 'l':
                    utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_LAND_START)
                # case 'r':
                #     self.phase_change(Phase.PHASE_RTL_START)
                case 'h':
                    utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_HOLD)
                case 'k':
                    utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_KILL)


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs_user = GCSUser()

    # Take in user commands (blocking)
    gcs_user.user_commands()

    # Destroy node
    gcs_user.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()