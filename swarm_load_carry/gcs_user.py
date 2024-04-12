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
from px4_msgs.msg import ManualControlSetpoint


class GCSUser(Node):

    def __init__(self):
        super().__init__('gcs_user')

        ## Print information
        self.get_logger().info('GCS USER INTERACTION NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        self.declare_parameter('load_id', 1)
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('first_drone_num', 1)
        self.declare_parameter('auto_level', 0)
        self.declare_parameter('phase_change_requests_through_background', True)
        self.declare_parameter('user_interaction_through_rc', True)

        self.declare_parameter('timer_period_gcs_user', 0.1)

        self.declare_parameter('rc_aux_thresholds', [0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.95])
        self.declare_parameter('rc_aux_buffer', 0.01)
        

        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value
        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.auto_level = self.get_parameter('auto_level').get_parameter_value().integer_value
        self.phase_change_requests_through_background = self.get_parameter('phase_change_requests_through_background').get_parameter_value().bool_value
        self.user_interaction_through_rc = self.get_parameter('user_interaction_through_rc').get_parameter_value().bool_value

        self.timer_period_gcs_user = self.get_parameter('timer_period_gcs_user').get_parameter_value().double_value

        self.rc_aux_thresholds = np.array(self.get_parameter('rc_aux_thresholds').get_parameter_value().double_array_value)
        self.rc_aux_buffer = self.get_parameter('rc_aux_buffer').get_parameter_value().double_value
        

        ## VARIABLES
        if self.user_interaction_through_rc:
            self.rc_aux_1 = 0.0 # Stores the value of the aux1 output from the RC
            self.rc_aux_1_prev = 0.0 # Stores the previous value of aux1 output to detect changes

            # Timer for actioning any changes
            self.timer = self.create_timer(self.timer_period_gcs_user, self.clbk_cmdloop)

        ### ROS2
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ## PUBLISHERS
        ## SUBSCRIBERS
        # If the user interaction occurs through the RC, need to subscribe to the RC control inputs
        if self.user_interaction_through_rc:
            self.sub_local_position = self.create_subscription(
                ManualControlSetpoint,
                f'px4_{self.first_drone_num}/fmu/out/manual_control_setpoint',
                self.clbk_manual_control_setpoint, 
                qos_profile)  

        ## SERVICES

        ## CLIENTS
        # Send the phase change requests through the background node
        if self.phase_change_requests_through_background:
            self.cli_phase_change = [self.create_client(PhaseChange,f'/gcs_background_{self.load_id}/phase_change_request')]
            
            while not self.cli_phase_change[0].wait_for_service(timeout_sec=3.0): #1.0
                self.get_logger().info(f'Waiting for phase change service request: gcs_1')

        # Send the phase change requests directly to the drones
        else:
            self.cli_phase_change = [None] * self.num_drones
            for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
                self.cli_phase_change[i-self.first_drone_num] = self.create_client(PhaseChange,f'/px4_{i}/phase_change')
                while not self.cli_phase_change[i-self.first_drone_num].wait_for_service(timeout_sec=3.0): #1.0
                    self.get_logger().info(f'Waiting for phase change service: drone {i}')

            ## Command list - select based on level of autonomy
            match(self.auto_level):
                case 0: # Highly manual mode
                    self.command_list = ['t = takeoff', 'f = move into formation', 'e = engage tension', 'm = mission start', 'l = land', 'h = hold', 'k = kill', 'q= quit'] #'u = lift load up', 
                case 1: # Semi-automatic mode
                    self.command_list = ['t = takeoff', 'm = mission start', 'l = land', 'h = hold', 'k = kill', 'q= quit']
                case 2: # Fully automatic mode
                    self.command_list = []


        self.get_logger().info('Setup complete')


    ## CALLBACKS
    def clbk_manual_control_setpoint(self, msg):
        self.rc_aux_1 = msg.aux1

    # Take in user commands through the RC
    def clbk_cmdloop(self):
        # Only execute if the value of the aux1 output has changed
        if self.rc_aux_1 != self.rc_aux_1_prev:
            if self.rc_aux_1 >= (self.rc_aux_thresholds[0] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[0] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_SETUP_DRONE)
            elif self.auto_level == 0 and self.rc_aux_1 >= (self.rc_aux_thresholds[1] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[1] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_TAKEOFF_PRE_TENSION)
            elif self.auto_level == 0 and self.rc_aux_1 >= (self.rc_aux_thresholds[2] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[2] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_TAKEOFF_POST_TENSION)
            elif self.rc_aux_1 >= (self.rc_aux_thresholds[3] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[3] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_MISSION_START)
            elif self.rc_aux_1 >= (self.rc_aux_thresholds[4] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[4] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_LAND_START)
            elif self.rc_aux_1 >= (self.rc_aux_thresholds[5] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[5] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_HOLD)
            elif self.rc_aux_1 >= (self.rc_aux_thresholds[6] - self.rc_aux_buffer) and self.rc_aux_1 <= (self.rc_aux_thresholds[6] + self.rc_aux_buffer):
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_KILL)

            # Update the previous value of aux1 output
            self.rc_aux_1_prev = self.rc_aux_1

            self.get_logger().info(f'New self.rc_aux_1: {self.rc_aux_1}')


    ## MISSION CONTROL
    # Take in user commands through the GCS
    def commands_gcs(self):
        cmd = None

        # Check if command_list is not empty
        if self.command_list:
            # Convert the command list into a formatted string for the prompt
            command_prompt = ', '.join(self.command_list) + ':'
        else:
            # Fallback prompt if command_list is empty
            command_prompt = 'Enter command (q= quit):'

        # Stay in command loop until quit
        while cmd != 'q':
            cmd = input(command_prompt) #'Enter command [t = takeoff, m = mission start, l = land, h = hold, k = kill, q= quit]:') #r = return to home,

            match(cmd):
                case 't':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_SETUP_DRONE)
                case _ if self.auto_level == 0 and cmd == 'f':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_TAKEOFF_PRE_TENSION)
                case _ if self.auto_level == 0 and cmd == 'e':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_TAKEOFF_POST_TENSION)
                # case _ if self.auto_level == 0 and cmd == 'u':
                #     utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_TAKEOFF_POST_TENSION) 
                case 'm':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_MISSION_START)
                case 'l':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_LAND_START)
                # case 'r':
                #     self.phase_change(Phase.PHASE_RTL_START)
                case 'h':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_HOLD)
                case 'k':
                    utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_KILL)


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs_user = GCSUser()

    if gcs_user.user_interaction_through_rc:
        # Take in user commands through RC
        rclpy.spin(gcs_user)
    else:
        # Take in user commands through GCS
        # Done like this because 'input' function is blocking
        gcs_user.commands_gcs()

    # Destroy node
    gcs_user.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()