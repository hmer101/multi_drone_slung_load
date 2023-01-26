# Contains the GCS (ground control station) class and related methods
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
from threading import Thread

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

class GroundControlStation(Node):

    def __init__(self):
        super().__init__('gcs')

        self.load_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.load_local_position = np.array([0.0, 0.0, 0.0])
        #self.load_local_velocity = np.array([0.0, 0.0, 0.0])
        self.load_setpoint_position = np.array([0.0, 0.0, 0.0])

        timer_period = 0.02  # seconds
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10
        self.omega = 0.5

        ## Print information
        self.get_logger().info('GCS NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS

        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)
        self.declare_parameter('load_id', DEFAULT_LOAD_ID)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value

        # Timers
        #self.timer = self.create_timer(timer_period, self.clbk_cmdloop)

        ## PUBLISHERS
        self.pub_load_attitude_desired = self.create_publisher(VehicleAttitudeSetpoint, f'load_{self.load_id}/in/desired_attitude', qos_profile)
        self.pub_load_position_desired = self.create_publisher(VehicleLocalPositionSetpoint, f'load_{self.load_id}/in/desired_local_position', qos_profile)

        ## SUBSCRIBERS
        ## SERVICES

        ## CLIENTS
        self.cli_mode_change = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            self.cli_mode_change[i-self.first_drone_num] = self.create_client(ModeChange,f'/px4_{i}/mode_change')
            while not self.cli_mode_change[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for offboard ROS start service {i}')

        self.get_logger().info('Setup complete')


        ## THREADS
        # self.th_user_input = Thread(target = self.user_commands)
        # self.th_user_input.start()

        self.th_publish_load_setpoint = Thread(target = self.send_load_setpoint)
        self.th_publish_load_setpoint.start()


    def __del__(self):
        self.th_publish_load_setpoint.join()


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


    def send_load_setpoint(self): #clbk_cmdloop
        setpoint_msg = VehicleLocalPositionSetpoint()
        setpoint_msg.x = self.radius * np.cos(self.theta)
        setpoint_msg.y = self.radius * np.sin(self.theta)
        setpoint_msg.z = -10.0
        self.pub_load_position_desired.publish(setpoint_msg)

        self.theta = self.theta + self.omega * self.dt


# def start_commands_thread():
#     th_user_input = Thread(target = gcs.user_commands)
#     th_user_input.start()

def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs = GroundControlStation()

    # Take in user commands whilst spinning node
    # th_user_input = Thread(target = gcs.user_commands)
    # th_user_input.start()
    
    # Maintain node
    #th_user_input.join()
    rclpy.spin(gcs) #also run in own thread?

    # Destroy node
    gcs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()