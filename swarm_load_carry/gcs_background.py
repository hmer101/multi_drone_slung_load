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

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

class GCSBackground(Node):

    def __init__(self):
        super().__init__('gcs_background')

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
        self.get_logger().info('GCS BACKGROUND NODE')
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
        self.timer = self.create_timer(timer_period, self.clbk_send_load_setpoint)

        ## PUBLISHERS
        self.pub_load_attitude_desired = self.create_publisher(VehicleAttitudeSetpoint, f'load_{self.load_id}/in/desired_attitude', qos_profile)
        self.pub_load_position_desired = self.create_publisher(VehicleLocalPositionSetpoint, f'load_{self.load_id}/in/desired_local_position', qos_profile)

        ## SUBSCRIBERS
        ## SERVICES
        ## CLIENTS

        self.get_logger().info('Setup complete')


    ## CALLBACKS

    ## MISSION CONTROL
    def clbk_send_load_setpoint(self):
        setpoint_msg = VehicleLocalPositionSetpoint()
        setpoint_msg.x = self.radius * np.cos(self.theta)
        setpoint_msg.y = self.radius * np.sin(self.theta)
        setpoint_msg.z = -10.0
        self.pub_load_position_desired.publish(setpoint_msg)

        self.theta = self.theta + self.omega * self.dt


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs_background = GCSBackground()
    
    # Maintain node
    rclpy.spin(gcs_background)

    # Destroy node
    gcs_background.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()