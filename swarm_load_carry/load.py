# Contains the Load class and related methods
#
# Author: Harvey Merton
# Date: 01/26/2023

import numpy as np
import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint, VehicleLocalPosition, VehicleLocalPositionSetpoint

# Could make subclasses for different load types (e.g. camera etc.)
class Load(Node):
    def __init__(self):
        super().__init__('load')

        # self.load_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        # self.load_local_position = np.array([0.0, 0.0, 0.0])
        # #self.load_local_velocity = np.array([0.0, 0.0, 0.0])
        # self.load_setpoint_position = np.array([0.0, 0.0, 0.0])

        # timer_period = 0.02  # seconds
        # self.dt = timer_period
        # self.theta = 0.0
        # self.radius = self.drone_id*5
        # self.omega = 0.5

        ## Print information
        self.load_id = int(str(self.get_name())[-1])

        self.get_logger().info('LOAD NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ## TIMERS
        

        ## PUBLISHERS
        self.pub_load_attitude = self.create_publisher(VehicleAttitude, f'load_{self.load_id}/out/attitude', qos_profile)
        self.pub_load_position = self.create_publisher(VehicleLocalPosition, f'load_{self.load_id}/out/local_position', qos_profile)
        

        ## SUBSCRIBERS
        self.sub_load_attitude_desired = self.create_subscription(
            VehicleAttitudeSetpoint,
            f'load_{self.load_id}/in/desired_attitude',
            self.clbk_desired_load_attitude,
            qos_profile)

        self.sub_load_position_desired = self.create_subscription(
            VehicleLocalPositionSetpoint,
            f'load_{self.load_id}/in/desired_local_position',
            self.clbk_desired_load_local_position,
            qos_profile)

        ## SERVICES
        ## CLIENTS

    ## CALLBACKS
    def clbk_desired_load_attitude(self, msg):
        load_attitude = VehicleAttitude()
        load_attitude.q = msg.q_d

        self.pub_load_attitude.publish(load_attitude)
    
    def clbk_desired_load_local_position(self, msg):
        load_position = VehicleLocalPosition()
        load_position.x = msg.x
        load_position.y = msg.y
        load_position.z = msg.z

        self.pub_load_position.publish(load_position)
        self.get_logger().info(f'Received and published desired pos: {load_position.x, load_position.y, load_position.z}')    


def main():
    # Create node
    rclpy.init()
    load = Load()

    # Maintain node
    rclpy.spin(load)

    # Destroy node
    load.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()