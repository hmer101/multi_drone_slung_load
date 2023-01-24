# Contains the GCS (ground control station) class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from swarm_load_carry_interfaces.srv import ModeChange

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1

class GroundControlStation(Node):

    def __init__(self):
        super().__init__('gcs')

        ## Print information
        self.ns = self.get_namespace()

        self.get_logger().info('GCS NODE')
        self.get_logger().info(f'Namespace: {self.ns}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)
    
        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value

        ## PUBLISHERS
        ## SUBSCRIBERS
        ## SERVICES
        # self.srv_mode_change = self.create_service(
        #     ModeChange,
        #     f'{self.ns}/mode_change',
        #     self.clbk_mode_change)


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
    def mode_change(self, mode_desired): #clbk_mode_change(self, request, response)
        # Prepare request
        mode_req = ModeChange.Request()
        mode_req.mode = mode_desired #request.mode 

        self.get_logger().info('GCS about to send request')

        # Send request
        mode_future = [None] * self.num_drones

        self.get_logger().info(f'CLI MODE CHANGE: {self.cli_mode_change}')

        for i in range(self.num_drones):
            mode_future[i] = self.cli_mode_change[i].call_async(mode_req)
        
        self.get_logger().info('GCS WAITING FOR RESPONSE')

        # Wait for response
        for i in range(self.num_drones):
            self.get_logger().info(f'Mode_future[{i}]: {mode_future[i]}')
            rclpy.spin_until_future_complete(self, mode_future[i])
            self.get_logger().info(f'RESPONSE FROM DRONE {i}')

        self.get_logger().info('GCS RESPONSE RECEIVED')

        #response.success = True

        return True #response


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
    rclpy.spin(gcs)

    # Destroy node
    gcs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()