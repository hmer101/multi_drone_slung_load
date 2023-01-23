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
        print('GCS NODE')
        print(f'Namespace: {self.get_namespace()}')
        print(f'Name: {self.get_name()}')

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

        # self.cli_mode_change_1 = self.create_client(ModeChange,f'/px4_1/mode_change')
        # while not self.cli_mode_change_1.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f'Waiting for offboard ROS start service 1')
        
        # self.cli_mode_change_2 = self.create_client(ModeChange,f'/px4_2/mode_change')
        # while not self.cli_mode_change_2.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f'Waiting for offboard ROS start service 2')

        # self.cli_mode_change_3 = self.create_client(ModeChange,f'/px4_3/mode_change')
        # while not self.cli_mode_change_3.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info(f'Waiting for offboard ROS start service 3')


    ## CALLBACKS

    ## MISSION CONTROL
    # Change the offboard ROS mode of the specified drones
    def mode_change(self, mode_desired):
        # Prepare request
        offboard_mode_req = ModeChange.Request()
        offboard_mode_req.mode = mode_desired 

        # Send request
        offboard_mode_future = [None] * self.num_drones

        for i in range(self.num_drones):
            self.cli_mode_change[i].call_async(offboard_mode_req)
        
        # Wait for response
        for i in range(self.num_drones):
            rclpy.spin_until_future_complete(self, offboard_mode_future[i])
        
        # offboard_mode_future_1 = self.cli_mode_change_1.call_async(offboard_mode_req)
        # offboard_mode_future_2 = self.cli_mode_change_2.call_async(offboard_mode_req)
        # offboard_mode_future_3 = self.cli_mode_change_3.call_async(offboard_mode_req)

        # rclpy.spin_until_future_complete(self, offboard_mode_future_1)
        # rclpy.spin_until_future_complete(self, offboard_mode_future_2)
        # rclpy.spin_until_future_complete(self, offboard_mode_future_3)

        return offboard_mode_future[0].result()


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