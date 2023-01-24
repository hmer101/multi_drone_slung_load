# Contains the controller class and related methods
#
# Author: Harvey Merton
# Date: 01/24/2023

import rclpy
from rclpy.node import Node
from swarm_load_carry_interfaces.srv import ModeChange

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        ## Print information
        self.get_logger().info('Controller')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        ## PUBLISHERS
        ## SUBSCRIBERS
        ## SERVICES
        ## CLIENTS
        self.cli_mode_change = self.create_client(ModeChange,f'/gcs_1/mode_change')
        while not self.cli_mode_change.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for GCS mode change service')

        self.get_logger().info('Setup complete')


    ## CALLBACKS

    ## COMMANDS
    # Send mode change request to GCS
    def mode_change(self, mode_desired):
        # Prepare request
        mode_req = ModeChange.Request()
        mode_req.mode = mode_desired 

        # Send request
        mode_future = self.cli_mode_change.call_async(mode_req)
        
        print('CONTROLLER WAITING FOR RESPONSE')

        # Wait for response
        rclpy.spin_until_future_complete(self, mode_future)

        print('CONTROLLER RESPONSE RECEIVED')

        return mode_future.result()


def main(args=None):
    # Create node
    rclpy.init(args=args)
    cntrl = Controller()

    # Send takeoff
    input('Press enter when you want to take off')
    cntrl.mode_change(ModeChange.Request.MODE_TAKEOFF_MAV_START)

    # Send offboard start
    input('Press enter when you want to start offboard control')
    cntrl.mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_START) 

    # Send offboard end
    input('Press enter when you want to end offboard control')
    cntrl.mode_change(ModeChange.Request.MODE_OFFBOARD_ROS_END)

    # Send land
    input('Press enter when you want to land at home')
    cntrl.mode_change(ModeChange.Request.MODE_LAND_MAV_START)


    # Maintain node
    rclpy.spin(cntrl)

    # Destroy node
    cntrl.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()