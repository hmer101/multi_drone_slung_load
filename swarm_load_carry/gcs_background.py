# Contains the GCS (ground control station) background class and related methods.
# This class runs the commands to place the load and drones at the desired pose as determined by the gcs_user node.
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

import numpy as np
import quaternionic as qt
import pymap3d as pm
import utils

from swarm_load_carry.state import State, CS_type

from swarm_load_carry_interfaces.srv import SetLocalPose, GetGlobalInitPose
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

class GCSBackground(Node):

    def __init__(self):
        super().__init__('gcs_background')

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

        ## Print information
        self.get_logger().info('GCS BACKGROUND NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## VARIABLES
        self.load_desired_state = State(f'{self.load_id}_init', CS_type.ENU)

        timer_period = 0.02  # seconds
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10
        self.omega = 0.5

        # Timers
        self.timer = self.create_timer(timer_period, self.clbk_send_load_setpoint)

        ## PUBLISHERS
        self.pub_load_attitude_desired = self.create_publisher(VehicleAttitudeSetpoint, f'load_{self.load_id}/in/desired_attitude', qos_profile)
        self.pub_load_position_desired = self.create_publisher(VehicleLocalPositionSetpoint, f'load_{self.load_id}/in/desired_local_position', qos_profile)

        ## SUBSCRIBERS
        ## SERVICES
        ## CLIENTS
        self.cli_set_drone_poses_rel_load = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            # Desired poses rel load
            self.cli_set_drone_poses_rel_load[i-self.first_drone_num] = self.create_client(SetLocalPose,f'/px4_{i}/desired_pose_rel_load')

            while not self.cli_set_drone_poses_rel_load[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for set pose rel load service: drone {i}')

        # Set drone arrangement around load
        self.set_drone_arrangement(2, [2, 2, 2])

        self.get_logger().info('Setup complete')


    ## CALLBACKS
    def clbk_send_load_setpoint(self):
        # Set desired pose
        self.load_desired_state.pos = np.array([0.0, 10, 10]) 
        self.load_desired_state.att_q = qt.array([1.0, 0.0, 0.0, 0.0]) 
        self.theta = self.theta + self.omega * self.dt

        # Send position setpoint
        setpoint_msg_pos = VehicleLocalPositionSetpoint()
        setpoint_msg_pos.x = self.load_desired_state.pos[0] 
        setpoint_msg_pos.y = self.load_desired_state.pos[1] 
        setpoint_msg_pos.z = self.load_desired_state.pos[2]
        self.pub_load_position_desired.publish(setpoint_msg_pos)

        # Send orientation setpoint
        q_d = self.load_desired_state.att_q
        setpoint_msg_att = VehicleAttitudeSetpoint()
        setpoint_msg_att.q_d = [float(q_d.w), float(q_d.x), float(q_d.y), float(q_d.z)]
        self.pub_load_attitude_desired.publish(setpoint_msg_att)

        

    ## HELPERS
    # TODO: Set drones to positions that minimizes sum of squared distance from drone start points to desired points
    def set_drone_arrangement(self, r, z):
        ref_points = utils.generate_points_cylinder(self.num_drones, r, z)

        pos_req_future = [None] * self.num_drones

        # Get current drone positions

        # Set drone arrangements
        for i, next_cli_set_drone_pose in enumerate(self.cli_set_drone_poses_rel_load):
            pos_req = SetLocalPose.Request()
            pos_req.transform_stamped.header.frame_id = f'load{self.load_id}'
            pos_req.transform_stamped.child_frame_id = f'drone{i}'

            pos_req.transform_stamped.transform.translation.x = float(ref_points[i][0])
            pos_req.transform_stamped.transform.translation.y = float(ref_points[i][1])
            pos_req.transform_stamped.transform.translation.z = float(ref_points[i][2])

            pos_req.transform_stamped.transform.rotation.x = float(0)
            pos_req.transform_stamped.transform.rotation.y = float(0)
            pos_req.transform_stamped.transform.rotation.z = float(0)
            pos_req.transform_stamped.transform.rotation.w = float(1)

            pos_req_future[i] = next_cli_set_drone_pose.call_async(pos_req)

        # Wait for response
        for i in range(self.num_drones):
            rclpy.spin_until_future_complete(self, pos_req_future[i])



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