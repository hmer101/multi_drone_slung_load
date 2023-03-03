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
        self.cli_get_drone_init_global_poses = [None] * self.num_drones
        self.cli_set_drone_init_local_poses = [None] * self.num_drones
        self.cli_set_drone_poses_rel_load = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            # Global initial poses
            self.cli_get_drone_init_global_poses[i-self.first_drone_num] = self.create_client(GetGlobalInitPose,f'/px4_{i}/global_initial_pose')

            while not self.cli_get_drone_init_global_poses[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for global initial pose service drone {i}')

            # Local initial poses
            self.cli_set_drone_init_local_poses[i-self.first_drone_num] = self.create_client(SetLocalPose,f'/px4_{i}/local_initial_pose')

            while not self.cli_set_drone_init_local_poses[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for local initial pose service drone {i}')

            # Desired poses rel load
            self.cli_set_drone_poses_rel_load[i-self.first_drone_num] = self.create_client(SetLocalPose,f'/px4_{i}/desired_pose_rel_load')

            while not self.cli_set_drone_poses_rel_load[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for set pose rel load service: drone {i}')


        # Send initial pose (relative to world) tfs
        self.send_initial_pose_tfs()

        # Set drone arrangement around load
        self.set_drone_arrangement(2, [2, 2, 2])

        self.get_logger().info('Setup complete')


    ## CALLBACKS
    def clbk_send_load_setpoint(self):
        # Set desired pose
        self.load_desired_state.pos = np.array([0.0, 10, 10]) #np.array([self.radius * np.cos(self.theta), self.radius * np.sin(self.theta), 5]) 
        self.load_desired_state.att_q = qt.array([1.0, 0.0, 0.0, 0.0])
        self.theta = self.theta + self.omega * self.dt

        # Send position setpoint
        setpoint_msg_pos = VehicleLocalPositionSetpoint()
        setpoint_msg_pos.x = self.load_desired_state.pos[0] #
        setpoint_msg_pos.y = self.load_desired_state.pos[1] #
        setpoint_msg_pos.z = self.load_desired_state.pos[2]
        self.pub_load_position_desired.publish(setpoint_msg_pos)

        # Send orientation setpoint
        q_d = self.load_desired_state.att_q
        setpoint_msg_att = VehicleAttitudeSetpoint()
        setpoint_msg_att.q_d = [float(q_d.x), float(q_d.y), float(q_d.z), float(q_d.w)]
        self.pub_load_attitude_desired.publish(setpoint_msg_att)

        

    ## HELPERS
     # Set initial drone poses relative to world
    def send_initial_pose_tfs(self):
        ## Get drone initial positions
        # Prepare request
        init_global_pose_req = GetGlobalInitPose.Request()

        # Send request
        init_global_pose_future = [None] * self.num_drones

        for i in range(self.num_drones):
            init_global_pose_future[i] = self.cli_get_drone_init_global_poses[i].call_async(init_global_pose_req)

        # Wait for response and accumulate result
        drone_global_init_poses = [None] * self.num_drones

        for i, next_future in enumerate(init_global_pose_future):
            rclpy.spin_until_future_complete(self, next_future)
            drone_global_init_poses[i] = next_future.result()


        ## Set drone local initial poses relative to world (allowing the drones to publish local tfs)       
        # Send request
        future_drone_init_local_pose = [None] * self.num_drones

        for i, next_drone_init_pose in enumerate(drone_global_init_poses):
            req = SetLocalPose.Request()

            req.transform_stamped.header.stamp = self.get_clock().now().to_msg()
            req.transform_stamped.header.frame_id = 'world'
            req.transform_stamped.child_frame_id = f'drone{self.first_drone_num + i}_init'

            req.transform_stamped.transform.translation.x, req.transform_stamped.transform.translation.y, req.transform_stamped.transform.translation.z = pm.geodetic2enu(next_drone_init_pose.global_pos.lat, next_drone_init_pose.global_pos.lon, next_drone_init_pose.global_pos.alt, drone_global_init_poses[0].global_pos.lat, drone_global_init_poses[0].global_pos.lon, drone_global_init_poses[0].global_pos.alt) 
                
            # All in ENU co-ordinates so no relative orientation
            req.transform_stamped.transform.rotation.x = 0.0
            req.transform_stamped.transform.rotation.y = 0.0
            req.transform_stamped.transform.rotation.z = 0.0
            req.transform_stamped.transform.rotation.w = 1.0

            future_drone_init_local_pose[i] = self.cli_set_drone_init_local_poses[i].call_async(req)


        # Wait for response 
        for i, next_future in enumerate(future_drone_init_local_pose):
            rclpy.spin_until_future_complete(self, next_future)

            self.get_logger().info(f'Local TF set for drone {i+1}')
            

    # TODO: Set drones to positions that minimizes sum of squared distance from drone start points to desired points
    def set_drone_arrangement(self, r, z):
        ref_points = utils.generate_points_cylinder(self.num_drones, r, z)
        #print(f'REF POINTS: \n {ref_points}')

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