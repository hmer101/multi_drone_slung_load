# Contains the Load class and related methods
#
# Author: Harvey Merton
# Date: 01/26/2023

import numpy as np
#import quaternionic as quaternion
import quaternion
import utils
import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

import frame_transforms as ft

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition, VehicleCommand 
from geometry_msgs.msg import Pose, PoseArray

from swarm_load_carry.state import State, CS_type
from swarm_load_carry.pose_pixhawk import PosePixhawk
from swarm_load_carry_interfaces.msg import Phase, GlobalPose

# Could make subclasses for different load types (e.g. camera etc.)
class Load(Node):
    def __init__(self):
        super().__init__('load9')

        ## Print information
        self.load_id = int(str(self.get_name())[-1])

        self.get_logger().info('LOAD NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        self.declare_parameter('env', 'phys')
        self.declare_parameter('load_pose_type', 'quasi-static')
        self.declare_parameter('num_drones', 1)
        self.declare_parameter('first_drone_num', 1)
        self.declare_parameter('evaluate', False)

        self.declare_parameter('t_marker_rel_load', [0.0, 0.0, 0.1])
        self.declare_parameter('R_marker_rel_load', [0.0, 0.0, np.pi/2])

        self.declare_parameter('height_drone_cs_rel_gnd', 0.0)
        self.declare_parameter('height_load_cs_rel_gnd', 0.0)
        self.declare_parameter('r_drones_rel_load', 1.0)

        self.declare_parameter('cable_length', 1.0)
        self.declare_parameter('load_connection_point_r', 0.0)

        self.declare_parameter('timer_period_load', 0.2)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.env = self.get_parameter('env').get_parameter_value().string_value
        self.load_pose_type = self.get_parameter('load_pose_type').get_parameter_value().string_value
        self.evaluate = self.get_parameter('evaluate').get_parameter_value().bool_value

        self.t_marker_rel_load = np.array(self.get_parameter('t_marker_rel_load').get_parameter_value().double_array_value)
        self.R_marker_rel_load = np.array(self.get_parameter('R_marker_rel_load').get_parameter_value().double_array_value)

        self.r_drones_rel_load = self.get_parameter('r_drones_rel_load').get_parameter_value().double_value
        self.height_drone_cs_rel_gnd = self.get_parameter('height_drone_cs_rel_gnd').get_parameter_value().double_value
        self.height_load_cs_rel_gnd = self.get_parameter('height_load_cs_rel_gnd').get_parameter_value().double_value

        self.cable_length = self.get_parameter('cable_length').get_parameter_value().double_value
        self.load_connection_point_r = self.get_parameter('load_connection_point_r').get_parameter_value().double_value

        self.timer_period_load = self.get_parameter('timer_period_load').get_parameter_value().double_value
        
        # Calculate height drone rel load
        self.height_drone_rel_load = utils.drone_height_rel_load(self.cable_length, self.r_drones_rel_load, self.load_connection_point_r)

        # QoS profiles
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile_gt = QoSProfile(
            reliability=qos.ReliabilityPolicy.RELIABLE,
            durability=qos.DurabilityPolicy.VOLATILE,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ## TFS
        self.tf_broadcaster = TransformBroadcaster(self)
        tf_static_broadcaster_init_pose = StaticTransformBroadcaster(self)
        tf_static_broadcaster_marker_rel_load = StaticTransformBroadcaster(self)
        tf_static_broadcaster_marker_rel_load_gt = StaticTransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ## VARIABLES
        self.load_desired_state = State(f'{self.get_name()}_init', CS_type.ENU)

        self.pixhawk_pose = PosePixhawk(self.get_name(), self.env, self.load_pose_type, self.evaluate, self.get_logger(), \
                                        self.tf_broadcaster, tf_static_broadcaster_init_pose, \
                                        tf_static_broadcaster_marker_rel_load, tf_static_broadcaster_marker_rel_load_gt)
  
        self.load_state_gt = State('ground_truth', CS_type.XYZ)

        self.drone_phases = np.array([-1] * self.num_drones)

        ## TIMERS
        self.timer = self.create_timer(self.timer_period_load, self.clbk_publoop)
        self.cnt_phase_ticks = 0

        ## PUBLISHERS
        self.pub_vehicle_command = None # Initialized below

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
        
        # Drone current phases
        self.sub_drone_phases = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            callback = lambda msg, drone_ind=(i-self.first_drone_num): self.clbk_update_drone_phase(msg, drone_ind)
        
            self.sub_drone_phases[i-self.first_drone_num] = self.create_subscription(
                Phase,
                f'/px4_{i}/out/current_phase',
                callback,
                qos_profile)

        
        # Subscribe to ground truth load feedback if we are using ground truth, or evaluating the system
        if self.load_pose_type == 'ground_truth' or self.evaluate == True:
            if self.env == 'sim':
                self.sub_load_pose_gt = self.create_subscription(
                    PoseArray,
                    f'load_{self.load_id}/out/pose_ground_truth/gz',
                    self.clbk_load_pose_gt,
                    qos_profile_gt)
            elif self.env == 'phys':
                # Add subscriber for ground truth pose in physical environment
                # Note that ground truth simply comes from the vehicle local position from the EKF
                self.sub_attitude = self.create_subscription(
                    VehicleAttitude,
                    f'load_{self.load_id}/fmu/out/vehicle_attitude',
                    lambda msg: self.pixhawk_pose.clbk_vehicle_attitude(msg, self.get_clock().now().to_msg()),
                    qos_profile)

                self.sub_local_position = self.create_subscription(
                    VehicleLocalPosition,
                    f'load_{self.load_id}/fmu/out/vehicle_local_position',
                    lambda msg: self.pixhawk_pose.clbk_vehicle_local_position(msg, self.get_clock().now().to_msg()), 
                    qos_profile) 
                 
                self.pub_vehicle_command = self.create_publisher(VehicleCommand, f'load_{self.load_id}/fmu/in/vehicle_command', qos_profile)
                self.sub_global_position = self.create_subscription(
                    VehicleGlobalPosition,
                    f'load_{self.load_id}/fmu/out/vehicle_global_position',
                    lambda msg: self.pixhawk_pose.clbk_vehicle_global_position(msg, np.all(self.drone_phases == Phase.PHASE_SETUP_LOAD), int(self.get_clock().now().nanoseconds/1000), self.pub_vehicle_command), 
                    qos_profile) 
                
                # First drone's global origin - world origin for setting initial pose
                self.sub_global_origin = self.create_subscription(
                    GlobalPose,
                    f'/px4_{self.first_drone_num}/out/global_init_pose', 
                    self.pixhawk_pose.clbk_global_origin,
                    qos_profile)

        
        ## SERVICES
        ## CLIENTS

        self.get_logger().info('Initialization complete')


    ## CALLBACKS
    def clbk_desired_load_attitude(self, msg):
        # Update stored setpoint
        self.load_desired_state.att_q = np.quaternion(*[msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])

        # TEMP: Assume load attitude moves directly to desired attitude (TODO: add dynamics or sensing/estimation. Publish actual attitude in timer clbk instead)

    def clbk_desired_load_local_position(self, msg):
        # Update stored setpoint
        self.load_desired_state.pos = np.array([msg.x, msg.y, msg.z])
        
        # TEMP: Assume load position moves directly to desired position (TODO: add dynamics or sensing/estimation. Publish actual pos in timer clbk instead)

    def clbk_update_drone_phase(self, msg, drone_ind):
        self.drone_phases[drone_ind] = msg.phase
            

    def clbk_load_pose_gt(self, msg):
        # Extract the load pose from the message
        load_pose_gt = utils.extract_pose_from_pose_array_msg(msg, 1)

        self.load_state_gt.pos = np.array([load_pose_gt.position.x, load_pose_gt.position.y, load_pose_gt.position.z])
        self.load_state_gt.att_q = np.quaternion(load_pose_gt.orientation.w, load_pose_gt.orientation.x, load_pose_gt.orientation.y, load_pose_gt.orientation.z)

        # Publish load ground truth
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'ground_truth', f'{self.get_name()}_gt', self.load_state_gt.pos, self.load_state_gt.att_q, self.tf_broadcaster)


    # Loop on timer to publish actual load pose
    def clbk_publoop(self):       
        # TEMP: SPOOF LOAD POSE GT so don't need working load GPS yet (need to spoof init pose for drones + load too)
        #utils.broadcast_tf(self.get_clock().now().to_msg(), 'ground_truth', f'{self.get_name()}_gt', self.load_state_gt.pos, self.load_state_gt.att_q, self.tf_broadcaster)
        
        # Reset flags if all drones are in load setup phase for the first time
        if np.all(self.drone_phases == Phase.PHASE_SETUP_LOAD):
            if self.cnt_phase_ticks == 0:
                self.reset_pre_arm()

            self.cnt_phase_ticks += 1

        # Publish load pose with selected method
        if self.load_pose_type == 'quasi-static' or self.load_pose_type == 'visual': #TODO: Add visual pose estimation
            # Set load_state_rel_world using quasi-static method
            load_state_rel_world = self.calc_load_pose_quasi_static()

            self.get_logger().info(f'load_state_rel_world QS calced!')
            
        
        elif self.load_pose_type == 'ground_truth':
            # Set load_state_rel_world using ground truth
            if self.env == 'sim':
                load_state_rel_world = utils.transform_frames(self.load_state_gt, 'world', self.tf_buffer, self.get_logger(), cs_out_type=CS_type.ENU)

            elif self.env == 'phys':
                # Convert the pixhawk measured pose to the 'world' frame  
                load_state_rel_world = utils.transform_frames(self.pixhawk_pose.local_state, 'world', self.tf_buffer, self.get_logger(), cs_out_type=CS_type.ENU)

                # Cannot find the transform from the load_init to the world frame - local init pose hasn't been set yet
                # Can set it if the global origin state and the gps home have been set
                # Only set if the system is in the setup load phase
                # TODO: TO TEST.
                if load_state_rel_world is None and self.pixhawk_pose.flag_gps_home_set and \
                    self.pixhawk_pose.global_origin_state != self.pixhawk_pose.global_origin_state_prev and np.all(self.drone_phases == Phase.PHASE_SETUP_LOAD):
                    
                    self.pixhawk_pose.set_local_init_pose_non_ref(self.get_clock().now().to_msg(), initial_state_rel_world=None, cs_offset=np.array([0.0, 0.0, self.height_load_cs_rel_gnd]), \
                                                                 item2_name='load_marker', t_item2_rel_item1=self.t_marker_rel_load, R_item2_rel_item1=self.R_marker_rel_load)
                    self.pixhawk_pose.global_origin_state_prev = self.pixhawk_pose.global_origin_state.copy()

        # Publish load pose if it has been set and if the initial load pose has been set
        if load_state_rel_world != None:
            self.get_logger().info(f'load_state_rel_world QS: {load_state_rel_world.pos}')
            
            # If all drones are in load setup phase, setup load
            if np.all(self.drone_phases == Phase.PHASE_SETUP_LOAD):
                self.pixhawk_pose.set_local_init_pose_non_ref(self.get_clock().now().to_msg(), initial_state_rel_world=load_state_rel_world, cs_offset=np.array([0.0, 0.0, 0.0]), item2_name='load_marker', t_item2_rel_item1=self.t_marker_rel_load, R_item2_rel_item1=self.R_marker_rel_load)

                self.get_logger().info(f'SETUP LOAD INIT POSE DONE')
                # TODO: If in physical, ARM LOAD's PX4/start log
            else:
                # Reset phase tick counter so load will reset on next setup
                self.cnt_phase_ticks = 0

            # Set load relative to load initial position for publishing
            load_rel_load_init = utils.transform_frames(load_state_rel_world, f'{self.get_name()}_init', self.tf_buffer, self.get_logger(), cs_out_type=CS_type.ENU)

            # Publish load relative to load initial position
            if load_rel_load_init != None:
                self.get_logger().info(f'load_rel_load_init: {load_rel_load_init.pos} {load_rel_load_init.att_q}')
                utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', self.get_name(), load_rel_load_init.pos, load_rel_load_init.att_q, self.tf_broadcaster)          
                #self.get_logger().info(f'Published load local pose!')
            

    ## HELPER FUNCTIONS
    def reset_pre_arm(self):
        self.pixhawk_pose.reset()

        self.get_logger().info('RESET PRE-ARM COMPLETE')

    def calc_load_pose_quasi_static(self):
        load_state_rel_world_qs = State('world', CS_type.ENU)

        drone_positions, drone_orientations, count_tf = utils.get_drone_poses(self.num_drones, self.first_drone_num, self.tf_buffer, self.get_logger())

        # Only return load position if all drone positions can be found to estimate it
        if count_tf == self.num_drones:
            # Estimate load position as average of drone positions 
            load_state_rel_world_qs.pos  = np.average(drone_positions, axis=0) # TODO: Wrong average!!! Should be geometric average

            # TODO: Better height estimate
            min_height = self.height_load_cs_rel_gnd - self.height_drone_cs_rel_gnd

            # TODO: Better height estimate
            if np.all(self.drone_phases >= Phase.PHASE_TAKEOFF_POST_TENSION):
                load_state_rel_world_qs.pos[2] -= self.height_drone_rel_load 
                load_state_rel_world_qs.pos[2] = max(load_state_rel_world_qs.pos[2], min_height) # Ensure load doesn't go below ground
            else: 
                load_state_rel_world_qs.pos[2] = self.height_load_cs_rel_gnd - self.height_drone_cs_rel_gnd

            # Estimate load orientation #TODO: Better orientation estimation method. 
            # Take the drone's orientation until in formation when the yaw is opposite
            # if self.drone_phases[0] <= Phase.PHASE_TAKEOFF_START:
            #     load_state_rel_world_qs.att_q = drone_orientations[0]
            # else:
            
            # Subtract yaw of pi from drone to get load. Note subtraction is done with quaternion multiplication
            # NOTE: Drone 1 must therefore start pointing towards load otherwise the load's initial orientation will be incorrect!!
            q_w_d1 = drone_orientations[0]
            
            q_d1_L_list = ft.quaternion_from_euler(0, 0, -np.pi)
            q_d1_L = np.quaternion(*q_d1_L_list)

            load_state_rel_world_qs.att_q = q_w_d1*q_d1_L

            return load_state_rel_world_qs
        else:
            return None


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