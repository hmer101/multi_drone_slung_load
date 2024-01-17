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

from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint
from geometry_msgs.msg import Pose, PoseArray

from swarm_load_carry.state import State, CS_type
from swarm_load_carry_interfaces.msg import Phase

#PUB_LOOP_TIMER_PERIOD=0.1

#HEIGHT_DRONE_CS_REL_GND=0 #0.24 # m
#HEIGHT_LOAD_CS_REL_GND=0 #0.1 #0.1 # m
#HEIGHT_DRONE_REL_LOAD=1.5 #2 #m


#q_list = ft.quaternion_from_euler(0, 0, np.pi/2)
#DRONE_1_ORIENT_REL_LOAD = np.quaternion(*q_list) # Drone initial orientation relative to load #TODO: Find better way to set load_initial frame orientation

#t_MARKER_REL_LOAD = np.array([0.0, 0.0, 0.1]) # Marker translation relative to load center
#R_MARKER_REL_LOAD = np.array([0.0, 0.0, np.pi/2]) # Marker rotation relative to load

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
        self.declare_parameter('height_drone_rel_load', 1.5)

        self.declare_parameter('timer_period_load', 0.2)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.env = self.get_parameter('env').get_parameter_value().string_value
        self.load_pose_type = self.get_parameter('load_pose_type').get_parameter_value().string_value
        self.evaluate = self.get_parameter('evaluate').get_parameter_value().bool_value

        self.t_marker_rel_load = np.array(self.get_parameter('t_marker_rel_load').get_parameter_value().double_array_value)
        self.R_marker_rel_load = np.array(self.get_parameter('R_marker_rel_load').get_parameter_value().double_array_value)

        self.height_drone_cs_rel_gnd = self.get_parameter('height_drone_cs_rel_gnd').get_parameter_value().double_value
        self.height_load_cs_rel_gnd = self.get_parameter('height_load_cs_rel_gnd').get_parameter_value().double_value
        self.height_drone_rel_load = self.get_parameter('height_drone_rel_load').get_parameter_value().double_value

        self.timer_period_load = self.get_parameter('timer_period_load').get_parameter_value().double_value
        

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

        ## VARIABLES
        self.load_desired_state = State(f'{self.get_name()}_init', CS_type.ENU)

        self.load_initial_state_rel_world = State('world', CS_type.ENU)
        self.load_state_rel_world = State('world', CS_type.ENU)
        self.load_state_gt = State('ground_truth', CS_type.XYZ)

        self.drone_phases = np.array([-1] * self.num_drones)

        ## TIMERS
        self.timer = self.create_timer(self.timer_period_load, self.clbk_publoop)

        ## PUBLISHERS
        
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
        
        # Subscribe to ground truth load feedback if we are using ground truth, or evaluating the system
        if self.load_pose_type == 'ground_truth' or self.evaluate == True:
            if self.env == 'sim':
                self.sub_load_pose_gt = self.create_subscription(
                    PoseArray,
                    f'load_{self.load_id}/out/pose_ground_truth/gz',
                    self.clbk_load_pose_gt,
                    qos_profile_gt)
            else:
                pass # TODO: Add subscriber for ground truth pose in physical environment
                # self.sub_load_pose_gt = self.create_subscription(
                #     PoseArray,
                #     f'load_{self.load_id}/out/pose_ground_truth/gnss',
                #     self.clbk_load_pose_gt,
                #     qos_profile)

        # Drone current phases
        self.sub_drone_phases = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            callback = lambda msg, drone_ind=(i-self.first_drone_num): self.clbk_update_drone_phase(msg, drone_ind)
        
            self.sub_drone_phases[i-self.first_drone_num] = self.create_subscription(
                Phase,
                f'/px4_{i}/out/current_phase',
                callback,
                qos_profile)

        ## SERVICES
        ## CLIENTS

        ## TFS
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster_init_pose = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster_marker_rel_load = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster_marker_rel_load_gt = StaticTransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        # Get quasi-static load pose estimate for setting load initial TF relative to world
        load_state_rel_world_qs = self.calc_load_pose_quasi_static()
        
        # Publish load pose with selected method
        if self.load_pose_type == 'quasi-static':
            # Set self.load_state_rel_world using quasi-static method
            self.load_state_rel_world = load_state_rel_world_qs

        elif self.load_pose_type == 'ground_truth':
            # Set self.load_state_rel_world using ground truth
            # Convert the ground truth pose to the 'world' frame TODO: HHHHHHEEEEEEEEEEE - THIS IS CAUSING GT FB ISSUE!!!
            self.load_state_rel_world = load_state_rel_world_qs #utils.transform_frames(self.load_state_gt, 'world', self.tf_buffer, self.get_logger())

        #elif self.load_pose_type == 'visual': #TODO: Take estimation from slung_pose_estimation node
            # Set self.load_state_rel_world using visual estimation result

        # Publish load pose if it has been set and if the initial load pose has been set
        if load_state_rel_world_qs != None and self.load_state_rel_world != None:
            # If all drones are in load setup phase, reset load's init pose
            if np.all(self.drone_phases == Phase.PHASE_SETUP_LOAD):
                self.set_tf_init_pose(load_state_rel_world_qs)

            # Publish load relative to load initial position
            load_rel_load_init = utils.transform_frames(self.load_state_rel_world, f'{self.get_name()}_init', self.tf_buffer, self.get_logger(), cs_out_type=CS_type.ENU)

            if load_rel_load_init != None:
                utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', self.get_name(), load_rel_load_init.pos, load_rel_load_init.att_q, self.tf_broadcaster)          


    ## HELPER FUNCTIONS
    def calc_load_pose_quasi_static(self):
        load_state_rel_world_qs = State('world', CS_type.ENU)

        # Retrieve drone information 
        drone_positions = np.zeros((self.num_drones, 3))
        drone_orientations = np.array([np.quaternion(*q) for q in np.zeros((self.num_drones, 4))])

        # Store position and orientation of each drone relative to world
        count_tf = 0

        for i in range(self.num_drones):
            target_frame = 'world'
            source_frame = f'drone{i+self.first_drone_num}'
            
            t = utils.lookup_tf(target_frame, source_frame, self.tf_buffer, rclpy.time.Time(), self.get_logger())

            if t != None:
                drone_positions[i, :] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                drone_orientations[i] = np.quaternion(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z)

                count_tf += 1

        # Only return load position if all drone positions can be found to estimate it
        if count_tf == self.num_drones:
            # Estimate load position as average of drone positions 
            load_state_rel_world_qs.pos  = np.average(drone_positions, axis=0) # TODO: Wrong average!!! Should be geometric average

            # TODO: Better height estimate
            min_height = self.height_load_cs_rel_gnd - self.height_drone_cs_rel_gnd

            # TODO: Better height estimate
            if np.all(self.drone_phases >= Phase.PHASE_TAKEOFF_POST_TENSION):
                load_state_rel_world_qs.pos[2] -= self.height_drone_rel_load 
                load_state_rel_world_qs.pos[2] = max(self.load_state_rel_world.pos[2], min_height) # Ensure load doesn't go below ground
            else: 
                load_state_rel_world_qs.pos[2] = self.height_load_cs_rel_gnd - self.height_drone_cs_rel_gnd

            # Estimate load orientation #TODO: Better orientation estimation method
            load_state_rel_world_qs.att_q = drone_orientations[0] #np.quaternion(*drone_orientations[0, :])

            return load_state_rel_world_qs
        else:
            return None

    def set_tf_init_pose(self, load_initial_state_rel_world):
        # Set initial pose
        self.load_initial_state_rel_world.pos =  np.copy(load_initial_state_rel_world.pos)
        self.load_initial_state_rel_world.att_q = load_initial_state_rel_world.att_q.copy()
        
        # Publish static transform for init pose (relative to world)
        # As CS is in ENU, always aligned
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'world', f'{self.get_name()}_init', self.load_initial_state_rel_world.pos, np.quaternion(*[1.0, 0.0, 0.0, 0.0]), self.tf_static_broadcaster_init_pose)

        # Publish other static transforms
        # Marker relative to load
        q_list = ft.quaternion_from_euler(self.R_marker_rel_load[0], self.R_marker_rel_load[1], self.R_marker_rel_load[2])
        r_marker_rel_load = np.quaternion(*q_list)
        utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}', f'load_marker{self.load_id}', self.t_marker_rel_load, r_marker_rel_load, self.tf_static_broadcaster_marker_rel_load)
        utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_gt', f'load_marker{self.load_id}_gt', self.t_marker_rel_load, r_marker_rel_load, self.tf_static_broadcaster_marker_rel_load_gt)

        # Send complete message
        self.get_logger().info('Initial pose TF set')


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