# Contains the drone class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import asyncio, rclpy, utils # Note import utils needs additions to setup.py. See here: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
import swarm_load_carry.drone_offboard_ros as offboard_ros
import numpy as np
# import pymap3d as pm
import quaternion

import frame_transforms as ft

from swarm_load_carry.state import State, CS_type
from swarm_load_carry.pose_pixhawk import PosePixhawk

import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose, PoseArray
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, VehicleControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, VehicleGlobalPosition
from swarm_load_carry_interfaces.srv import PhaseChange, SetLocalPose # Note must build workspace and restart IDE before custom packages are found by python
from swarm_load_carry_interfaces.msg import Phase, GlobalPose

# Node to encapsulate drone information and actions
class Drone(Node):

    ## Initialization
    def __init__(self, name, namespace):
        super().__init__(node_name=name, namespace=namespace)

        self.ns = self.get_namespace()
        self.drone_id = int(str(self.ns)[-1])

        ## PARAMETERS
        self.declare_parameter('env', 'phys')

        self.declare_parameter('load_id', 1)
        self.declare_parameter('load_pose_type', 'quasi-static')
        self.declare_parameter('evaluate', False)

        self.declare_parameter('num_drones', 1)
        self.declare_parameter('num_cameras', 0)
        self.declare_parameter('first_drone_num', 0) #1
        
        self.declare_parameter('auto_level', 0)
        
        #self.declare_parameter('vel_drone', 0.1)
        self.declare_parameter('yawspeed_drone', 0.393)

        self.declare_parameter('height_drone_cs_rel_gnd', 0.0)
        self.declare_parameter('height_load_pre_tension', -0.2)
        self.declare_parameter('pos_threshold', 0.3)

        self.declare_parameter('cable_length', 1.0)
        self.declare_parameter('load_connection_point_r', 0.0)

        self.declare_parameter('takeoff_height_load', 1.0)
        self.declare_parameter('r_drones_rel_load', 1.0)

        self.declare_parameter('t_cam_rel_pixhawk', [-0.1, 0.03, -0.025])
        self.declare_parameter('R_cam_rel_pixhawk', [np.pi, 0.0, -np.pi/2])

        self.declare_parameter('timer_period_drone', 0.1)

        self.declare_parameter('cnt_threshold_drone_setup', 50) # Defaults set when timer_period_drone = 0.1
        self.declare_parameter('cnt_threshold_takeoff_start', 30)
        self.declare_parameter('cnt_threshold_takeoff_pre_tension', 50)
        self.declare_parameter('cnt_threshold_fully_auto_pre_takeoff', 50)
        self.declare_parameter('cnt_threshold_land_pre_descent', 50)
        self.declare_parameter('cnt_threshold_land_post_load_down', 50)
        self.declare_parameter('cnt_threshold_land_drones', 50)
        self.declare_parameter('cnt_threshold_land_pre_disarm', 30)


        self.env = self.get_parameter('env').get_parameter_value().string_value

        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value
        self.load_name = f'load{self.load_id}'
        self.load_pose_type = self.get_parameter('load_pose_type').get_parameter_value().string_value
        self.evaluate = self.get_parameter('evaluate').get_parameter_value().bool_value

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.num_cameras = self.get_parameter('num_cameras').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        
        self.auto_level = self.get_parameter('auto_level').get_parameter_value().integer_value
        
        #self.vel_drone = self.get_parameter('vel_drone').get_parameter_value().double_value
        self.yawspeed_drone = self.get_parameter('yawspeed_drone').get_parameter_value().double_value

        self.height_drone_cs_rel_gnd = self.get_parameter('height_drone_cs_rel_gnd').get_parameter_value().double_value
        self.height_load_pre_tension = self.get_parameter('height_load_pre_tension').get_parameter_value().double_value
        self.pos_threshold = self.get_parameter('pos_threshold').get_parameter_value().double_value

        self.takeoff_height_load = self.get_parameter('takeoff_height_load').get_parameter_value().double_value
        self.r_drones_rel_load = self.get_parameter('r_drones_rel_load').get_parameter_value().double_value

        self.cable_length = self.get_parameter('cable_length').get_parameter_value().double_value
        self.load_connection_point_r = self.get_parameter('load_connection_point_r').get_parameter_value().double_value

        self.t_cam_rel_pixhawk = np.array(self.get_parameter('t_cam_rel_pixhawk').get_parameter_value().double_array_value)
        self.R_cam_rel_pixhawk = np.array(self.get_parameter('R_cam_rel_pixhawk').get_parameter_value().double_array_value)

        self.timer_period_drone = self.get_parameter('timer_period_drone').get_parameter_value().double_value

        self.cnt_threshold_drone_setup = self.get_parameter('cnt_threshold_drone_setup').get_parameter_value().integer_value
        self.cnt_threshold_takeoff_start = self.get_parameter('cnt_threshold_takeoff_start').get_parameter_value().integer_value
        self.cnt_threshold_takeoff_pre_tension = self.get_parameter('cnt_threshold_takeoff_pre_tension').get_parameter_value().integer_value
        self.cnt_threshold_fully_auto_pre_takeoff = self.get_parameter('cnt_threshold_fully_auto_pre_takeoff').get_parameter_value().integer_value
        self.cnt_threshold_land_pre_descent = self.get_parameter('cnt_threshold_land_pre_descent').get_parameter_value().integer_value
        self.cnt_threshold_land_post_load_down = self.get_parameter('cnt_threshold_land_post_load_down').get_parameter_value().integer_value
        self.cnt_threshold_land_drones = self.get_parameter('cnt_threshold_land_drones').get_parameter_value().integer_value
        self.cnt_threshold_land_pre_disarm = self.get_parameter('cnt_threshold_land_pre_disarm').get_parameter_value().integer_value

        # Calculate height drone rel load
        self.height_drone_rel_load = utils.drone_height_rel_load(self.cable_length, self.r_drones_rel_load, self.load_connection_point_r)
        
        ## TFS
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)
        tf_static_broadcaster_init_pose = StaticTransformBroadcaster(self) #self.
        tf_static_broadcaster_cam_rel_drone = StaticTransformBroadcaster(self)
        tf_static_broadcaster_cam_rel_drone_gt = StaticTransformBroadcaster(self)
        tf_static_broadcaster_world_rel_gt = StaticTransformBroadcaster(self)
        

        ## VARIABLES
        # Vehicle
        self.vehicle_status = None  # Current state of the FMU
        self.phase = Phase.PHASE_UNASSIGNED        # Desired phase (action to perform when in offboard mode. Use 'phase' to differentiate from 'mode' of the FMU)
        self.phase_restore = Phase.PHASE_UNASSIGNED # Phase to restore to after dropping out of offboard mode 

        self.pixhawk_pose = PosePixhawk(self.get_name(), self.env, self.load_pose_type, self.evaluate, self.get_logger(), \
                                        self.tf_broadcaster, tf_static_broadcaster_init_pose,  \
                                        tf_static_broadcaster_cam_rel_drone, tf_static_broadcaster_cam_rel_drone_gt, \
                                        tf_static_broadcaster_world_rel_gt)

        self.vehicle_state_gt = State('ground_truth', CS_type.XYZ)
        self.vehicle_desired_state_rel_load = State(f'{self.load_name}', CS_type.ENU)

        # Load
        self.load_desired_local_state = State(f'{self.load_name}_init', CS_type.ENU)

        
        ## TIMERS
        self.timer = self.create_timer(self.timer_period_drone, self.clbk_cmdloop)
        self.cnt_phase_ticks = 0

        ### ROS2
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

        
        ## PUBLISHERS
        # Local FMU inputs
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', qos_profile)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', qos_profile)
        self.pub_vehicle_command = self.create_publisher(VehicleCommand, f'{self.ns}/fmu/in/vehicle_command', qos_profile)

        # To drone system
        self.pub_current_phase = self.create_publisher(Phase, f'{self.ns}/out/current_phase', qos_profile)
        self.pub_global_init_pose = self.create_publisher(GlobalPose, f'{self.ns}/out/global_init_pose', qos_profile)

        ## SUBSCRIBERS
        # Local FMU outputs
        # See full available list here: https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.ns}/fmu/out/vehicle_status',
            self.clbk_vehicle_status,
            qos_profile)

        self.sub_attitude = self.create_subscription(
            VehicleAttitude,
            f'{self.ns}/fmu/out/vehicle_attitude',
            lambda msg: self.pixhawk_pose.clbk_vehicle_attitude(msg, self.get_clock().now().to_msg()),
            qos_profile)

        self.sub_local_position = self.create_subscription(
            VehicleLocalPosition,
            f'{self.ns}/fmu/out/vehicle_local_position',
            lambda msg: self.pixhawk_pose.clbk_vehicle_local_position(msg, self.get_clock().now().to_msg()), 
            qos_profile)  
        
        self.sub_global_position = self.create_subscription(
            VehicleGlobalPosition,
            f'{self.ns}/fmu/out/vehicle_global_position',
            lambda msg: self.pixhawk_pose.clbk_vehicle_global_position(msg, self.phase == Phase.PHASE_SETUP_DRONE, int(self.get_clock().now().nanoseconds/1000), self.pub_vehicle_command, self.pub_global_init_pose),
            qos_profile) 

        # Payload 
        self.sub_payload_attitude_desired = self.create_subscription(
            VehicleAttitudeSetpoint,
            f'/load_{self.load_id}/in/desired_attitude',
            self.clbk_load_desired_attitude,
            qos_profile)
        
        self.sub_payload_position_desired = self.create_subscription(
            VehicleLocalPositionSetpoint,
            f'/load_{self.load_id}/in/desired_local_position',
            self.clbk_load_desired_local_position,
            qos_profile)

        # Other drones
        if not self.drone_id == self.first_drone_num:
            self.sub_global_origin = self.create_subscription(
                GlobalPose,
                f'/px4_{self.first_drone_num}/out/global_init_pose', 
                self.pixhawk_pose.clbk_global_origin, #lambda msg: self.pixhawk_pose.clbk_global_origin(msg),
                qos_profile)
        
        # Ground truth
        self.sub_vehicle_pose_gt = None

        if self.load_pose_type == 'ground_truth' or self.evaluate == True:
            if self.env == 'sim':               
                self.sub_vehicle_pose_gt = self.create_subscription(
                    PoseArray,
                    f'/px4_{self.drone_id}/out/pose_ground_truth/gz',
                    self.clbk_vehicle_pose_gt,
                    qos_profile_gt)
            else:
                # Closest we can get to ground truth in physical environment is to use the EKF position estimate using the RTK GPS
                # This is handled in the clbk_vehicle_local_position
                pass
                
                
        ## SERVICES
        self.srv_phase_change = self.create_service(
            PhaseChange,
            f'{self.ns}/phase_change',
            self.clbk_change_phase)
        
        self.srv_set_pose_rel_load = self.create_service(
            SetLocalPose,
            f'{self.ns}/desired_pose_rel_load',
            self.clbk_set_desired_pose_rel_load)

        ## CLIENTS

        ## FLAGS 
        # Ensure set services are called at least once before taking off)
        self.flag_desired_pose_rel_load_set = False


        ## Print information
        self.get_logger().info('DRONE NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')
    

    # Create a node with MAVLINK connections initialized
    @classmethod
    def create(cls, node_name='drone9', namespace='px4_9'):
        # Create node without MAVLINK connections (i.e. only has ROS connections here)
        self = Drone(name=node_name, namespace=namespace)
        self.get_logger().info('Initialization complete')

        return self


    ## CALLBACKS
    def clbk_vehicle_status(self, msg):
        self.vehicle_status = msg

    def clbk_vehicle_pose_gt(self, msg):
        # Extract the drone pose from the message
        drone_pose_gt = utils.extract_pose_from_pose_array_msg(msg, 2) #2 with cameras on in gz, 1 without

        self.vehicle_state_gt.pos = np.array([drone_pose_gt.position.x, drone_pose_gt.position.y, drone_pose_gt.position.z])
        self.vehicle_state_gt.att_q = np.quaternion(drone_pose_gt.orientation.w, drone_pose_gt.orientation.x, drone_pose_gt.orientation.y, drone_pose_gt.orientation.z)

        # Publish drone ground truth
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'ground_truth', f'{self.get_name()}_gt', self.vehicle_state_gt.pos, self.vehicle_state_gt.att_q, self.tf_broadcaster)


    def clbk_load_desired_attitude(self, msg):
        self.load_desired_local_state.att_q = np.quaternion(msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3])

    def clbk_load_desired_local_position(self, msg):
        self.load_desired_local_state.pos = np.array([msg.x, msg.y, msg.z])
        
    def clbk_set_desired_pose_rel_load(self, request, response):
        self.vehicle_desired_state_rel_load.pos = np.array([request.transform_stamped.transform.translation.x, request.transform_stamped.transform.translation.y, request.transform_stamped.transform.translation.z])
        self.vehicle_desired_state_rel_load.att_q = np.quaternion(request.transform_stamped.transform.rotation.w, request.transform_stamped.transform.rotation.x, request.transform_stamped.transform.rotation.y, request.transform_stamped.transform.rotation.z) 

        self.flag_desired_pose_rel_load_set = True
        response.success = True

        return response


    def clbk_change_phase(self, request, response):

        # Change phase to the beginning of major phases (some minor phase transitions are enabled in manual mode, or automatically occur)
        match request.phase_request.phase:
            case Phase.PHASE_SETUP_DRONE:
                self.phase=Phase.PHASE_SETUP_DRONE

            # case Phase.PHASE_TAKEOFF_START: # Minor
            #     self.phase = Phase.PHASE_TAKEOFF_START

            case Phase.PHASE_TAKEOFF_PRE_TENSION: # Minor
                self.phase=Phase.PHASE_TAKEOFF_PRE_TENSION

            case Phase.PHASE_TAKEOFF_POST_TENSION: # Minor
                self.phase=Phase.PHASE_TAKEOFF_POST_TENSION

            case Phase.PHASE_MISSION_START:
                self.phase=Phase.PHASE_MISSION_START
            
            case Phase.PHASE_LAND_START:
                self.phase=Phase.PHASE_LAND_START

            case Phase.PHASE_HOLD:
                self.phase=Phase.PHASE_HOLD

            case Phase.PHASE_KILL:
                self.phase=Phase.PHASE_KILL


        response.success = True
        self.get_logger().info(f'Requested change to phase: {self.phase}')

        return response


    def clbk_cmdloop(self):  
        # Safety switch for switching out of offboard mode and back into offboard mode
        if self.vehicle_status is not None: # and if self.env == 'phys':
            # If taken out of offboard mode by RC, and not pre-setup, switch to hold phase
            # Don't include land drones phase as intentionally switch out of offboard mode here
            if self.vehicle_status.nav_state_user_intention != VehicleStatus.NAVIGATION_STATE_OFFBOARD \
                and self.phase >= Phase.PHASE_TAKEOFF_START and self.phase != Phase.PHASE_HOLD and self.phase != Phase.PHASE_KILL and self.phase != Phase.PHASE_LAND_DRONES and self.phase != Phase.PHASE_LAND_END:
                self.phase_restore = self.phase
                self.phase = Phase.PHASE_HOLD
                self.get_logger().info(f'Offboard mode switched off. Switching to hold phase.')

            # If only just put into offboard mode by RC, and a phase to restore has been set, restore to that phase
            elif self.vehicle_status.nav_state_user_intention == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.phase_restore != Phase.PHASE_UNASSIGNED:
                self.phase = self.phase_restore
                self.phase_restore = Phase.PHASE_UNASSIGNED
                self.get_logger().info(f'Offboard mode switched on. Restoring to phase {self.phase}.')


        # Continually publish offboard mode heartbeat (note need setpoint published too to stay in offboard mode)
        timestamp = int(self.get_clock().now().nanoseconds/1000)
        offboard_ros.publish_offboard_control_heartbeat_signal(self.pub_offboard_mode, 'pos', timestamp)
        
        # Get TF relative to world if the world position is set
        if self.pixhawk_pose.flag_local_init_pose_set:
            # Get actual position feedback 
            # Start with drone position (TODO: incorporate load position feedback later)
            tf_drone_rel_world = utils.lookup_tf('world', self.get_name(), self.tf_buffer, rclpy.time.Time(), self.get_logger())


        # Start setup and take-off automatically if in fully-auto mode. 
        if (self.auto_level == 2) and self.phase == Phase.PHASE_UNASSIGNED and self.vehicle_status is not None:
            # Counter starts when attempt to put into offboard mode by RC
            if self.vehicle_status.nav_state_user_intention == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if self.cnt_phase_ticks >= self.cnt_threshold_fully_auto_pre_takeoff:
                    self.phase = Phase.PHASE_SETUP_DRONE
                    self.cnt_phase_ticks = 0

                else:
                    self.cnt_phase_ticks += 1

        # Generate trajectory message for formation used after take-off.
        elif self.phase > Phase.PHASE_SETUP_GCS:
            # Note speed setpoints are not included by default (include in particular phases below)
            trajectory_msg = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, self.load_desired_local_state, self.get_name(), self.tf_buffer, timestamp, self.get_logger())
            #trajectory_msg_with_speed = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, self.load_desired_local_state, self.get_name(), self.tf_buffer, timestamp, self.get_logger(), drone_prev_local_state=self.vehicle_local_state, v_scalar=self.vel_drone, yawspeed_scalar=self.yawspeed_drone)

            if trajectory_msg == None:
                #self.get_logger().warn(f'Load or drone initial position not found. Skipping this command loop.') TODO: re-enable
                return


        # Perform actions depending on what mode is requested
        # TODO: Add some formation feedback to all phases (especially mission)
        match self.phase:
            # Run origin-altering setup pre-arming
            case Phase.PHASE_SETUP_DRONE:               
                # Reset FMU and TF home positions
                if self.cnt_phase_ticks == 0:
                    #self.flag_reset_pre_arm_complete = False
                    self.reset_pre_arm()
                
                elif self.cnt_phase_ticks > self.cnt_threshold_drone_setup: #Note: Tried flag (self.flag_reset_pre_arm_complete) but didn't work. Perhaps other processes on PX4 need time to reset (i.e. to get to reset what is published on the global pose topic)
                    self.get_logger().info(f'flag_gps_home_set: {self.pixhawk_pose.flag_gps_home_set}, self.pixhawk_pose.flag_local_init_pose_set: {self.pixhawk_pose.flag_local_init_pose_set}')
                    
                    # Set initial poses when ready
                    if self.pixhawk_pose.flag_gps_home_set and not self.pixhawk_pose.flag_local_init_pose_set:                    
                        # Rest of setup differs for first drone and others
                        if self.drone_id == self.first_drone_num:
                            #self.set_local_init_pose_first_drone()
                            self.pixhawk_pose.set_local_init_pose_ref(self.get_clock().now().to_msg(), cs_offset=np.array([0.0, 0.0, self.height_drone_cs_rel_gnd]), \
                                                                      state_gt=self.vehicle_state_gt, item2_name='camera', t_item2_rel_item1=self.t_cam_rel_pixhawk, R_item2_rel_item1=self.R_cam_rel_pixhawk)

                        else:
                            # Set init poses once the first drone's initial position has been received
                            if self.pixhawk_pose.global_origin_state != self.pixhawk_pose.global_origin_state_prev:
                                self.pixhawk_pose.set_local_init_pose_non_ref(self.get_clock().now().to_msg(), initial_state_rel_world=None, cs_offset=np.array([0.0, 0.0, self.height_drone_cs_rel_gnd]), \
                                                                 item2_name='camera', t_item2_rel_item1=self.t_cam_rel_pixhawk, R_item2_rel_item1=self.R_cam_rel_pixhawk)
                                #self.set_local_init_pose_later_drones()
                                self.pixhawk_pose.global_origin_state_prev = self.pixhawk_pose.global_origin_state.copy()
                    
                    # Exit setup only once drone's GPS home and initial positions have been set, 
                    # the drone's desired pose relative to the load has been set and the load's initial pose has been set
                    elif self.pixhawk_pose.flag_gps_home_set and self.pixhawk_pose.flag_local_init_pose_set: #and self.flag_desired_pose_rel_load_set: # and (tf_load_init_rel_world != None):
                        self.cnt_phase_ticks = 0
                        self.phase = Phase.PHASE_SETUP_LOAD
                        self.get_logger().info(f'Drone setup complete')

                        return
                        
                self.cnt_phase_ticks += 1

            # Wait for load to be set up (load set up requires drones to be properly set up). Have an extra phase to ensure drones are properly set up before
            # load pulls the transforms to set up, otherwise can inadvertently pull old transforms
            case Phase.PHASE_SETUP_LOAD:
                # Check if load's setup is complete
                tf_load_init_rel_world = utils.lookup_tf('world', f'{self.load_name}_init', self.tf_buffer, rclpy.time.Time(), self.get_logger())

                # Exit setup only once load's initial pose has been set
                if tf_load_init_rel_world != None and self.cnt_phase_ticks > self.cnt_threshold_drone_setup:
                    self.cnt_phase_ticks = 0
                    self.phase = Phase.PHASE_SETUP_GCS
                    self.get_logger().info(f'Load setup complete')

                    return

                self.cnt_phase_ticks += 1

            # Wait for GCS to be set up
            case Phase.PHASE_SETUP_GCS:               
                # GCS sets up desired poses of drones relative to load so check if this has been set
                if self.flag_desired_pose_rel_load_set:
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'GCS setup complete')
                    self.get_logger().info(f'SETUP COMPLETE')

                    # Automatically transition to takeoff if in more autonomy than manual mode
                    #if self.auto_level >=1:
                    self.phase = Phase.PHASE_TAKEOFF_START

            # Run takeoff
            case Phase.PHASE_TAKEOFF_START:               
                # Override trajectory msg for straight-up takeoff in first phase
                trajectory_msg = utils.gen_traj_msg_straight_up(self.height_load_pre_tension+self.height_drone_rel_load, self.pixhawk_pose.local_state.att_q, timestamp)

                # Send takeoff setpoint
                self.pub_trajectory.publish(trajectory_msg)

                # Update counter for arm phase
                if self.cnt_phase_ticks <=self.cnt_threshold_takeoff_start:                   
                    self.cnt_phase_ticks += 1

                # Continue to send setpoint whilst taking off
                if self.vehicle_status.arming_state==VehicleStatus.ARMING_STATE_ARMED and self.vehicle_status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    # Takeoff to pre-tension level. Only transition once pre-tension level reached and pose rel load set. Only transition if not in lowest autonomy level
                    if tf_drone_rel_world.transform.translation.z>=(self.height_load_pre_tension+self.height_drone_rel_load-self.pos_threshold) and self.auto_level >=1: 
                        self.phase = Phase.PHASE_TAKEOFF_PRE_TENSION
                        self.cnt_phase_ticks = 0  
                        self.get_logger().info(f'Pre-tension level reached')        

                # Arm vehicle (and switch to offboard mode) when offboard message has been published for long enough, and if not already armed or in offboard mode
                elif self.cnt_phase_ticks >= self.cnt_threshold_takeoff_start:                   
                    # Set in offboard mode
                    # if self.vehicle_status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    #     offboard_ros.engage_offboard_mode(self.pub_vehicle_command, timestamp)

                    # Arm (once in offboard mode)
                    if self.vehicle_status.arming_state!=VehicleStatus.ARMING_STATE_ARMED: #elif
                        offboard_ros.arm(self.pub_vehicle_command, timestamp)


            case Phase.PHASE_TAKEOFF_PRE_TENSION:
                # Send pre-tension setpoint: in formation but slightly below height
                desired_state_rel_load_lower_z = self.vehicle_desired_state_rel_load.copy()
                desired_state_rel_load_lower_z.pos[2] += self.height_load_pre_tension

                # Wait before attempt to pick up load as get into formation
                # Travel at set yawspeed and direction for this, then at no speed setpoint afterwards
                if self.cnt_phase_ticks < self.cnt_threshold_takeoff_pre_tension:
                    self.cnt_phase_ticks += 1
                    trajectory_msg = utils.gen_traj_msg_circle_load(desired_state_rel_load_lower_z, self.load_desired_local_state, self.get_name(), self.tf_buffer, timestamp, self.get_logger(), drone_prev_local_state=self.pixhawk_pose.local_state, yawspeed_scalar=self.yawspeed_drone)
                else:                   
                    if self.auto_level >=1: # Only automatically transition if not in lowest autonomy mode
                        # TODO: Slowly rise to engage tension
                        self.phase = Phase.PHASE_TAKEOFF_POST_TENSION
                        self.cnt_phase_ticks = 0
                        self.get_logger().info(f'Takeoff pre-tension complete') 

                self.pub_trajectory.publish(trajectory_msg)
                     

            case Phase.PHASE_TAKEOFF_POST_TENSION:
                #Takeoff complete
                if tf_drone_rel_world.transform.translation.z>=(self.takeoff_height_load+self.height_drone_rel_load-self.pos_threshold):
                    self.phase = Phase.PHASE_TAKEOFF_END 
                    
                    self.get_logger().info(f'TAKEOFF COMPLETE')

                # Send takeoff setpoint
                self.pub_trajectory.publish(trajectory_msg)

            # Takeoff complete - hover at takeoff end location
            case Phase.PHASE_TAKEOFF_END:
                self.pub_trajectory.publish(trajectory_msg)
                self.cnt_phase_ticks = 0


            # Run main offboard mission
            case Phase.PHASE_MISSION_START:
                self.pub_trajectory.publish(trajectory_msg)
                    

            # Run land 
            case Phase.PHASE_LAND_START:
                if self.cnt_phase_ticks < self.cnt_threshold_land_pre_descent:
                    self.cnt_phase_ticks += 1
                else:
                    self.phase = Phase.PHASE_LAND_DESCENT
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'Land descent beginning') 

                # Send hold setpoint
                self.pub_trajectory.publish(trajectory_msg)
            
            case Phase.PHASE_LAND_DESCENT:
                if tf_drone_rel_world.transform.translation.z<=(self.height_load_pre_tension+self.height_drone_rel_load-self.pos_threshold):
                    self.phase = Phase.PHASE_LAND_POST_LOAD_DOWN
                    self.get_logger().info(f'Load placed on ground') 

                # Send descending setpoint
                self.pub_trajectory.publish(trajectory_msg)


            case Phase.PHASE_LAND_POST_LOAD_DOWN:
                if self.cnt_phase_ticks <self.cnt_threshold_land_post_load_down:
                    self.cnt_phase_ticks += 1
                else: 
                    self.phase = Phase.PHASE_LAND_DRONES
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'Ready to set drones down') 

                # Send spread out setpoint
                self.pub_trajectory.publish(trajectory_msg)


            case Phase.PHASE_LAND_DRONES:
                if self.cnt_phase_ticks <self.cnt_threshold_land_drones:
                    self.cnt_phase_ticks += 1
                else: 
                    self.phase = Phase.PHASE_LAND_END
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'Ready to disarm') 

                    #offboard_ros.land(self.pub_vehicle_command, timestamp) 

                # Send landing setpoint
                self.pub_trajectory.publish(trajectory_msg)


            case Phase.PHASE_LAND_END:
                # Disarm when landed
                if tf_drone_rel_world.transform.translation.z<=0.05:
                    if self.cnt_phase_ticks <self.cnt_threshold_land_pre_disarm:
                        self.cnt_phase_ticks += 1
                    else:
                        #offboard_ros.disengage_offboard_mode(self.pub_vehicle_command, timestamp)
                        #offboard_ros.disarm(self.pub_vehicle_command, timestamp)
                        self.phase = Phase.PHASE_UNASSIGNED
                        self.cnt_phase_ticks = 0

                        self.get_logger().info(f'LANDED AND DISARMED') 
            
            case Phase.PHASE_HOLD:
                # Send setpoint as current position
                self.pub_trajectory.publish(trajectory_msg)
                pass

            # Kill
            case Phase.PHASE_KILL:
                offboard_ros.kill(self.pub_vehicle_command, timestamp)
                self.cnt_phase_ticks = 0

                pass

        # Publish the phase the drone is currently in 
        msg_current_phase = Phase()
        msg_current_phase.phase = self.phase
        self.pub_current_phase.publish(msg_current_phase)
    

    ## HELPER FUNCTIONS
    def reset_pre_arm(self):
        self.pixhawk_pose.reset()
        self.flag_desired_pose_rel_load_set = False

        #self.flag_reset_pre_arm_complete = True # To ensure that this function finishes running before setup is complete
        self.get_logger().info('RESET PRE-ARM COMPLETE')


def main():
    # Create node
    rclpy.init()
    drone = Drone.create(node_name='drone9', namespace='px4_9')
    rclpy.spin(drone)

    # Destroy node
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    