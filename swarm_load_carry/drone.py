# Contains the drone class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import asyncio, rclpy, utils # Note import utils needs additions to setup.py. See here: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
import swarm_load_carry.drone_offboard_ros as offboard_ros
import numpy as np
import pymap3d as pm
import quaternionic as qt

import frame_transforms as ft

from swarm_load_carry.state import State, CS_type

import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, VehicleGlobalPosition
from swarm_load_carry_interfaces.srv import PhaseChange, SetLocalPose # Note must build workspace and restart IDE before custom packages are found by python
from swarm_load_carry_interfaces.msg import Phase, GlobalPose

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1
DEFAULT_FULLY_AUTO=False

MAIN_TIMER_PERIOD=0.1 # sec

HEIGHT_DRONE_REL_LOAD=2 # m
HEIGHT_LOAD_PRE_TENSION=-0.2
POS_THRESHOLD=0.3 #0.1

TAKEOFF_HEIGHT_LOAD=3.0 

SETUP_CNT_THRESHOLD=5/MAIN_TIMER_PERIOD

TAKEOFF_START_CNT_THRESHOLD=3/MAIN_TIMER_PERIOD
TAKEOFF_PRE_TENSION_CNT_THRESHOLD=5/MAIN_TIMER_PERIOD

LAND_PRE_DESCENT_CNT_THRESHOLD=5/MAIN_TIMER_PERIOD
LAND_POST_LOAD_DOWN_CNT_THRESHOLD=5/MAIN_TIMER_PERIOD
LAND_PRE_DISARM_CNT_THRESHOLD=3/MAIN_TIMER_PERIOD

FULLY_AUTO_PRE_TAKEOFF_CNT_THRESHOLD=5/MAIN_TIMER_PERIOD


# Node to encapsulate drone information and actions
class Drone(Node):

    ## Initialization
    def __init__(self, name, namespace):
        super().__init__(node_name=name, namespace=namespace)

        self.ns = self.get_namespace()
        self.drone_id = int(str(self.ns)[-1])

        # Parameters
        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)
        self.declare_parameter('load_id', DEFAULT_LOAD_ID)
        self.declare_parameter('fully_auto', DEFAULT_FULLY_AUTO)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.fully_auto = self.get_parameter('fully_auto').get_parameter_value().bool_value
        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value
        self.load_name = f'load{self.load_id}'

        # Vehicle
        self.vehicle_status = None  # Current state of the FMU

        self.phase = Phase.PHASE_UNASSIGNED        # Desired phase (action to perform when in offboard mode. Use 'phase' to differentiate from 'mode' of the FMU)

        self.vehicle_local_state = State(f'{self.get_name()}_init', CS_type.ENU)

        self.vehicle_initial_global_state = State('globe', CS_type.LLA)
        self.vehicle_initial_state_rel_world = State('world', CS_type.ENU)
        
        self.vehicle_desired_state_rel_load = State(f'{self.load_name}', CS_type.ENU)

        # Other vehicles
        self.global_origin_state = State('globe', CS_type.LLA)
        self.global_origin_state_prev = self.global_origin_state.copy()

        # Load
        self.load_desired_local_state = State(f'{self.load_name}_init', CS_type.ENU)

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        ## TIMERS
        timer_period = MAIN_TIMER_PERIOD
        self.timer = self.create_timer(timer_period, self.clbk_cmdloop)
        self.cnt_phase_ticks = 0

        ### ROS2
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ## TFS
        self.tf_static_broadcaster_init_pose = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        
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
            self.clbk_vehicle_attitude,
            qos_profile)

        self.sub_local_position = self.create_subscription(
            VehicleLocalPosition,
            f'{self.ns}/fmu/out/vehicle_local_position',
            self.clbk_vehicle_local_position,
            qos_profile)  
        
        self.sub_global_position = self.create_subscription(
            VehicleGlobalPosition,
            f'{self.ns}/fmu/out/vehicle_global_position',
            self.clbk_vehicle_global_position,
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
                self.clbk_global_origin,
                qos_profile)

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
        self.flag_gps_home_set = False # GPS home set when vehicle armed
        self.flag_local_init_pose_set = False 
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

    def clbk_vehicle_attitude(self, msg):
        # Original q from FRD->NED
        # Handles FRD->NED to FLU->ENU transformation 
        q_px4 = utils.q_to_normalized_np(qt.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]]))
        q_ros = ft.px4_to_ros_orientation(q_px4)

        self.vehicle_local_state.att_q.w = q_ros[0]
        self.vehicle_local_state.att_q.x = q_ros[1] 
        self.vehicle_local_state.att_q.y = q_ros[2]   
        self.vehicle_local_state.att_q.z = q_ros[3] 
        

        if not self.flag_gps_home_set:
            # Set the initial attitude as the current attitude
            self.vehicle_initial_global_state.att_q = self.vehicle_local_state.att_q.copy()

        # Update tf
        # if not (np.isnan(self.vehicle_local_state.pos[0])):
        #     utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', f'{self.get_name()}', self.vehicle_local_state.pos, self.vehicle_local_state.att_q, self.tf_broadcaster)

    def clbk_vehicle_local_position(self, msg):
        # Handles NED->ENU transformation 
        self.vehicle_local_state.pos[0] = msg.y 
        self.vehicle_local_state.pos[1] = msg.x 
        self.vehicle_local_state.pos[2] = -msg.z
        self.vehicle_local_state.vel[0] = msg.vy 
        self.vehicle_local_state.vel[1] = msg.vx 
        self.vehicle_local_state.vel[2] = -msg.vz

        # Publish TF
        if not (np.isnan(self.vehicle_local_state.att_q.x)):
            # Update tf
            utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', f'{self.get_name()}', self.vehicle_local_state.pos, self.vehicle_local_state.att_q, self.tf_broadcaster)
     

    def clbk_vehicle_global_position(self, msg):
        # Set GPS/location home immediately prior to first arming/takeoff
        if not self.flag_gps_home_set and (self.phase == Phase.PHASE_SETUP):          
            # Set the initial position as the current global position 
            # (could average over last few samples but GPS seems to have low frequency noise of about 0.1m -> can turn down in simulation if required)
            self.vehicle_initial_global_state.pos[0] = msg.lat
            self.vehicle_initial_global_state.pos[1] = msg.lon 
            self.vehicle_initial_global_state.pos[2] = msg.alt

            offboard_ros.set_origin(self.pub_vehicle_command, msg.lat, msg.lon, msg.alt, int(self.get_clock().now().nanoseconds/1000))

            # Publish this information
            msg_global_pose = GlobalPose()

            msg_global_pose.global_pos.lat = float(self.vehicle_initial_global_state.pos[0])
            msg_global_pose.global_pos.lon = float(self.vehicle_initial_global_state.pos[1])
            msg_global_pose.global_pos.alt = float(self.vehicle_initial_global_state.pos[2])

            msg_global_pose.global_att.q[0] = float(self.vehicle_initial_global_state.att_q.w)
            msg_global_pose.global_att.q[1] = float(self.vehicle_initial_global_state.att_q.x)
            msg_global_pose.global_att.q[2] = float(self.vehicle_initial_global_state.att_q.y)
            msg_global_pose.global_att.q[3] = float(self.vehicle_initial_global_state.att_q.z)

            self.pub_global_init_pose.publish(msg_global_pose)

            self.flag_gps_home_set = True   

    def clbk_load_desired_attitude(self, msg):
        self.load_desired_local_state.att_q = qt.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])

    def clbk_load_desired_local_position(self, msg):
        self.load_desired_local_state.pos = np.array([msg.x, msg.y, msg.z])
    
    def clbk_global_origin(self, msg):
        self.global_origin_state.pos[0] = msg.global_pos.lat
        self.global_origin_state.pos[1] = msg.global_pos.lon
        self.global_origin_state.pos[2] = msg.global_pos.alt

        self.global_origin_state.att_q.w = msg.global_att.q[0]
        self.global_origin_state.att_q.x = msg.global_att.q[1]
        self.global_origin_state.att_q.y = msg.global_att.q[2]
        self.global_origin_state.att_q.z = msg.global_att.q[3]

        # Global origin updated - must update local initial poses
        self.flag_local_init_pose_set = False
        

    def clbk_set_desired_pose_rel_load(self, request, response):
        self.vehicle_desired_state_rel_load.pos = np.array([request.transform_stamped.transform.translation.x, request.transform_stamped.transform.translation.y, request.transform_stamped.transform.translation.z])
        self.vehicle_desired_state_rel_load.att_q = qt.array([request.transform_stamped.transform.rotation.w, request.transform_stamped.transform.rotation.x, request.transform_stamped.transform.rotation.y, request.transform_stamped.transform.rotation.z])

        self.flag_desired_pose_rel_load_set = True
        response.success = True

        return response


    def clbk_change_phase(self, request, response):

        # Call helper functions if required
        match request.phase_request.phase:
            case Phase.PHASE_SETUP:
                # Takeoff
                self.phase=Phase.PHASE_SETUP

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
        # Continually publish offboard mode heartbeat (note need setpoint published too to stay in offboard mode)
        timestamp = int(self.get_clock().now().nanoseconds/1000)
        offboard_ros.publish_offboard_control_heartbeat_signal(self.pub_offboard_mode, 'pos', timestamp)
        
        # Get TF relative to world if the world position is set
        if self.flag_local_init_pose_set:
            # Get actual position feedback 
            # Start with drone position (TODO: incorporate load position feedback later)
            tf_drone_rel_world = utils.lookup_tf('world', self.get_name(), self.tf_buffer, rclpy.time.Time(), self.get_logger())


        # Start setup and take-off automatically if in fully-auto mode. 
        if self.fully_auto and self.phase == Phase.PHASE_UNASSIGNED and self.vehicle_status is not None:
            # Counter starts when attempt to put into offboard mode by RC
            if self.vehicle_status.nav_state_user_intention == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if self.cnt_phase_ticks >= FULLY_AUTO_PRE_TAKEOFF_CNT_THRESHOLD:
                    self.phase = Phase.PHASE_SETUP
                    self.cnt_phase_ticks = 0

                else:
                    self.cnt_phase_ticks += 1

        # Generate trajectory message for formation used after take-off. #TODO: utils.gen_traj_msg_circle_load runs slowly and causes cut-out
        elif self.phase > Phase.PHASE_TAKEOFF_START:
            trajectory_msg = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, self.load_desired_local_state, self.get_name(), self.tf_buffer, timestamp, self.get_logger())

            if trajectory_msg == None:
                self.get_logger().warn(f'Load or drone initial position not found. Skipping this command loop.')
                return


        # Perform actions depending on what mode is requested
        # TODO: Add some formation feedback to all phases (especially mission)
        match self.phase:
            # Run origin-altering setup pre-arming
            case Phase.PHASE_SETUP:
                # If drone is armed, setup has already been performed. Skip straight to takeoff phase
                if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.cnt_phase_ticks = 0
                    self.phase = Phase.PHASE_TAKEOFF_PRE_TENSION
                    self.get_logger().info(f'Vehicle already setup. Skipping to takeoff phase.')

                    return
                
                # Check if load's setup is complete
                tf_load_init_rel_world = utils.lookup_tf('world', f'{self.load_name}_init', self.tf_buffer, rclpy.time.Time(), self.get_logger())
                
                # Reset FMU and TF home positions
                if self.cnt_phase_ticks == 0:
                    self.reset_pre_arm()

                # Set initial poses when ready
                if self.flag_gps_home_set and not self.flag_local_init_pose_set:                    
                    # Rest of setup differs for first drone and others
                    if self.drone_id == self.first_drone_num:
                        self.set_local_init_pose_first_drone()

                    else:
                        # Set init poses once the first drone's initial position has been received
                        if self.global_origin_state != self.global_origin_state_prev:
                            self.set_local_init_pose_later_drones()
                            self.global_origin_state_prev = self.global_origin_state.copy()
                
                # Exit setup only once drone's GPS home and initial positions have been set, 
                # the drone's desired pose relative to the load has been set and the load's initial pose has been set
                elif self.flag_gps_home_set and self.flag_local_init_pose_set and self.flag_desired_pose_rel_load_set and (tf_load_init_rel_world != None):
                    self.cnt_phase_ticks = 0
                    self.phase = Phase.PHASE_TAKEOFF_START
                    self.get_logger().info(f'SETUP COMPLETE')

                    return
                        
                self.cnt_phase_ticks += 1
                
            # Run takeoff
            case Phase.PHASE_TAKEOFF_START:               
                # Override trajectory msg for straight-up takeoff in first phase
                trajectory_msg = utils.gen_traj_msg_straight_up(HEIGHT_LOAD_PRE_TENSION+HEIGHT_DRONE_REL_LOAD, self.vehicle_local_state.att_q, timestamp)

                # Send takeoff setpoint
                self.pub_trajectory.publish(trajectory_msg)

                # Update counter for arm phase
                if self.cnt_phase_ticks <=TAKEOFF_START_CNT_THRESHOLD:                   
                    self.cnt_phase_ticks += 1

                # Continue to send setpoint whilst taking off
                if self.vehicle_status.arming_state==VehicleStatus.ARMING_STATE_ARMED and self.vehicle_status.nav_state==VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    # Takeoff to pre-tension level. Only transition once pre-tension level reached and pose rel load set
                    if tf_drone_rel_world.transform.translation.z>=(HEIGHT_LOAD_PRE_TENSION+HEIGHT_DRONE_REL_LOAD-POS_THRESHOLD): # and self.flag_desired_pose_rel_load_set:     
                        self.phase = Phase.PHASE_TAKEOFF_PRE_TENSION
                        self.cnt_phase_ticks = 0  
                        self.get_logger().info(f'PRE_TENSION LEVEL REACHED')        

                # Arm vehicle (and switch to offboard mode) when offboard message has been published for long enough, and if not already armed or in offboard mode
                elif self.cnt_phase_ticks >= TAKEOFF_START_CNT_THRESHOLD:                   
                    # Set in offboard mode
                    if self.vehicle_status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                        offboard_ros.engage_offboard_mode(self.pub_vehicle_command, timestamp)

                    # Arm (once in offboard mode)
                    elif self.vehicle_status.arming_state!=VehicleStatus.ARMING_STATE_ARMED:
                        offboard_ros.arm(self.pub_vehicle_command, timestamp)


            case Phase.PHASE_TAKEOFF_PRE_TENSION:
                # Send takeoff setpoint
                self.pub_trajectory.publish(trajectory_msg)

                # Wait before attempt to pick up load as get into formation
                if self.cnt_phase_ticks < TAKEOFF_PRE_TENSION_CNT_THRESHOLD:
                    self.cnt_phase_ticks += 1
                else:
                    self.phase = Phase.PHASE_TAKEOFF_POST_TENSION
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'PRE_TENSION LEVEL COMPLETE') 
                     

            case Phase.PHASE_TAKEOFF_POST_TENSION:
                #Takeoff complete
                if tf_drone_rel_world.transform.translation.z>=(TAKEOFF_HEIGHT_LOAD+HEIGHT_DRONE_REL_LOAD-POS_THRESHOLD):     
                    self.phase = Phase.PHASE_TAKEOFF_END 
                    
                    self.get_logger().info(f'Takeoff complete')

                # Send takeoff setpoint
                self.pub_trajectory.publish(trajectory_msg)

            # Takeoff complete - hover at takeoff end location
            case Phase.PHASE_TAKEOFF_END:
                self.pub_trajectory.publish(trajectory_msg)
                self.cnt_phase_ticks = 0


            # Run main offboard mission
            case Phase.PHASE_MISSION_START:
                self.pub_trajectory.publish(trajectory_msg)
                self.cnt_phase_ticks += 1
                    

            # Run land 
            case Phase.PHASE_LAND_START:
                if self.cnt_phase_ticks < LAND_PRE_DESCENT_CNT_THRESHOLD:
                    self.cnt_phase_ticks += 1
                else:
                    self.phase = Phase.PHASE_LAND_DESCENT
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'LAND DESCENT BEGINNING') 

                # Send hold setpoint
                self.pub_trajectory.publish(trajectory_msg)
            
            case Phase.PHASE_LAND_DESCENT:
                if tf_drone_rel_world.transform.translation.z<=(HEIGHT_LOAD_PRE_TENSION+HEIGHT_DRONE_REL_LOAD-POS_THRESHOLD):
                    self.phase = Phase.PHASE_LAND_POST_LOAD_DOWN
                    self.get_logger().info(f'LOAD PLACED ON GROUND') 

                # Send descending setpoint
                self.pub_trajectory.publish(trajectory_msg)


            case Phase.PHASE_LAND_POST_LOAD_DOWN:
                if self.cnt_phase_ticks <LAND_POST_LOAD_DOWN_CNT_THRESHOLD:
                    self.cnt_phase_ticks += 1
                else: 
                    self.phase = Phase.PHASE_LAND_END
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'READY TO SET DRONES DOWN') 

                    offboard_ros.land(self.pub_vehicle_command, timestamp) 

                # Send spread out setpoint
                self.pub_trajectory.publish(trajectory_msg)


            case Phase.PHASE_LAND_END:
                # Disarm when landed
                if tf_drone_rel_world.transform.translation.z<=0.05:
                    if self.cnt_phase_ticks <LAND_PRE_DISARM_CNT_THRESHOLD:
                        self.cnt_phase_ticks += 1
                    else:
                        offboard_ros.disarm(self.pub_vehicle_command, timestamp)
                        offboard_ros.disengage_offboard_mode(self.pub_vehicle_command, timestamp)
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
        self.flag_gps_home_set = False
        self.flag_local_init_pose_set = False
        self.flag_desired_pose_rel_load_set = False


    def broadcast_tf_init_pose(self):
        # Publish static transform for init pose (relative to world)
        # As all init CS are in ENU, they are all aligned in orientation
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'world', f'drone{self.drone_id}_init', self.vehicle_initial_state_rel_world.pos, qt.array([1.0, 0.0, 0.0, 0.0]), self.tf_static_broadcaster_init_pose)

        self.flag_local_init_pose_set = True 
        self.get_logger().info('Local init pose set')

    def set_local_init_pose_later_drones(self):
        ## Set initial pose relative to first drone's initial pose
        origin_state_lla = self.global_origin_state 

        # Perform transformation
        trans_E, trans_N, trans_U = pm.geodetic2enu(self.vehicle_initial_global_state.pos[0], self.vehicle_initial_global_state.pos[1], self.vehicle_initial_global_state.pos[2], origin_state_lla.pos[0], origin_state_lla.pos[1], origin_state_lla.pos[2]) 
        
        # Set local init pose (relative to base CS)
        self.vehicle_initial_state_rel_world.pos = np.array([trans_E, trans_N, trans_U])
        self.vehicle_initial_state_rel_world.att_q = self.vehicle_initial_global_state.att_q.copy() #TODO: If required to set relative to world, need extra flag to ensure inital att is set before sending global init pose

        # Broadcast tf
        self.broadcast_tf_init_pose()

    def set_local_init_pose_first_drone(self):
        # Set local initial state
        self.vehicle_initial_state_rel_world.pos = np.array([0.0, 0.0, 0.0])
        self.vehicle_initial_state_rel_world.att_q = self.vehicle_initial_global_state.att_q.copy()

        # Broadcast tf
        self.broadcast_tf_init_pose()


     

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