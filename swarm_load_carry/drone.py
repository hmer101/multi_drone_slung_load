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
from swarm_load_carry_interfaces.srv import PhaseChange, GetGlobalInitPose, SetLocalPose # Note must build workspace and restart IDE before custom packages are found by python
from swarm_load_carry_interfaces.msg import Phase

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

MAIN_TIMER_PERIOD=0.2 # sec

TAKEOFF_HEIGHT_LOAD_PRE_TENSION=-0.2
TAKEOFF_HEIGHT_LOAD=3.0 
TAKEOFF_HEIGHT_DRONE_REL_LOAD=1.082

TAKEOFF_START_CNT_THRESHOLD=2/MAIN_TIMER_PERIOD
TAKEOFF_PRE_TENSION_CNT_THRESHOLD=5/MAIN_TIMER_PERIOD
TAKEOFF_POS_THRESHOLD=0.1

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

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value
        self.load_name = f'load{self.load_id}'

        # Vehicle
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX     # Actual mode of the FMU
        self.arm_state = VehicleStatus.ARMING_STATE_MAX         # Actual arming state of FMU
        self.phase = Phase.PHASE_UNASSIGNED        # Desired phase (action to perform when in offboard mode. Use 'phase' to differentiate from 'mode' of the FMU)

        self.vehicle_local_state = State(f'{self.get_name()}_init', CS_type.ENU)

        self.vehicle_initial_global_state = State('globe', CS_type.LLA)
        self.vehicle_initial_state_rel_world = State('world', CS_type.ENU)
        
        self.vehicle_desired_state_rel_load = State(f'{self.load_name}', CS_type.ENU)

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

        ## SERVICES
        self.srv_phase_change = self.create_service(
            PhaseChange,
            f'{self.ns}/phase_change',
            self.clbk_change_phase)
        
        self.srv_get_global_init_pose = False # Only create service once drone is ready to send global initial position (i.e. has set global initial pose)
        
        self.srv_set_pose_rel_load = self.create_service(
            SetLocalPose,
            f'{self.ns}/desired_pose_rel_load',
            self.clbk_set_desired_pose_rel_load)

        ## CLIENTS
        # Set local world frame origin to first drone's initial position
        self.cli_get_first_drone_init_global_pose = self.create_client(GetGlobalInitPose,f'/px4_{self.first_drone_num}/global_initial_pose')

        # Futures
        self.first_drone_init_global_pose_future = None

        ## FLAGS 
        # Ensure set services are called at least once before taking off)
        self.flag_gps_home_set = False # GPS home set when vehicle first armed
        self.flag_local_init_pos_set = False 
        self.flag_desired_pos_rel_load_set = False

        ## SETUP HELPERS
        # Get the first drone's position when ready as it defines the `world' CS
        if not self.drone_id == self.first_drone_num:
            # Otherwise must request first drone's initial position (acted on in main clbk loop)
            while not self.cli_get_first_drone_init_global_pose.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for global initial pose service drone {self.first_drone_num}')

            self.update_first_drone_init_pose() # TODO: Call this periodically incase any FMU restarts and changes init pose


        ## Print information
        self.get_logger().info('DRONE NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')
    

    # Create a node with MAVLINK connections initialized
    @classmethod
    def create(cls, node_name='drone9', namespace='px4_9'):
        # Create node without MAVLINK connections (i.e. only has ROS connections here)
        self = Drone(name=node_name, namespace=namespace)
        self.get_logger().info('Setup complete')

        return self


    ## CALLBACKS
    def clbk_vehicle_status(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

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
        if not (np.isnan(self.vehicle_local_state.pos[0])):
            utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', f'{self.get_name()}', self.vehicle_local_state.pos, self.vehicle_local_state.att_q, self.tf_broadcaster)

    def clbk_vehicle_local_position(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_state.pos[0] = msg.x
        self.vehicle_local_state.pos[1] = -msg.y
        self.vehicle_local_state.pos[2] = -msg.z
        self.vehicle_local_state.vel[0] = msg.vx
        self.vehicle_local_state.vel[1] = -msg.vy
        self.vehicle_local_state.vel[2] = -msg.vz

        # Publish TF
        if not (np.isnan(self.vehicle_local_state.att_q.x)):
            # Update tf
            utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', f'{self.get_name()}', self.vehicle_local_state.pos, self.vehicle_local_state.att_q, self.tf_broadcaster)
     

    def clbk_vehicle_global_position(self, msg):
        # Set GPS/location home immediately prior to first arming/takeoff
        if not self.flag_gps_home_set and (self.cnt_phase_ticks > 1):          
            # Set the initial position as the current global position
            self.vehicle_initial_global_state.lat = msg.lat
            self.vehicle_initial_global_state.lon = msg.lon 
            self.vehicle_initial_global_state.alt = msg.alt

            offboard_ros.set_origin(self.pub_vehicle_command, msg.lat, msg.lon, msg.alt, int(self.get_clock().now().nanoseconds/1000))

            self.flag_gps_home_set = True   

    def clbk_load_desired_attitude(self, msg):
        self.load_desired_local_state.att_q = qt.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])

    def clbk_load_desired_local_position(self, msg):
        self.load_desired_local_state.pos = np.array([msg.x, msg.y, msg.z])
    
    def clbk_set_desired_pose_rel_load(self, request, response):
        self.vehicle_desired_state_rel_load.pos = np.array([request.transform_stamped.transform.translation.x, request.transform_stamped.transform.translation.y, request.transform_stamped.transform.translation.z])
        self.vehicle_desired_state_rel_load.att_q = qt.array([request.transform_stamped.transform.rotation.w, request.transform_stamped.transform.rotation.x, request.transform_stamped.transform.rotation.y, request.transform_stamped.transform.rotation.z])

        self.flag_desired_pos_rel_load_set = True
        response.success = True

        return response


    def clbk_send_global_init_pose(self, request, response):
        response.global_pos.lat = float(self.vehicle_initial_global_state.lat)
        response.global_pos.lon = float(self.vehicle_initial_global_state.lon)
        response.global_pos.alt = float(self.vehicle_initial_global_state.alt)

        response.global_att.q[0] = self.vehicle_initial_global_state.att_q.w
        response.global_att.q[1] = self.vehicle_initial_global_state.att_q.x
        response.global_att.q[2] = self.vehicle_initial_global_state.att_q.y
        response.global_att.q[3] = self.vehicle_initial_global_state.att_q.z
                                      
        return response


    def clbk_change_phase(self, request, response):

        # Call helper functions if required
        match request.phase_request.phase:
            case Phase.PHASE_TAKEOFF_START:
                # Takeoff
                self.phase=Phase.PHASE_TAKEOFF_START

            case Phase.PHASE_MISSION_START:
                self.phase=Phase.PHASE_MISSION_START
            
            case Phase.PHASE_LAND_START:
                self.phase=Phase.PHASE_LAND_START

            case Phase.PHASE_HOLD:
                self.phase=Phase.PHASE_HOLD

                #self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.hold())

            case Phase.PHASE_KILL:
                self.phase=Phase.PHASE_KILL

                #self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.kill())


        response.success = True
        self.get_logger().info(f'Requested change to phase: {self.phase}')

        return response


    def clbk_cmdloop(self):       
        # Continually publish offboard mode heartbeat (note need setpoint published too to stay in offboard mode)
        timestamp = int(self.get_clock().now().nanoseconds/1000)
        offboard_ros.publish_offboard_control_heartbeat_signal(self.pub_offboard_mode, 'pos', timestamp)

        # Run rest of setup when ready and not already setup
        if self.flag_gps_home_set and not self.flag_local_init_pos_set:
            # Rest of setup differs for first drone and others
            if self.drone_id == self.first_drone_num:
                self.set_local_init_pose_first_drone()
            else:
                # Set init poses once the first drone's initial position has been received
                if self.first_drone_init_global_pose_future.done():
                    self.set_local_init_pose_later_drones()
        
        # Get TF relative to world if the world position is set
        if self.flag_local_init_pos_set:
            # Get actual position feedback 
            # Start with drone position (TODO: incorporate load position feedback later)
            tf_drone_rel_world = utils.lookup_tf('world', self.get_name(), self.tf_buffer, rclpy.time.Time(), self.get_logger())

        # Generate trajectory message
        trajectory_msg = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, self.load_desired_local_state, self.get_name(), self.tf_buffer, timestamp, self.get_logger()) #load_takeoff_state
        
        # Perform actions depending on what mode is requested
        match self.phase:
            # Run takeoff
            # TODO: Add some formation feedback to all phases (especially mission)
            case Phase.PHASE_TAKEOFF_START:
                # Override trajectory msg for straight-up takeoff in first phase
                trajectory_msg = utils.gen_traj_msg_straight_up(TAKEOFF_HEIGHT_LOAD_PRE_TENSION+TAKEOFF_HEIGHT_DRONE_REL_LOAD, self.vehicle_local_state.att_q, timestamp)
                
                # Update counter for arm phase
                if self.cnt_phase_ticks <=TAKEOFF_START_CNT_THRESHOLD: 
                    # Send takeoff setpoint
                    self.pub_trajectory.publish(trajectory_msg)

                    self.cnt_phase_ticks += 1

                # Arm vehicle when offboard message has been published for long enough
                if self.cnt_phase_ticks == TAKEOFF_START_CNT_THRESHOLD:                   
                    # Arm
                    offboard_ros.arm(self.pub_vehicle_command, timestamp)

                    # Set in offboard mode
                    offboard_ros.engage_offboard_mode(self.pub_vehicle_command, timestamp)

                # Continue to send setpoint whilst taking off
                elif self.nav_state ==VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arm_state==VehicleStatus.ARMING_STATE_ARMED: 
                    # Takeoff to pre-tension level
                    if tf_drone_rel_world.transform.translation.z>=(TAKEOFF_HEIGHT_LOAD_PRE_TENSION+TAKEOFF_HEIGHT_DRONE_REL_LOAD-TAKEOFF_POS_THRESHOLD):     
                        self.phase = Phase.PHASE_TAKEOFF_PRE_TENSION
                        self.cnt_phase_ticks = 0  
                        self.get_logger().info(f'PRE_TENSION LEVEL REACHED')        
                        
                    # Send takeoff setpoint
                    self.pub_trajectory.publish(trajectory_msg)


            case Phase.PHASE_TAKEOFF_PRE_TENSION:
                # Wait before attempt to pick up load as get into formation
                if self.cnt_phase_ticks < TAKEOFF_PRE_TENSION_CNT_THRESHOLD:
                    self.cnt_phase_ticks += 1
                else:
                    self.phase = Phase.PHASE_TAKEOFF_POST_TENSION
                    self.cnt_phase_ticks = 0
                    self.get_logger().info(f'PRE_TENSION LEVEL COMPLETE') 
                
                # Send takeoff setpoint
                self.pub_trajectory.publish(trajectory_msg)
            

            case Phase.PHASE_TAKEOFF_POST_TENSION:
                #Takeoff complete
                if tf_drone_rel_world.transform.translation.z>=(TAKEOFF_HEIGHT_LOAD+TAKEOFF_HEIGHT_DRONE_REL_LOAD-TAKEOFF_POS_THRESHOLD):     
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


            # Run land 
            case Phase.PHASE_LAND_START:
                
                self.phase = Phase.PHASE_LAND_END 

                # TODO: Add some formation feedback

            # Run RTL 
            # case Phase.PHASE_RTL_START:

            #     self.phase = Phase.PHASE_RTL_END

            #     # TODO: Add some formation feedback

            # Hold 
            # case Phase.PHASE_HOLD:
            #     pass

            # Kill
            case Phase.PHASE_KILL:
                offboard_ros.kill(self.pub_vehicle_command, timestamp)

        # Publish the phase the drone is currently in 
        msg_current_phase = Phase()
        msg_current_phase.phase = self.phase
        self.pub_current_phase.publish(msg_current_phase)

        #self.get_logger().info(f'Phase: {self.phase}')
    

    ## HELPER FUNCTIONS
    def update_first_drone_init_pose(self):
        # Request updated init pose of first drone
        self.first_drone_init_global_pose_future = self.cli_get_first_drone_init_global_pose.call_async(GetGlobalInitPose.Request())

        # Set flag to reset other local init poses
        self.flag_local_init_pos_set = False

    def broadcast_tf_init_pose(self):
        # Publish static transform for init pose (relative to world)
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'world', f'drone{self.drone_id}_init', self.vehicle_initial_state_rel_world.pos, self.vehicle_initial_state_rel_world.att_q, self.tf_static_broadcaster_init_pose)

        self.flag_local_init_pos_set = True 
        self.get_logger().info('Local init pos set')

    def get_origin_pose(self):
        # Return result in state format
        global_origin_pose = self.first_drone_init_global_pose_future.result()
        global_origin_state = State('globe', CS_type.LLA)

        global_origin_state.lat = global_origin_pose.global_pos.lat
        global_origin_state.lon = global_origin_pose.global_pos.lon 
        global_origin_state.alt = global_origin_pose.global_pos.alt
        
        global_origin_state.att_q = qt.array([global_origin_pose.global_att.q[0], global_origin_pose.global_att.q[1], global_origin_pose.global_att.q[2], global_origin_pose.global_att.q[3]]) 

        return global_origin_state

    def set_local_init_pose_later_drones(self):
        ## Set initial pose relative to first drone's initial pose
        origin_state_lla = self.get_origin_pose()

        # Perform transformation
        trans_E, trans_N, trans_U = pm.geodetic2enu(self.vehicle_initial_global_state.lat, self.vehicle_initial_global_state.lon, self.vehicle_initial_global_state.alt, origin_state_lla.lat, origin_state_lla.lon, origin_state_lla.alt) 
        
        # Set local init pose (relative to base CS)
        self.vehicle_initial_state_rel_world.pos = np.array([trans_E, trans_N, trans_U])
        self.vehicle_initial_state_rel_world.att_q = self.vehicle_initial_global_state.att_q.copy() #TODO: If required to set relative to world, need extra flag to ensure inital att is set before sending global init pose
        #(1/self.vehicle_initial_global_state.att_q) * origin_state_lla.att_q #  #qt.distance.rotation.intrinsic() # #qt.array([1.0, 0.0, 0.0, 0.0]) 

        # Broadcast tf
        self.broadcast_tf_init_pose()

    def set_local_init_pose_first_drone(self):
        # Set local initial state
        self.vehicle_initial_state_rel_world.pos = np.array([0.0, 0.0, 0.0])
        self.vehicle_initial_state_rel_world.att_q = self.vehicle_initial_global_state.att_q.copy()
        
        # Create global initial pose service to allow other nodes to receive this drone's initial pose
        self.srv_get_global_init_pose = self.create_service(
                                            GetGlobalInitPose,
                                            f'{self.ns}/global_initial_pose',
                                            self.clbk_send_global_init_pose)

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