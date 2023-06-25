# Contains the drone class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import asyncio, rclpy, utils # Note import utils needs additions to setup.py. See here: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
import swarm_load_carry.drone_offboard_ros as offboard_ros
import numpy as np
import pymap3d as pm
import quaternionic as qt
import time

from swarm_load_carry.state import State, CS_type

from mavsdk import System, offboard, telemetry

import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.task import Future

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, VehicleGlobalPosition
from swarm_load_carry_interfaces.srv import ModeChange, GetGlobalInitPose, SetLocalPose # Note must build workspace and restart IDE before custom packages are found by python

from concurrent.futures import ThreadPoolExecutor


DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

#TAKEOFF_HEIGHT_LOAD=2.0
TAKEOFF_HEIGHT_DRONE=5.0
TAKEOFF_CNT_THRESHOLD=10

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
        self.mode = ModeChange.Request.MODE_UNASSIGNED          # Desired mode

        self.vehicle_local_state = State(f'{self.get_name()}_init', CS_type.ENU)

        self.vehicle_initial_global_state = State('globe', CS_type.LLA)
        self.vehicle_initial_state_rel_world = State('world', CS_type.ENU)
        
        self.vehicle_desired_state_rel_load = State(f'{self.load_name}', CS_type.ENU)

        # Load
        self.load_desired_state = State(f'world', CS_type.ENU)

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # For MAVLINK connection
        self.drone_system = None
        self.msg_future_return = None
        self.async_loop = asyncio.get_event_loop()

        
        ## TIMERS
        timer_period = 0.1 #0.02  # seconds
        self.timer = self.create_timer(timer_period, self.clbk_cmdloop)
        self.offboard_setpoint_counter = 0

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
        
        # self.sub_global_position = self.create_subscription(
        #     VehicleGlobalPosition, #SensorGps,
        #     f'{self.ns}/fmu/out/vehicle_global_position', #f'{self.ns}/fmu/out/vehicle_gps_position',
        #     self.clbk_vehicle_global_position,
        #     qos_profile) 

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
    

        # TODO: Sub to other drones for distributed control!

        ## SERVICES
        self.srv_mode_change = self.create_service(
            ModeChange,
            f'{self.ns}/mode_change',
            self.clbk_change_mode_ros)
        
        self.srv_get_global_init_pose = self.create_service(
            GetGlobalInitPose,
            f'{self.ns}/global_initial_pose',
            self.clbk_send_global_init_pose) #False # Only create service once drone is ready to send global initial position 
        
        # self.srv_set_local_init_pose = self.create_service(
        #     SetLocalPose,
        #     f'{self.ns}/local_initial_pose',
        #     self.clbk_set_local_init_pose)
        
        self.srv_set_pose_rel_load = self.create_service(
            SetLocalPose,
            f'{self.ns}/desired_pose_rel_load',
            self.clbk_set_desired_pose_rel_load)

        ## CLIENTS
        # Set local world frame origin to first drone's initial position
        self.cli_get_first_drone_init_global_pose = self.create_client(GetGlobalInitPose,f'/px4_{self.first_drone_num}/global_initial_pose')

        ## FLAGS 
        # Ensure set services are called at least once before taking off)
        self.flag_gps_home_set = False # GPS home set when vehicle first armed
        self.flag_local_init_pos_set = False 
        self.flag_desired_pos_rel_load_set = False

        ## SETUP HELPERS
        self.set_local_init_pose() #TODO: Test for multi-drone

        ## Print information
        self.get_logger().info('DRONE NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')
        #self.get_logger().info(f'ID: {self.drone_id}')
    

    # Create a node with MAVLINK connections initialized
    @classmethod
    def create(cls, node_name='drone9', namespace='px4_9', msg_future_return=None): #async
        # Create node without MAVLINK connections (i.e. only has ROS connections here)
        self = Drone(name=node_name, namespace=namespace)
        
        
        # Connect MAVLINK
        # await self.connect_mavlink()
        # self.msg_future_return = msg_future_returnself.get_logger().info(f'Height: {load_z}')

        self.get_logger().info('Setup complete')

        return self


    ## CALLBACKS
    def clbk_vehicle_status(self, msg):
        # TODO: handle NED->ENU transformation
        #self.get_logger().info(f'NAV_STATUS: {msg.nav_state}')
        #self.get_logger().info(f'  - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}')
        #self.get_logger().info(f'Desired pose relative to load: {self.vehicle_desired_state_rel_load.pos}, {self.vehicle_desired_state_rel_load.att_q}')
        self.nav_state = msg.nav_state

    def clbk_vehicle_attitude(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_state.att_q.x = msg.q[1] #msg.q[0]
        self.vehicle_local_state.att_q.y = -msg.q[2] #msg.q[1]
        self.vehicle_local_state.att_q.z = -msg.q[3] #-msg.q[2]
        self.vehicle_local_state.att_q.w = msg.q[0] #-msg.q[3]

        # Update tf
        utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', f'{self.get_name()}', self.vehicle_local_state.pos, self.vehicle_local_state.att_q, self.tf_broadcaster)

    def clbk_vehicle_local_position(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_state.pos[0] = msg.x
        self.vehicle_local_state.pos[1] = -msg.y
        self.vehicle_local_state.pos[2] = -msg.z
        self.vehicle_local_state.vel[0] = msg.vx
        self.vehicle_local_state.vel[1] = -msg.vy
        self.vehicle_local_state.vel[2] = -msg.vz

        # Update tf
        utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', f'{self.get_name()}', self.vehicle_local_state.pos, self.vehicle_local_state.att_q, self.tf_broadcaster)
        #self.get_logger().info(f'ref: {msg.ref_lat, msg.ref_lon, msg.ref_alt}')
        #self.get_logger().info(f'vehicle_local_state.pos: {self.vehicle_local_state.pos[0], self.vehicle_local_state.pos[1], self.vehicle_local_state.pos[2]}')

        # Set GPS home 
        if not self.flag_gps_home_set:
            self.vehicle_initial_global_state.pos[0] = msg.ref_lat
            self.vehicle_initial_global_state.pos[1] = msg.ref_lon
            self.vehicle_initial_global_state.pos[2] = msg.ref_alt

            self.flag_gps_home_set = True


    # def clbk_vehicle_global_position(self, msg):
    #     # Update the GPS home position until it is set (at first arming)
    #     if not self.flag_gps_home_set:
    #         self.vehicle_initial_global_state.pos = np.array([msg.lat, msg.lon, msg.alt])

    #     self.get_logger().info(f'global_pos: {msg.lat, msg.lon, msg.alt}')

        


    def clbk_load_desired_attitude(self, msg):
        self.load_desired_state.att_q = qt.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])

    def clbk_load_desired_local_position(self, msg):
        self.load_desired_state.pos = np.array([msg.x, msg.y, msg.z])
    
    def clbk_set_desired_pose_rel_load(self, request, response):
        self.vehicle_desired_state_rel_load.pos = np.array([request.transform_stamped.transform.translation.x, request.transform_stamped.transform.translation.y, request.transform_stamped.transform.translation.z])
        self.vehicle_desired_state_rel_load.att_q = qt.array([request.transform_stamped.transform.rotation.w, request.transform_stamped.transform.rotation.x, request.transform_stamped.transform.rotation.y, request.transform_stamped.transform.rotation.z])

        self.flag_desired_pos_rel_load_set = True
        response.success = True

        return response


    def clbk_send_global_init_pose(self, request, response):
        response.global_pos.lat = float(self.vehicle_initial_global_state.pos[0])
        response.global_pos.lon = float(self.vehicle_initial_global_state.pos[1])
        response.global_pos.alt = float(self.vehicle_initial_global_state.pos[2])

        return response

    # def clbk_set_local_init_pose(self, request, response):
    #     # Set local init pose (relative to base CS)
    #     self.vehicle_initial_state_rel_world.pos = np.array([request.transform_stamped.transform.translation.x, request.transform_stamped.transform.translation.y, request.transform_stamped.transform.translation.z])
    #     self.vehicle_initial_state_rel_world.att_q = qt.array([request.transform_stamped.transform.rotation.w, request.transform_stamped.transform.rotation.x, request.transform_stamped.transform.rotation.y, request.transform_stamped.transform.rotation.z])
        
    #     # Publish static transform for init pose (relative to world)
    #     utils.broadcast_tf(self.get_clock().now().to_msg(), request.transform_stamped.header.frame_id, request.transform_stamped.child_frame_id, self.vehicle_initial_state_rel_world.pos, self.vehicle_initial_state_rel_world.att_q, self.tf_static_broadcaster_init_pose)

    #     self.flag_local_init_pos_set = True 
    #     response.success = True
    #     return response


    def clbk_change_mode_ros(self, request, response):
        self.mode = request.mode

        # Call helper functions if required
        match self.mode:
            case ModeChange.Request.MODE_TAKEOFF_START:
                # Takeoff
                self.mode=ModeChange.Request.MODE_TAKEOFF_START

            # case ModeChange.Request.MODE_TAKEOFF_MAV_END:
            #     self.mode=ModeChange.Request.MODE_TAKEOFF_MAV_END

            case ModeChange.Request.MODE_MISSION_START:
                self.mode=ModeChange.Request.MODE_MISSION_START
            
            # case ModeChange.Request.MODE_OFFBOARD_ROS_END:
            #     self.mode=ModeChange.Request.MODE_OFFBOARD_ROS_END

            #     # Switch to hold mode (doing nothing will leave hovering in offboard mode)
            #     self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.hold())

            case ModeChange.Request.MODE_LAND_START:
                self.mode=ModeChange.Request.MODE_LAND_START

            case ModeChange.Request.MODE_LAND_END:
                self.mode=ModeChange.Request.MODE_LAND_END
                #self.get_logger().info("In mode land end")

            case ModeChange.Request.MODE_HOLD:
                self.mode=ModeChange.Request.MODE_HOLD

                #self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.hold())

            case ModeChange.Request.MODE_KILL:
                self.mode=ModeChange.Request.MODE_KILL

                #self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.kill())


        response.success = True
        self.get_logger().info(f'Requested change to mode: {self.mode}')

        return response

    async def clbk_change_mode_mav_ros(self, request, response):
        self.mode = request.mode

        # Call helper functions if required
        match self.mode:
            case ModeChange.Request.MODE_TAKEOFF_MAV_START:
                # Takeoff
                self.mode=ModeChange.Request.MODE_TAKEOFF_MAV_START
                await self.mission_start()

                # Takeoff finished - transition to end
                self.mode=ModeChange.Request.MODE_TAKEOFF_MAV_END

            case ModeChange.Request.MODE_TAKEOFF_MAV_END:
                self.mode=ModeChange.Request.MODE_TAKEOFF_MAV_END

            case ModeChange.Request.MODE_OFFBOARD_ROS_START:
                self.mode=ModeChange.Request.MODE_OFFBOARD_ROS_START

                # Switch to offboard control to allow ROS command loop to take over
                await self.mission_offboard_ros()
            
            case ModeChange.Request.MODE_OFFBOARD_ROS_END:
                self.mode=ModeChange.Request.MODE_OFFBOARD_ROS_END

                # Switch to hold mode (doing nothing will leave hovering in offboard mode)
                self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.hold())

            case ModeChange.Request.MODE_LAND_MAV_START:
                self.mode=ModeChange.Request.MODE_LAND_MAV_START
                await self.mission_end()

                # Land finished - transition to end
                self.mode=ModeChange.Request.MODE_LAND_MAV_END

            case ModeChange.Request.MODE_LAND_MAV_END:
                self.mode=ModeChange.Request.MODE_LAND_MAV_END
                #self.get_logger().info("In mode land end")

            case ModeChange.Request.MODE_HOLD:
                self.mode=ModeChange.Request.MODE_HOLD

                self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.hold())

            case ModeChange.Request.MODE_KILL:
                self.mode=ModeChange.Request.MODE_KILL

                self.async_loop.run_in_executor(ThreadPoolExecutor(), asyncio.run, self.drone_system.action.kill())


        response.success = True
        self.get_logger().info(f'Changed to mode: {self.mode}')

        return response


    def clbk_cmdloop(self):
        # Continually publish offboard mode heartbeat (note need setpoint published too to stay in offboard mode)
        timestamp = int(self.get_clock().now().nanoseconds/1000)
        offboard_ros.publish_offboard_control_heartbeat_signal(self.pub_offboard_mode, timestamp)

        # Get actual position feedback 
        # Start with drone position (TODO: incorporate load position feedback later)
        tf_drone_rel_world = utils.lookup_tf('world', self.get_name(), self.tf_buffer, timestamp, self.get_logger())


        # Perform actions depending on what mode is requested
        match self.mode:
            # Run takeoff
            # Note that arming and taking off like this may not perform all pre-flight checks that MAVLINK does. 
            # TODO: test if performs preflight checks. Perform manually if doesn't
            case ModeChange.Request.MODE_TAKEOFF_START:
                # Send takeoff setpoint
                offboard_ros.publish_position_setpoint(self.pub_trajectory, 0.0, 0.0, -TAKEOFF_HEIGHT_DRONE, 0.0, timestamp)
                #TODO: Add some formation feedback

                # Update counter for arm phase
                if self.offboard_setpoint_counter <=TAKEOFF_CNT_THRESHOLD: 
                    self.offboard_setpoint_counter += 1
                    #self.get_logger().info(f'offboard_setpoint_counter: {self.offboard_setpoint_counter}')

                # Arm vehicle when offboard message has been published for long enough
                if self.offboard_setpoint_counter == TAKEOFF_CNT_THRESHOLD:                   
                    # Arm
                    offboard_ros.arm(self.pub_vehicle_command, timestamp)

                    # Set in offboard mode
                    offboard_ros.engage_offboard_mode(self.pub_vehicle_command, timestamp)

                # Takeoff complete
                elif self.nav_state ==VehicleStatus.NAVIGATION_STATE_OFFBOARD and tf_drone_rel_world.transform.z>=TAKEOFF_HEIGHT_DRONE:
                    self.mode = ModeChange.Request.MODE_TAKEOFF_END
                    self.offboard_setpoint_counter = 0     
                    self.get_logger().info(f'Takeoff complete')


            # Run main offboard mission
            case ModeChange.Request.MODE_MISSION_START:
                #offboard_ros.engage_offboard_mode(self.pub_vehicle_command, timestamp)
                
                # Only go into offboard once local initial pose and drone arrangements are set (note if this takes too long, vehicle will disarm)
                # TODO: This won't work if FMU is restarted but offboard is not
                # if self.flag_local_init_pos_set and self.flag_desired_pos_rel_load_set:
                #     pass
                    # load_takeoff_state.pos[2] = TAKEOFF_HEIGHT_LOAD 
                    #trajectory_msg = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, load_takeoff_state, self.get_name(), self.tf_buffer, self.get_logger())
                    #self.pub_trajectory.publish(trajectory_msg)

                # Publish setpoints if vehicle is actually in offboard mode
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    ## Trajectory setpoint - input to PID position controller (can set position, velocity and acceleration)
                    # Move to position
                    trajectory_msg = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, self.load_desired_state, self.get_name(), self.tf_buffer, self.get_logger())
                    self.pub_trajectory.publish(trajectory_msg)

                # TODO: Add some formation feedback


            # Run land 
            case ModeChange.Request.MODE_LAND_START:
                
                self.mode = ModeChange.Request.MODE_LAND_END 

                # TODO: Add some formation feedback

            # Run RTL 
            case ModeChange.Request.MODE_RTL_START:

                self.mode = ModeChange.Request.MODE_RTL_END

                # TODO: Add some formation feedback

            # Hold 
            case ModeChange.Request.MODE_HOLD:
                pass

            # Kill
            case ModeChange.Request.MODE_KILL:
                pass
                    
    def clbk_cmdloop_old(self):
        # Publish offboard control modes if OFFBOARD_ROS_START is set
        match self.mode:
            case ModeChange.Request.MODE_OFFBOARD_ROS_START:
                timestamp = int(self.get_clock().now().nanoseconds/1000)
                offboard_ros.publish_offboard_control_heartbeat_signal(self.pub_offboard_mode, timestamp)
               
                # Publish waypoints if vehicle is actually in offboard mode
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    # Move to position
                    trajectory_msg = utils.gen_traj_msg_circle_load(self.vehicle_desired_state_rel_load, self.load_desired_state, self.get_name(), self.tf_buffer, self.get_logger())
                    self.pub_trajectory.publish(trajectory_msg)
    
    ## HELPER FUNCTIONS
    def get_origin_pose(self):
        # Prepare request
        global_origin_pose_req = GetGlobalInitPose.Request()

        # Send request and wait for response
        global_origin_pose_future = self.cli_get_first_drone_init_global_pose.call_async(global_origin_pose_req)
        rclpy.spin_until_future_complete(self, global_origin_pose_future)

        # Return result in state format
        global_origin_pose = global_origin_pose_future.result()
        global_origin_state = State('globe', CS_type.LLA)

        global_origin_state.pos = np.array([global_origin_pose.global_pos.lat, global_origin_pose.global_pos.lon, global_origin_pose.global_pos.alt])
        global_origin_state.att_q = qt.array([global_origin_pose.global_att.q[0], global_origin_pose.global_att.q[1], global_origin_pose.global_att.q[2], global_origin_pose.global_att.q[3]])

        return global_origin_state


    def set_local_init_pose(self):
        # If drone is the first drone, this defines the 'world' co-ordinate system
        if self.drone_id == self.first_drone_num:
            self.vehicle_initial_state_rel_world.pos = np.array([0.0, 0.0, 0.0])
            self.vehicle_initial_state_rel_world.att_q = qt.array([1.0, 0.0, 0.0, 0.0])
        
        # Other drones' local positions are measured relative to the first drone's
        else: # Subscribe to first drone's global pose to set transforms
            while not self.cli_get_first_drone_init_global_pose.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for global initial pose service drone {self.first_drone_num}')

            ## Set initial pose relative to first drone's initial pose
            origin_lla = self.get_origin_pose()

            # Perform transformation
            trans_x, trans_y, trans_z = pm.geodetic2enu(self.vehicle_initial_global_state.pos[0], self.vehicle_initial_global_state.pos[1], self.vehicle_initial_global_state.pos[2], origin_lla.pos[0], origin_lla.pos[1], origin_lla.pos[2]) 

            # Set local init pose (relative to base CS)
            self.vehicle_initial_state_rel_world.pos = np.array([trans_x, trans_y, trans_z])
            self.vehicle_initial_state_rel_world.att_q = qt.array([1.0, 0.0, 0.0, 0.0]) #TODO: Set proper relative orientations
            

        # Publish static transform for init pose (relative to world)
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'world', f'drone{self.drone_id}_init', self.vehicle_initial_state_rel_world.pos, self.vehicle_initial_state_rel_world.att_q, self.tf_static_broadcaster_init_pose)

        self.flag_local_init_pos_set = True 
        


    ### MAVLINK
    ## SETUP 
    # Connect to drone via MAVLINK through UDP
    # Return system object representing drone connected on input address
    # Perform health checks upon connection if desired
    async def connect_mavlink(self, system_address=None, port=None, mavsdk_server_address="localhost"):
        # Set MAVLINK connection ports/addresses based on drone id if not already set
        if system_address == None:
            system_address = f'udp://:1454{self.drone_id}' 
        if port == None:
            port = 50050 + self.drone_id

        self.drone_system = System(mavsdk_server_address=mavsdk_server_address, port=port)

        # Wait for drone to connect
        self.get_logger().info(f'STARTING: Connecting to drone at {system_address} through port {port}')
        await self.drone_system.connect(system_address)

        async for state in self.drone_system.core.connection_state():
            if state.is_connected:
                self.get_logger().info(f"Drone discovered")
                break

        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in self.drone_system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Global position estimate OK")
                break
        
        self.get_logger().info(f'COMPLETE: Connecting to drone at {system_address} \n')

    # Set drone PX4 parameters
    async def set_params(self, takeoff_alt_set=2, rtl_alt_set=5):
        self.get_logger().info("STARTING: Setting parameters")

        # Send setting commands
        await self.drone_system.action.set_takeoff_altitude(takeoff_alt_set)
        await self.drone_system.action.set_return_to_launch_altitude(rtl_alt_set)

        # Wait for settings to be set at the correct values
        takeoff_alt = await self.drone_system.action.get_takeoff_altitude()
        rtl_alt = await self.drone_system.action.get_return_to_launch_altitude()

        while takeoff_alt != takeoff_alt_set or  rtl_alt != rtl_alt_set:
            takeoff_alt = await self.drone_system.action.get_takeoff_altitude()
            rtl_alt = await self.drone_system.action.get_return_to_launch_altitude()

        self.get_logger().info("COMPLETE: Setting parameters \n")


    ## MISSION
    # Arm, takeoff and switch to offboard control mode using MAVSDK (can alternatively do on remote)
    # TODO: Add error checking between connection, arming and each mode transition to ensure successful. See: https://mavsdk.mavlink.io/main/en/cpp/guide/taking_off_landing.html 
    async def mission_start(self):
        self.get_logger().info("STARTING: Takeoff routine")

        # Start in hold mode
        #await self.drone_system.action.hold()
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.action.hold())
        executor.shutdown(wait=True)

        # Arm drone and wait 2 sec
        self.get_logger().info("-- Arming")
        #await self.drone_system.action.arm()
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.action.arm())
        executor.shutdown(wait=True)

        # Get drone to take off
        self.get_logger().info("-- Taking off")
        #await self.drone_system.action.takeoff()
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.action.takeoff())
        executor.shutdown(wait=True)

        self.get_logger().info("COMPLETE: Takeoff routine \n")



    # Use MAVLink to send waypoints
    # TODO: error checking
    async def mission_offboard_mav(self):
        self.get_logger().info("STARTING: Offboard routine - MAV")
        
        # Start sending velocity command (stay at current position)
        vel_start = offboard.VelocityBodyYawspeed(0,0,0,0)
        await self.drone_system.offboard.set_velocity_body(vel_start)

        # Switch to offboard control mode
        self.get_logger().info("-- Switch to offboard control")
        await self.drone_system.offboard.start()

        # Fly to desired position
        self.get_logger().info("-- Flying to set position")
        vel_2 = offboard.VelocityNedYaw(0.25,0,0,0)
        pos_2 = offboard.PositionNedYaw(10,0,-2,0)
        await self.drone_system.offboard.set_position_velocity_ned(pos_2, vel_2)

        # Only finish flight when within desired error of setpoint (might need to remove velocity feed-forward for more precise positioning)
        pos_2_set = telemetry.PositionNed(pos_2.north_m, pos_2.east_m, pos_2.down_m)
        err_rad = 0.25

        async for drone_pos_vel in self.drone_system.telemetry.position_velocity_ned():
            pos_current = (drone_pos_vel.position.north_m, drone_pos_vel.position.east_m, drone_pos_vel.position.down_m)
            pos_setpoint = (pos_2_set.north_m, pos_2_set.east_m, pos_2_set.down_m)

            if utils.within_radius_3D(pos_current, pos_setpoint, err_rad):
                break
        
        self.get_logger().info("-- At desired position")
        await asyncio.sleep(5)
        self.get_logger().info("COMPLETE: Offboard routine - MAV\n")


    # Start Offboard mode to allow ROS' control of waypoints (note that waypoints must first be allowed to be sent)
    # TODO: error checking
    async def mission_offboard_ros(self):
        self.get_logger().info("STARTING: Offboard routine setup - ROS")
        
        # Start sending velocity command (stay at current position)
        vel_start = offboard.VelocityBodyYawspeed(0,0,0,0)
        #await self.drone_system.offboard.set_velocity_body(vel_start)
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.offboard.set_velocity_body(vel_start))
        executor.shutdown(wait=True)

        # Switch to offboard control mode
        #await self.drone_system.offboard.start()
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.offboard.start())
        executor.shutdown(wait=True)

        self.get_logger().info("COMPLETE: Offboard routine setup - ROS\n")


    # RTL, land and disarm using MAVSDK (can alternatively do on remote)
    # TODO: Error checking
    async def mission_end(self):
        self.get_logger().info("STARTING: Landing routine")

        # Turn off offboard mode
        #await self.drone_system.offboard.stop()
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.offboard.stop())
        executor.shutdown(wait=True)

        # Return to home and land
        self.get_logger().info("-- Returning to home")
        #await self.drone_system.action.return_to_launch()
        executor = ThreadPoolExecutor(max_workers=1)
        self.async_loop.run_in_executor(executor, asyncio.run, self.drone_system.action.return_to_launch())
        executor.shutdown(wait=True)

        self.get_logger().info("COMPLETE: Landing routine \n")


async def main_async(args=None):
    rclpy.init(args=args)

    # Could use a task group here if wanted to create multiple independent drone tasks for some reason
    drone = await Drone.create(node_name='drone9', namespace='px4_9')

    rclpy.spin(drone)

    # Destroy node
    drone.destroy_node()
    rclpy.shutdown()

def main():
    #asyncio.run(main_async())

    # Create node
    rclpy.init()
    drone = Drone.create(node_name='drone9', namespace='px4_9')
    rclpy.spin(drone)

    # Destroy node
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()