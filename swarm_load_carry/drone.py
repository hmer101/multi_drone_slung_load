# Contains the drone class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import asyncio, rclpy, utils # Note import utils needs additions to setup.py. See here: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
import numpy as np

from mavsdk import System, offboard, telemetry

import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.task import Future

from geometry_msgs.msg import Pose, Point, Quaternion
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand
from swarm_load_carry_interfaces.srv import ModeChange # Note must build workspace and restart IDE before custom packages are found by python

from concurrent.futures import ThreadPoolExecutor
from threading import Thread
import time

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1

# Node to encapsulate drone information and actions
class Drone(Node):

    ## Initialization
    def __init__(self, name, namespace):
        super().__init__(node_name=name, namespace=namespace)

        self.ns = self.get_namespace()
        self.drone_id = int(str(self.ns)[-1])

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.mode = ModeChange.Request.MODE_UNASSIGNED

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])

        timer_period = 0.02  # seconds
        self.dt = timer_period
        self.theta = 0.0
        self.radius = self.drone_id*5
        self.omega = 0.5

        # For MAVLINK connection
        self.drone_system = None
        self.msg_future_return = None
        self.async_loop = asyncio.get_event_loop()

        self.takeoff_altitude = 0.0 # THIS IS TEMPORARY - REPLACE USEAGE WITH 'await self.drone_system.action.get_takeoff_altitude()'

        # Parameters
        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)
        self.declare_parameter('load_id', DEFAULT_LOAD_ID)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value


        ## TIMERS
        self.timer = self.create_timer(timer_period, self.clbk_cmdloop)

        ### ROS2
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ## PUBLISHERS
        #self.pub_vehicle_cmd = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_profile)
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', qos_profile)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', qos_profile)

        ## SUBSCRIBERS
        # Local FMU outputs
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

        # Payload 
        # self.sub_payload_attitude_desired = self.create_subscription(
        #     VehicleAttitude,
        #     f'load_{self.load_id}/out/attitude',
        #     self.clbk_load_attitude,
        #     qos_profile)
        
        # self.sub_payload_position_desired = self.create_subscription(
        #     VehicleLocalPosition,
        #     f'load_{self.load_id}/out/local_position',
        #     self.clbk_load_local_position,
        #     qos_profile)

        # self.sub_payload_attitude_desired = self.create_subscription(
        #     VehicleAttitudeSetpoint,
        #     f'load_{self.load_id}/in/desired_attitude',
        #     self.clbk_load_desired_attitude,
        #     qos_profile)
        
        # self.sub_payload_position_desired = self.create_subscription(
        #     VehicleLocalPositionSetpoint,
        #     f'load_{self.load_id}/in/desired_local_position',
        #     self.clbk_load_desired_local_position,
        #     qos_profile)

        # TODO: Sub to other drones and load setpoints for distributed control!

        ## SERVICES
        self.srv_mode_change = self.create_service(
            ModeChange,
            f'{self.ns}/mode_change',
            self.clbk_change_mode)

        ## CLIENTS


        ## Print information
        self.get_logger().info('DRONE NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')
        #self.get_logger().info(f'ID: {self.drone_id}')
    

    # Create a node with MAVLINK connections initialized
    @classmethod
    async def create(cls, node_name='drone9', namespace='px4_9', msg_future_return=None):
        # Create node without MAVLINK connections (i.e. only has ROS connections here)
        self = Drone(name=node_name, namespace=namespace)
        
        # Connect MAVLINK
        await self.connect_mavlink()
        self.msg_future_return = msg_future_return
        
        # Set drone default parameters
        await self.set_params()

        self.get_logger().info('DRONE NODE CONNECTED THROUGH MAVLINK')

        return self


    ## CALLBACKS
    def clbk_vehicle_attitude(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def clbk_vehicle_local_position(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz

        # pos_current = (self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2])
        # self.get_logger().info(f'POS UPDATE: {pos_current}')

    def clbk_vehicle_status(self, msg):
        # TODO: handle NED->ENU transformation
        #if self.mode == ModeChange.Request.MODE_UNASSIGNED: 
        self.get_logger().info(f'NAV_STATUS: {msg.nav_state}')
        self.get_logger().info(f'  - offboard status: {VehicleStatus.NAVIGATION_STATE_OFFBOARD}')
        self.nav_state = msg.nav_state

    async def clbk_change_mode(self, request, response):
        #asyncio.set_event_loop(self.async_loop)

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
        # Publish offboard control modes if OFFBOARD_ROS_START is set
        match self.mode:
            case ModeChange.Request.MODE_OFFBOARD_ROS_START:
                offboard_msg = OffboardControlMode()
                offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.position=True
                offboard_msg.velocity=False
                offboard_msg.acceleration=False
                self.pub_offboard_mode.publish(offboard_msg)

                # Publish waypoints if vehicle is actually in offboard mode
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.position[0] = self.radius * np.cos(self.theta)
                    trajectory_msg.position[1] = self.radius * np.sin(self.theta)
                    trajectory_msg.position[2] = -5.0*self.drone_id
                    self.pub_trajectory.publish(trajectory_msg)

                    self.theta = self.theta + self.omega * self.dt


    # # Only return once vehicle has reached desired position
    # def wait_for_pos(self, goal_pos, error_dist):
    #     pos_current = (self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2])

    #     while not utils.within_radius_3D(pos_current, goal_pos, error_dist):
    #         pos_current = (self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2])
    #         self.get_logger().info(f'pos_current: {pos_current}')
    #         self.get_logger().info(f'goal_pos: {goal_pos}')
    #         time.sleep(1)

    #     return True


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
        self.takeoff_alt = await self.drone_system.action.get_takeoff_altitude() # REMOVE self. ONCE TEMP VAR REMOVED!
        rtl_alt = await self.drone_system.action.get_return_to_launch_altitude()

        while self.takeoff_alt != takeoff_alt_set or  rtl_alt != rtl_alt_set:
            self.takeoff_alt = await self.drone_system.action.get_takeoff_altitude()
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
    asyncio.run(main_async())

if __name__ == '__main__':
    main()