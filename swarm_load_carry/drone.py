# Contains the drone class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023

import asyncio, rclpy, utils # Note import utils needs additions to setup.py. See here: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
import numpy as np

from mavsdk import System, offboard, telemetry

import rclpy.qos as qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import Pose, Point, Quaternion

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand


NUM_DRONES = 3
OFFBOARD_TIME_MAX = 10 # Maximum time to be in offboard mode (ROS)

# Node to encapsulate drone information and actions
class Drone(Node):
    ## Initialization
    #def __init__(self):
     #   super().__init__('droneX')

    @classmethod
    async def create(cls, system_address="udp://:14540", port=50050, mavsdk_server_address="localhost"):
        self = Drone('px4_X')
        #self.drone_id = 


        ### MAVLINK
        # Connect to drone via MAVLINK through UDP
        self.drone_system = System(mavsdk_server_address=mavsdk_server_address, port=port)
        await self.drone_system.connect(system_address)
        await self.wait_for_drone(system_address, port)



        ### ROS2
        ## SUBSCRIBERS
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT, #QoSReliabilityPolicy.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL, #QoSDurabilityPolicy.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST, #QoSHistoryPolicy.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Local FMU outputs
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/px4_1/fmu/out/vehicle_status',
            self.clbk_vehicle_status,
            qos_profile)

        self.sub_attitude = self.create_subscription(
            VehicleAttitude,
            '/px4_1/fmu/out/vehicle_attitude',
            self.clbk_vehicle_attitude,
            qos_profile)

        self.sub_local_position = self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.clbk_vehicle_local_position,
            qos_profile)  

        # TODO: Sub to other drones and load setpoints for distributed control!

        ## PUBLISHERS
        #self.pub_vehicle_cmd = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_profile)
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_profile)
        self.pub_trajectory = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_profile)

        #self.pub_pose_actual = self.create_publisher(Pose, 'pose_actual', 10)
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))

        ## TIMERS
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.clbk_cmdloop)

        
        ## INSTANCE VARS
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        #self.time_offboard = 0 # Length of time been in offboard mode for
        #self.offboard_done = False

        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.5


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

        # self.get_logger().info(f'Publishing pose: {self.vehicle_local_position}, {self.vehicle_attitude}')

        # pose = Pose()
        # pose.position = Point()
        # pose.position.x, pose.position.y, pose.position.z = self.vehicle_local_position[0], self.vehicle_local_position[1], self.vehicle_local_position[2]
        # pose.orientation = Quaternion()
        # pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w  = self.vehicle_attitude[0], self.vehicle_attitude[1], self.vehicle_attitude[2], self.vehicle_attitude[3]

        # self.pub_pose_actual.publish(pose)

    async def clbk_vehicle_status(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def clbk_cmdloop(self):
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
        #print("SENDING")
        #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, float(1))

        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.pub_offboard_mode.publish(offboard_msg)

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -5.0
            self.pub_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt

            # Start timer 
            # if self.time_offboard_start == False:
            #     self.time_offboard_start = int(Clock().now().nanoseconds / 1000)
            # elif (int(Clock().now().nanoseconds / 1000) - self.time_offboard_start) == OFFBOARD_TIME_MAX:
            #     self.time_offboard_start == True


    def publish_vehicle_command(self, command: int, param1: float, param2: float =0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        #msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)

        self.pub_vehicle_cmd.publish(msg)


    ## SETUP 
    # Return system object representing drone connected on input address
    # Perform health checks upon connection if desired
    async def wait_for_drone(self, system_address, port):
        print(f'STARTING: Connecting to drone at {system_address} through port {port}')

        async for state in self.drone_system.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone_system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break
        
        print(f'COMPLETE: Connecting to drone at {system_address} \n')


    # Set drone PX4 parameters
    async def set_params(self, takeoff_alt_set=2, rtl_alt_set=5):
        print("STARTING: Setting parameters")

        # Send setting commands
        await self.drone_system.action.set_takeoff_altitude(takeoff_alt_set)
        await self.drone_system.action.set_return_to_launch_altitude(rtl_alt_set)

        # Wait for settings to be set at the correct values
        takeoff_alt = await self.drone_system.action.get_takeoff_altitude()
        rtl_alt = await self.drone_system.action.get_return_to_launch_altitude()

        while takeoff_alt != takeoff_alt_set or  rtl_alt != rtl_alt_set:
            takeoff_alt = await self.drone_system.action.get_takeoff_altitude()
            rtl_alt = await self.drone_system.action.get_return_to_launch_altitude()

        print("COMPLETE: Setting parameters \n")


    ## MISSION
    # Arm, takeoff and switch to offboard control mode using MAVSDK (can alternatively do on remote)
    # TODO: Add error checking between connection, arming and each mode transition to ensure successful. See: https://mavsdk.mavlink.io/main/en/cpp/guide/taking_off_landing.html 
    async def mission_start(self):
        print("STARTING: Takeoff routine")

        # Start in hold mode
        await self.drone_system.action.hold()

        # Arm drone and wait 2 sec
        print("-- Arming")
        await self.drone_system.action.arm()
        await asyncio.sleep(2)

        # Get drone to take off
        print("-- Taking off")
        await self.drone_system.action.takeoff()

        # Wait until takeoff complete
        async for current_flight_mode in self.drone_system.telemetry.flight_mode(): 
            if current_flight_mode == telemetry.FlightMode.HOLD:
                break

        await asyncio.sleep(2)

        print("COMPLETE: Takeoff routine \n")


    # Use MAVLink to send waypoints
    # TODO: error checking
    async def mission_offboard_mav(self):
        print("STARTING: Offboard routine - MAV")
        
        # Start sending velocity command (stay at current position)
        vel_start = offboard.VelocityBodyYawspeed(0,0,0,0)
        await self.drone_system.offboard.set_velocity_body(vel_start)

        # Switch to offboard control mode
        print("-- Switch to offboard control")
        await self.drone_system.offboard.start()

        # Fly to desired position
        print("-- Flying to set position")
        vel_2 = offboard.VelocityNedYaw(0.25,0,0,0)
        pos_2 = offboard.PositionNedYaw(10,0,-2,0)
        await self.drone_system.offboard.set_position_velocity_ned(pos_2, vel_2)

        # Only finish flight when within desired error of setpoint (might need to remove velocity feed-forward for more precise positioning)
        pos_2_set = telemetry.PositionNed(pos_2.north_m, pos_2.east_m, pos_2.down_m)
        err_rad = 0.25

        async for drone_pos_vel in self.drone_system.telemetry.position_velocity_ned():
            # print(f"Current: {drone_pos_vel.position.north_m}, {drone_pos_vel.position.east_m}, {drone_pos_vel.position.down_m}")
            # print(f"Setpoint: {pos_2_set.north_m}, {pos_2_set.east_m}, {pos_2_set.down_m}")

            pos_current = (drone_pos_vel.position.north_m, drone_pos_vel.position.east_m, drone_pos_vel.position.down_m)
            pos_setpoint = (pos_2_set.north_m, pos_2_set.east_m, pos_2_set.down_m)

            if utils.within_radius_3D(pos_current, pos_setpoint, err_rad):
                break
        
        print("-- At desired position")
        await asyncio.sleep(5)
        print("COMPLETE: Offboard routine - MAV\n")


    # Start Offboard mode to allow ROS' control of waypoints (note that waypoints must first be allowed to be sent)
    # TODO: error checking
    async def mission_offboard_ros(self):
        print("STARTING: Offboard routine - ROS")
        
        # Start sending velocity command (stay at current position)
        vel_start = offboard.VelocityBodyYawspeed(0,0,0,0)
        await self.drone_system.offboard.set_velocity_body(vel_start)

        # Switch to offboard control mode
        print("-- Switch to offboard control")
        await self.drone_system.offboard.start()
        
        # Wait until drone switched back into hold mode (ROS offboard is done)
        # async for current_flight_mode in self.drone_system.telemetry.flight_mode(): 
        #     if current_flight_mode == telemetry.FlightMode.HOLD:
        #         break

        print("COMPLETE: Offboard routine - ROS\n")


    # RTL, land and disarm using MAVSDK (can alternatively do on remote)
    # TODO: Error checking
    async def mission_end(self):
        print("STARTING: Landing routine")

        # Turn off offboard mode
        await self.drone_system.offboard.stop()

        # Return to home and land
        print("-- Returning to home")
        await self.drone_system.action.return_to_launch()

        # Only disarm once landed
        async for drone_state in self.drone_system.telemetry.landed_state():
            if drone_state == telemetry.LandedState.ON_GROUND:
                break
        await self.drone_system.action.disarm()
        #await drone.action.hold()

        print("COMPLETE: Landing routine \n")


    # Run hardcoded offboard control mission with ros or MAVLINK sending offboard control commands
    async def run_hardcoded(self, method_ros=True):
        # Set drone parameters to default values
        await self.set_params()

        # Start mission
        await self.mission_start()

        # Run mission
        if method_ros:
            await self.mission_offboard_ros()
        else:
            await self.mission_offboard_mav()

            # End mission
            await self.mission_end()


async def main_async(args=None):
    # Create node and connect
    rclpy.init(args=args)
    drone = await Drone.create(system_address="udp://:14541", port=50051)
    #drone_fly_coroutine = asyncio.ensure_future(drone.run_hardcoded())
    #await drone_fly_coroutine

    # Offboard flight
    await drone.run_hardcoded()
    await rclpy.spin(drone)

    # Destroy node
    drone.destroy_node()
    rclpy.shutdown()

def main():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main_async())

if __name__ == '__main__':
    main()