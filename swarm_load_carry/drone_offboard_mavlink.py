# Contains methods to send commands to an FMU running PX4 via MAVLink
#
# Author: Harvey Merton
# Date: 06/26/2023

import asyncio, rclpy, utils # Note import utils needs additions to setup.py. See here: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
import numpy as np
import quaternionic as qt
from swarm_load_carry.state import State, CS_type

from mavsdk import System, offboard, telemetry

from swarm_load_carry_interfaces.srv import ModeChange # Note must build workspace and restart IDE before custom packages are found by python

from concurrent.futures import ThreadPoolExecutor


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

    # Get position where GPS co-ordinates are relative to
    initial_pos_lla = await self.drone_system.telemetry.get_gps_global_origin()
    self.vehicle_initial_global_state.pos = np.array([initial_pos_lla.latitude_deg, initial_pos_lla.longitude_deg, initial_pos_lla.altitude_m])
    
    # Log and return
    self.get_logger().info('DRONE NODE CONNECTED THROUGH MAVLINK')

    self.get_logger().info('Setup complete')

    return self


async def clbk_change_mode(self, request, response):
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

async def main_async(args=None):
    rclpy.init(args=args)

    # Could use a task group here if wanted to create multiple independent drone tasks for some reason
    drone = await Drone.create(node_name='drone9', namespace='px4_9')

    rclpy.spin(drone)

    # Destroy node
    drone.destroy_node()
    rclpy.shutdown()