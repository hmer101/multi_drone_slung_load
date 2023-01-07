# Contains the drone class and related methods
#
# Author: Harvey Merton
# Date: 01/06/2023


import asyncio
from mavsdk import System, offboard, telemetry
import utils

# Class to encapsulate drone information and actions
class Drone:
    @classmethod
    async def create(cls, system_address="udp://:14540", port=50050, mavsdk_server_address="localhost"):
        self = Drone()
        #self.drone_id = 

        # Connect to drone via MAVLINK through UDP
        self.drone_system = System(mavsdk_server_address=mavsdk_server_address, port=port)
        await self.drone_system.connect(system_address)
        await self.wait_for_drone(system_address, port)

        #self.uid = self.drone_system.info.get_identification()

        return self


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
    async def mission_offboard(self):
        print("STARTING: Offboard routine")
        
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
        print("COMPLETE: Offboard routine \n")



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


    # Run hardcoded offboard control mission using MavLink
    async def run_hardcoded_mavlink(self):
        # Set drone parameters to default values
        await self.set_params()

        # Start mission (maybe replace entirely with offboard later - for multi-drone co-ordination)
        await self.mission_start()

        # Run mission
        #await self.mission_offboard()

        # End mission (maybe replace entirely with offboard later - for multi-drone co-ordination)
        await self.mission_end()

