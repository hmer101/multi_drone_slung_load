# Contains methods to send commands to an FMU running PX4 directly via ROS2
#
# Modified from: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py
# Author: Harvey Merton
# Date: 06/21/2023


import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import utils


def arm(pub_vehicle_command, timestamp):
    """Send an arm command to the vehicle."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                            pub_vehicle_command, timestamp, param1=1.0)

def disarm(pub_vehicle_command, timestamp):
    """Send a disarm command to the vehicle."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                            pub_vehicle_command, timestamp, param1=0.0)

def kill(pub_vehicle_command, timestamp):
    """Send a kill command to the vehicle."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, 
                            pub_vehicle_command, timestamp, param1=1.0)

# TODO: Debug
# def takeoff(pub_vehicle_command, timestamp, takeoff_state_lla):
#     """Switch to takeoff mode."""
#     publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 
#                             pub_vehicle_command, timestamp, param1=0.0, param2=0.0, param4=0.0, 
#                             param5=takeoff_state_lla.pos[0], param6=takeoff_state_lla.pos[1], param7=takeoff_state_lla.pos[2]) #
#     # Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|

def land(pub_vehicle_command, timestamp):
    """Switch to land mode."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 
                            pub_vehicle_command, timestamp)


def engage_offboard_mode(pub_vehicle_command, timestamp):
    """Switch to offboard mode."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                            pub_vehicle_command, timestamp, param1=1.0, param2=6.0)


# Note that can only reset home after vehicle is armed
def set_origin(pub_vehicle_command, lat, lon, alt, timestamp):
    """Set GPS origin location"""   
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN, 
                            pub_vehicle_command, timestamp, param5=lat, param6=lon, param7=alt)
    


def publish_offboard_control_heartbeat_signal(pub_offboard_mode, what_control, timestamp):
    """Publish the offboard control mode."""
    msg = OffboardControlMode()

    # Select where setpoints are injected in: https://docs.px4.io/main/en/flight_stack/controller_diagrams.html 
    # Note that bipassed controllers are disabled
    msg.position = False
    msg.velocity = False
    msg.acceleration = False
    msg.attitude = False
    msg.body_rate = False
    msg.actuator = False

    match what_control:
        case 'pos':
            msg.position = True
        case 'vel':
            msg.velocity = True
        case 'accel':
            msg.acceleration = True

    msg.timestamp = timestamp
    pub_offboard_mode.publish(msg)

def publish_position_setpoint(pub_trajectory, x: float, y: float, z: float, yaw: float, timestamp):
    """Publish the trajectory setpoint."""
    msg = TrajectorySetpoint()
    msg.position = [x, y, z]
    msg.yaw = yaw
    msg.timestamp = timestamp
    pub_trajectory.publish(msg)


def publish_vehicle_command(command, pub_vehicle_command, timestamp, **params) -> None:
    """Publish a vehicle command."""
    instance_num = utils.extract_instance_from_connection(pub_vehicle_command)+1
    
    # Generate and publish the vehicle command
    msg = VehicleCommand()
    msg.command = command
    msg.param1 = params.get("param1", 0.0)
    msg.param2 = params.get("param2", 0.0)
    msg.param3 = params.get("param3", 0.0)
    msg.param4 = params.get("param4", 0.0)
    msg.param5 = params.get("param5", 0.0)
    msg.param6 = params.get("param6", 0.0)
    msg.param7 = params.get("param7", 0.0)
    msg.target_system = instance_num # Target system must match the MAV_SYS_ID/UXRCE_DDS_KEY which is px4_instance+1: https://docs.px4.io/main/en/ros/ros2_multi_vehicle.html#adjusting-the-target-system-value
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    msg.timestamp = timestamp
    pub_vehicle_command.publish(msg)
