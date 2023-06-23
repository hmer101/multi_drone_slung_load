# Contains methods to send commands to an FMU running PX4 directly via ROS2
#
# Modified from: https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard_py/offboard_control.py
# Author: Harvey Merton
# Date: 06/21/2023


import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import utils

# class OffboardControl(Node):
#     """Node for controlling a vehicle in offboard mode."""

#     def __init__(self) -> None:
        # super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        # Create publishers
        # self.offboard_control_mode_publisher = self.create_publisher(
        #     OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.trajectory_setpoint_publisher = self.create_publisher(
        #     TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        # self.vehicle_command_publisher = self.create_publisher(
        #     VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        # self.vehicle_local_position_subscriber = self.create_subscription(
        #     VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        # self.vehicle_status_subscriber = self.create_subscription(
        #     VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)



        # Initialize variables
        # self.offboard_setpoint_counter = 0
        # self.vehicle_local_position = VehicleLocalPosition()
        # self.vehicle_status = VehicleStatus()
        # self.takeoff_height = -5.0

        # # Create a timer to publish control commands
        # self.timer = self.create_timer(0.1, self.timer_callback)




    # def vehicle_local_position_callback(self, vehicle_local_position):
    #     """Callback function for vehicle_local_position topic subscriber."""
    #     self.vehicle_local_position = vehicle_local_position

    # def vehicle_status_callback(self, vehicle_status):
    #     """Callback function for vehicle_status topic subscriber."""
    #     self.vehicle_status = vehicle_status



def arm(pub_vehicle_command, timestamp):
    """Send an arm command to the vehicle."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                            pub_vehicle_command, timestamp, param1=1.0)
    #self.get_logger().info('Arm command sent')

def disarm(pub_vehicle_command, timestamp):
    """Send a disarm command to the vehicle."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                            pub_vehicle_command, timestamp, param1=0.0)
    #self.get_logger().info('Disarm command sent')

def engage_offboard_mode(pub_vehicle_command, timestamp):
    """Switch to offboard mode."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                            pub_vehicle_command, timestamp, param1=1.0, param2=6.0)
    #self.get_logger().info("Switching to offboard mode")

def takeoff(pub_vehicle_command, timestamp, takeoff_state_lla):
    """Switch to takeoff mode."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 
                            pub_vehicle_command, timestamp, param1=0.0, param2=0.0, param4=0.0, 
                            param5=takeoff_state_lla.pos[0], param6=takeoff_state_lla.pos[1], param7=takeoff_state_lla.pos[2])
    # Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|


def land(pub_vehicle_command, timestamp):
    """Switch to land mode."""
    publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 
                            pub_vehicle_command, timestamp)
    #self.get_logger().info("Switching to land mode")

def publish_offboard_control_heartbeat_signal(pub_offboard_mode, timestamp):
    """Publish the offboard control mode."""
    msg = OffboardControlMode()

    # Select where setpoints are injected in: https://docs.px4.io/main/en/flight_stack/controller_diagrams.html 
    # Note that bipassed controllers are disabled
    msg.position = True
    msg.velocity = False
    msg.acceleration = False
    msg.attitude = False
    msg.body_rate = False
    msg.actuator = False

    msg.timestamp = timestamp
    pub_offboard_mode.publish(msg)

# def publish_position_setpoint(self, x: float, y: float, z: float):
#     """Publish the trajectory setpoint."""
#     msg = TrajectorySetpoint()
#     msg.position = [x, y, z]
#     msg.yaw = 1.57079  # (90 degree)
#     msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
#     self.trajectory_setpoint_publisher.publish(msg)
#     self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

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
    msg.timestamp = timestamp #msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    pub_vehicle_command.publish(msg)

# def timer_callback(self) -> None:
#     """Callback function for the timer."""
#     self.publish_offboard_control_heartbeat_signal()

#     if self.offboard_setpoint_counter == 10:
#         self.engage_offboard_mode()
#         self.arm()

#     if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
#         self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
 
#     elif self.vehicle_local_position.z <= self.takeoff_height:
#         self.land()
#         exit(0)

#     if self.offboard_setpoint_counter < 11:
#         self.offboard_setpoint_counter += 1


# def main(args=None) -> None:
#     print('Starting offboard control node...')
#     rclpy.init(args=args)
#     offboard_control = OffboardControl()
#     rclpy.spin(offboard_control)
#     offboard_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)
