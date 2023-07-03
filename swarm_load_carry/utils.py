import math
import quaternionic as qt
import numpy as np
import rclpy

import frame_transforms as ft

from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import TrajectorySetpoint
from tf2_ros import TransformException

## STRING HANDLING 

# Extract the an instance number from a ROS publisher/subscriber/client/service
def extract_instance_from_connection(connection):
    topic_name = connection.topic_name

    namespace = topic_name.split('/')[1]
    instance_num = namespace.split('_')[1]

    return int(instance_num)

## CONVERSIONS
# Normalize a quaternion and return the equivalent numpy representation
def q_to_normalized_np(q: qt.array):
    q_norm = q.normalized

    return np.array([q_norm.w, q_norm.x, q_norm.y, q_norm.z])


## GEOMETRY

# Find Euler distance between two points in 3D space
# Inputs: points p1, p2 as 3-tuples
# Outputs: dist - distance between points (in input units)
def dist_euler_3D(p1, p2):
    dist = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**0.5

    return dist

# Check if a point is within an error radius of another point
# Inputs: points p1, p2 as 3-tuples
# Outputs: true if within radius, false otherwise
def within_radius_3D(p1, p2, rad):
    if dist_euler_3D(p1,p2) < rad:
        return True
    else:
        return False
    

# Generates a list of <X,Y,Z> co-ordinates to set drones equally spaced around a load on a circumscribing circle.
# Input r is radius of circle circumscribing drone positions.
# Input Z is a list (with num_drone elements) of Z heights above the load for the drones to fly 
def generate_points_cylinder(num_points, r, z):
    positions = []

	# Use polar co-ordinate system to generate (x,y) points in circumscribing circle 
    for i in range(num_points):
        positions.append(tuple([r*math.cos(2*math.pi*i/num_points), r*math.sin(2*math.pi*i/num_points), z[i]]))
    
    return positions


## TRANSFORMS

# Build and send a tf
# Inputs: time, names of parent and child frames, position (as np array x,y,z), attitude (as quaternionic array), broadcaster (static or dynamic broadcaster to send tf over)
# Outputs: None 
def broadcast_tf(time, frame_parent, frame_child, pos, att, broadcaster):
    # Broadcast frame transform
    t = TransformStamped()

    t.header.stamp = time
    t.header.frame_id = frame_parent
    t.child_frame_id = frame_child

    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]

    t.transform.rotation.x = float(att.x)
    t.transform.rotation.y = float(att.y)
    t.transform.rotation.z = float(att.z)
    t.transform.rotation.w = float(att.w)

    broadcaster.sendTransform(t)

# Looks up a co-ordinate frame transform 
def lookup_tf(target_frame, source_frame, tf_buffer, time, logger):
    t = None
    
    try:
        t = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            time)
    except TransformException as ex:
        logger.warn(
            f'Could not transform {source_frame} to {target_frame}: {ex}')
    
    return t


## TRAJECTORY GENERATION
# Note trajectories sent to Pixhawk controller must be in NED co-ordinates relative to initial drone position. ENU -> NED and frame transformations handled here

# Make drone orbit in a circle of radius r and height h
def gen_traj_msg_orbit(r, theta, h):
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.position[0] = r * np.cos(theta)
    trajectory_msg.position[1] = r * np.sin(theta)
    trajectory_msg.position[2] = -h

    return trajectory_msg

# Make drone follow desired load position, at the desired location relative to the load
# TODO: Add interpolation (especially for rotation/yaw of swarm)
def gen_traj_msg_circle_load(vehicle_desired_state_rel_load, load_desired_state, drone_name, tf_buffer, logger):
    trajectory_msg = TrajectorySetpoint()

    # Add effect of desired load orientation (note different physical connections to load will require different algorithms)
    # TODO: Add switch statement for gimbal controlling yaw
    vehicle_desired_pos_rel_load_rot =  load_desired_state.att_q.rotate(vehicle_desired_state_rel_load.pos) #np.copy(vehicle_desired_state_rel_load.pos) #

    # Desired vehicle pos relative to world (in ENU)= desired vehicle pos relative to load + desired load pos rel to world 
    vehicle_desired_state_rel_world = [vehicle_desired_pos_rel_load_rot[0] + load_desired_state.pos[0], 
                                            vehicle_desired_pos_rel_load_rot[1] + load_desired_state.pos[1],
                                            vehicle_desired_pos_rel_load_rot[2] + load_desired_state.pos[2]]
    
    # logger.info(f'UTILS: load_desired_state.pos: {[load_desired_state.pos[0], load_desired_state.pos[1], load_desired_state.pos[2]]}')
    # logger.info(f'UTILS: vehicle_desired_state_rel_load: {[vehicle_desired_state_rel_load.pos[0], vehicle_desired_state_rel_load.pos[1], vehicle_desired_state_rel_load.pos[2]]}')
    # logger.info(f'UTILS: vehicle_desired_state_rel_world: {vehicle_desired_state_rel_world}')

    # Need drone rel to drone_init (Pixhawk's frame). Add transforms from world to drone_init frames and convert from ENU to NED
    t = lookup_tf(f'{drone_name}_init', 'world', tf_buffer, rclpy.time.Time(), logger)

    if t != None:
        trajectory_msg = TrajectorySetpoint()

        # Converts ENU-> NED
        trajectory_msg.position[0] = vehicle_desired_state_rel_world[1] + t.transform.translation.y
        trajectory_msg.position[1] = vehicle_desired_state_rel_world[0] + t.transform.translation.x
        trajectory_msg.position[2] = -(vehicle_desired_state_rel_world[2] + t.transform.translation.z)

        # TODO: Handle yaw
        # trajectory_msg.yaw = 
        # drone_orientations[i, :] = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]

        # trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

    # logger.info(f'UTILS: traj_msg.pos: {[trajectory_msg.position[0], trajectory_msg.position[1], trajectory_msg.position[2]]}')


    return trajectory_msg

# Generate trajectory message that gives setpoint at a specific height and orientation
def gen_traj_msg_straight_up(takeoff_height, takeoff_q):
    trajectory_msg = TrajectorySetpoint()

    # Set position straight up
    trajectory_msg.position[0] = 0.0
    trajectory_msg.position[1] = 0.0
    trajectory_msg.position[2] = -takeoff_height

    # Get yaw in NED from takeoff_q
    q_NED = ft.ros_to_px4_orientation(q_to_normalized_np(takeoff_q))
    trajectory_msg.yaw = ft.quaternion_get_yaw(q_NED)

    return trajectory_msg


# Make drone travel at set velocity
def gen_traj_msg_vel(desired_velocity):
    trajectory_msg = TrajectorySetpoint()

    trajectory_msg.position[0] = None
    trajectory_msg.position[1] = None
    trajectory_msg.position[2] = None

    # Convert ENU-> NED
    trajectory_msg.velocity[0] = desired_velocity[1]
    trajectory_msg.velocity[1] = desired_velocity[0]
    trajectory_msg.velocity[2] = -desired_velocity[2]

    return trajectory_msg