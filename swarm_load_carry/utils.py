import math
import quaternionic as qt
import rclpy

import numpy as np
#from numpy.typing 

import frame_transforms as ft

from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import TrajectorySetpoint
from tf2_ros import TransformException

from swarm_load_carry.state import State, CS_type

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
        #logger.warn(
          #  f'Could not transform {source_frame} to {target_frame}: {ex}') #TODO: reactivate
          pass
    
    return t


# Transform a position from one frame into another
def transform_position(pos_frame_1: np.ndarray[(3,), float], pos_frame_2_rel_frame_1: np.ndarray[(3,), float]): 
    pos_frame_2 = pos_frame_1 + pos_frame_2_rel_frame_1

    return pos_frame_2

# Transform an orientation (given by a quaternion) from frame1 to frame2
def transform_orientation(q_frame_1, q_frame_2_rel_frame_1):
    q_frame_2 = q_frame_2_rel_frame_1*q_frame_1

    return q_frame_2

# Create a new state object that represents an input state transformed into frame2 
def transform_frames(state, frame2_name, tf_buffer, logger):
    state2 = State(f'{frame2_name}', CS_type.ENU)

    # Find the transform
    tf_f2_rel_f2 = lookup_tf(frame2_name, state.frame, tf_buffer, rclpy.time.Time(), logger)

    # Make the transform if the frames exist, otherwise return None
    if tf_f2_rel_f2 == None:
        return None

    else:
        state2.pos = transform_position(state.pos, np.array([tf_f2_rel_f2.transform.translation.x, 
                                                            tf_f2_rel_f2.transform.translation.y, 
                                                            tf_f2_rel_f2.transform.translation.z]))
        
        state2.att_q = transform_orientation(state.att_q, qt.array([tf_f2_rel_f2.transform.rotation.w, 
                                                                    tf_f2_rel_f2.transform.rotation.x,
                                                                    tf_f2_rel_f2.transform.rotation.y,
                                                                    tf_f2_rel_f2.transform.rotation.z]))
        
    return state2


## TRAJECTORY GENERATION
# Note trajectories sent to Pixhawk controller must be in NED co-ordinates relative to initial drone position. ENU -> NED and frame transformations handled here

# Make drone follow desired load position, at the desired location relative to the load
# TODO: Add interpolation (especially for rotation/yaw of swarm)
def gen_traj_msg_circle_load(vehicle_desired_state_rel_load, load_desired_local_state, drone_name, tf_buffer, timestamp, logger):
    # Generate trajectory message
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp
    
    # Convert load desired state into world frame
    load_desired_state_rel_world = transform_frames(load_desired_local_state, 'world', tf_buffer, logger)

    # Approximate load desired position as relative to world rather than load_init if load_init -> world tf not available
    if load_desired_state_rel_world == None:
        #logger.warn(f'Load initial position not found. Returning default trajectory msg.') #TODO: reactivate
        return trajectory_msg

    # Add effect of desired load orientation (note different physical connections to load will require different algorithms)
    vehicle_desired_pos_rel_load_rot = load_desired_state_rel_world.att_q.rotate(vehicle_desired_state_rel_load.pos)

    # Desired vehicle pos relative to world (in ENU)= desired vehicle pos relative to load + desired load pos rel to world 
    vehicle_desired_pos_rel_world = [vehicle_desired_pos_rel_load_rot[0] + load_desired_state_rel_world.pos[0], 
                                            vehicle_desired_pos_rel_load_rot[1] + load_desired_state_rel_world.pos[1],
                                            vehicle_desired_pos_rel_load_rot[2] + load_desired_state_rel_world.pos[2]]

    # Tranform relative to drone_init
    vehicle_desired_state_rel_world = State('world', CS_type.ENU)
    vehicle_desired_state_rel_world.pos = vehicle_desired_pos_rel_world
    vehicle_desired_state_rel_world.att_q = vehicle_desired_state_rel_load.att_q*load_desired_state_rel_world.att_q

    vehicle_desired_state_rel_drone_init = transform_frames(vehicle_desired_state_rel_world, f'{drone_name}_init', tf_buffer, logger)

    
    # Approximate vehicle desired position as relative to world rather than drone_init if drone_init -> world not available
    if vehicle_desired_state_rel_drone_init == None:
        #logger.warn(f'Drone initial position not found. Returning default trajectory msg.') #TODO: reactivate
        return trajectory_msg
        
    # Transform to NED into drone_init
    pos_drone_desired_px4 = np.array([vehicle_desired_state_rel_drone_init.pos[1], vehicle_desired_state_rel_drone_init.pos[0], -vehicle_desired_state_rel_drone_init.pos[2]])
    q_drone_desired_px4 = ft.ros_to_px4_orientation(q_to_normalized_np(vehicle_desired_state_rel_drone_init.att_q))

    # Generate trajectory message
    trajectory_msg.position[0] = pos_drone_desired_px4[0] 
    trajectory_msg.position[1] = pos_drone_desired_px4[1] 
    trajectory_msg.position[2] = pos_drone_desired_px4[2] 

    trajectory_msg.yaw = ft.quaternion_get_yaw(q_drone_desired_px4)

    return trajectory_msg


# Generate trajectory message that gives setpoint at a specific height and orientation
def gen_traj_msg_straight_up(takeoff_height, takeoff_q, timestamp):
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp

    # Set position straight up
    trajectory_msg.position[0] = 0.0
    trajectory_msg.position[1] = 0.0
    trajectory_msg.position[2] = -takeoff_height

    # Get yaw in NED from takeoff_q
    q_px4 = ft.ros_to_px4_orientation(q_to_normalized_np(takeoff_q))
    trajectory_msg.yaw = ft.quaternion_get_yaw(q_px4)

    return trajectory_msg

# Make drone orbit in a circle of radius r and height h
def gen_traj_msg_orbit(r, theta, h, timestamp):
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp

    trajectory_msg.position[0] = r * np.cos(theta)
    trajectory_msg.position[1] = r * np.sin(theta)
    trajectory_msg.position[2] = -h

    return trajectory_msg

# Make drone travel at set velocity
def gen_traj_msg_vel(desired_velocity, timestamp):
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp

    trajectory_msg.position[0] = None
    trajectory_msg.position[1] = None
    trajectory_msg.position[2] = None

    # Convert ENU-> NED
    trajectory_msg.velocity[0] = desired_velocity[1]
    trajectory_msg.velocity[1] = desired_velocity[0]
    trajectory_msg.velocity[2] = -desired_velocity[2]

    return trajectory_msg