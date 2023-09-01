import math
#import quaternionic as quaternion
import quaternion 
import rclpy

import numpy as np
#from numpy.typing 

import frame_transforms as ft

from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import TrajectorySetpoint
from tf2_ros import TransformException

from swarm_load_carry.state import State, CS_type
from swarm_load_carry_interfaces.srv import PhaseChange

## STRING HANDLING 

# Extract the an instance number from a ROS publisher/subscriber/client/service
def extract_instance_from_connection(connection):
    topic_name = connection.topic_name

    namespace = topic_name.split('/')[1]
    instance_num = namespace.split('_')[1]

    return int(instance_num)

## CONVERSIONS
# Normalize a quaternion and return the equivalent numpy representation
def q_to_normalized_np(q: np.quaternion): #quaternion.array):
    q_norm = q.normalized()

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
    

def generate_points_cylinder(num_points, r, z):
    positions = np.zeros((num_points, 3))

    # Use polar co-ordinate system to generate (x,y) points in circumscribing circle 
    for i in range(num_points):
        positions[i] = np.array([r*math.cos(2*math.pi*i/num_points), r*math.sin(2*math.pi*i/num_points), z[i]])
    
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
        logger.warn(f'Could not find transform: {source_frame} to {target_frame}: {ex}')
    
    return t


# Transform a position of an item (A) from one frame (B) into another (C) given the position of the item in frame B (p_BA), position of B rel C (p_CB)
# and the orientation of B rel C (q_CB)
def transform_position(p_BA: np.ndarray[(3,), float], p_CB: np.ndarray[(3,), float], q_CB, logger):
    # Perform translation between frames that are both rotated and translated 
    #p_CA = q_CB.rotate(p_BA) + p_CB
    #p_BA_quat = np.quaternion(0, *p_BA)
    p_BA_rotated = q_CB*(np.quaternion(0, *p_BA))*q_CB.inverse()
    p_CA = np.array([p_BA_rotated.x, p_BA_rotated.y, p_BA_rotated.z]) + p_CB

    # logger.info(f'p_BA: {p_BA}')
    # logger.info(f'q_CB: {q_CB}')
    # logger.info(f'p_BA_quat: {p_BA_quat}')
    # logger.info(f'p_BA_rotated: {p_BA_rotated}')
    # logger.info(f'p_CB: {p_CB}')
    # logger.info(f'p_CA: {p_CA}')

    return p_CA

# Transform an orientation A (given by a quaternion) from frame B to frame C
def transform_orientation(q_BA, q_CB): 
    q_CA = q_CB*q_BA

    return q_CA


# Create a new state object that represents an input state transformed into frame2 
def transform_frames(state, frame2_name, tf_buffer, logger):
    state2 = State(f'{frame2_name}', CS_type.ENU)

    # Find the transform
    tf_f1_rel_f2 = lookup_tf(frame2_name, state.frame, tf_buffer, rclpy.time.Time(), logger)

    # Make the transform if the frames exist, otherwise return None
    if tf_f1_rel_f2 == None:
        return None

    else:
        # Collect transformation vector and quaternion
        p_f2f1 = np.array([tf_f1_rel_f2.transform.translation.x, 
                            tf_f1_rel_f2.transform.translation.y, 
                            tf_f1_rel_f2.transform.translation.z])
        
        q_f2f1 = np.quaternion(tf_f1_rel_f2.transform.rotation.w, 
                          tf_f1_rel_f2.transform.rotation.x,
                          tf_f1_rel_f2.transform.rotation.y,
                          tf_f1_rel_f2.transform.rotation.z)

        # Perform transform
        state2.pos = transform_position(state.pos, p_f2f1, q_f2f1, logger)
        state2.att_q = transform_orientation(state.att_q, q_f2f1)

        logger.info(f'state2.pos: {state2.pos}')
        logger.info(f'state2.att_q: {state2.att_q}')
        
    return state2


## TRAJECTORY GENERATION
# Note trajectories sent to Pixhawk controller must be in NED co-ordinates relative to initial drone position. ENU -> NED and frame transformations handled here

# Make drone follow desired load position, at the desired location relative to the load
def gen_traj_msg_circle_load(vehicle_desired_state_rel_load, load_desired_local_state, drone_name, tf_buffer, timestamp, logger):
    # Generate trajectory message
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp
    
    ## GET LOAD DESIRED STATE
    # Convert load desired state into world frame
    load_desired_state_rel_world = transform_frames(load_desired_local_state, 'world', tf_buffer, logger)
    
    if load_desired_state_rel_world == None:
        return None

    ## GET VEHICLE DESIRED STATE   
    # Note: cannot use transform_frames() directly as requires load desired, not actual current TF
    vehicle_desired_state_rel_world = State('world', CS_type.ENU)
    
    vehicle_desired_state_rel_world.pos = transform_position(vehicle_desired_state_rel_load.pos, load_desired_state_rel_world.pos, load_desired_state_rel_world.att_q, logger) #transform_position(load_desired_state_rel_world.pos, vehicle_desired_pos_rel_load_rot, )
    vehicle_desired_state_rel_world.att_q = transform_orientation(vehicle_desired_state_rel_load.att_q, load_desired_state_rel_world.att_q)                             #load_desired_state_rel_world.att_q, vehicle_desired_state_rel_load.att_q)

    # Transform relative to drone_init
    vehicle_desired_state_rel_drone_init = transform_frames(vehicle_desired_state_rel_world, f'{drone_name}_init', tf_buffer, logger)
    

    ## CONVERT TO TRAJECTORY MSG
    if vehicle_desired_state_rel_drone_init == None:
        return None
        
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


## DRONE CONTROL
# Change the phase of a drone
def phase_change(cli_phase_change, phase_desired):
    # Prepare request
    phase_req = PhaseChange.Request()
    phase_req.phase_request.phase = phase_desired
    
    # Send request
    phase_future = cli_phase_change.call_async(phase_req)

    return phase_future

def change_phase_all_drones(node, num_drones, cli_array_phase_change, phase_desired):
    # Send request
    phase_future = [None] * num_drones

    for i in range(num_drones):
        phase_future[i] = phase_change(cli_array_phase_change[i], phase_desired)

    # Wait for response
    for i in range(num_drones):
        rclpy.spin_until_future_complete(node, phase_future[i])