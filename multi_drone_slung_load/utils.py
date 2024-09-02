import math
#import quaternionic as quaternion
import numpy as np
import quaternion 
import rclpy

#from numpy.typing 
import frame_transforms as ft

from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import TrajectorySetpoint
from tf2_ros import TransformException

from multi_drone_slung_load.state import State, CS_type
from multi_drone_slung_load_interfaces.srv import PhaseChange

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

# Extract a particular pose from a PoseArray message
#TODO: Improve this to be more general/not require known index
def extract_pose_from_pose_array_msg(pose_array, index):
    return pose_array.poses[index]


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


def regular_polygon_side_length(num_sides, radius):
    return radius*(2*(1-math.cos(2*math.pi/num_sides)))**0.5


# Find the height a drone should be relative to the load connection point to give the desired formation radius
# Inputs: Cable length, drone formation radius, radius of cable attachment points
# Outputs: height
def drone_height_rel_load(cable_length, r_formation, r_cable_attachments, height_offset):
    return math.sqrt(cable_length**2 - (r_formation-r_cable_attachments)**2) + height_offset


## RELATIVE POSES

# Find a normalized direction vector between two points (p2 and p1) specified in the same frame
def get_norm_dir_vec(p1, p2):
    p_12 = p2 - p1
    p_hat_12 = p_12/np.linalg.norm(p_12)

    return p_hat_12


# Find the direction of yaw turn that traverses the smaller angle (1 = cw, -1 = ccw)
def find_shortest_yaw_dir(yaw1, yaw2):
    # Find the sign of the relative yaw
    yaw_diff = yaw2-yaw1
    dir = np.sign(yaw_diff)

    # Change the direction of turn if the yaw is over pi rad
    if yaw_diff > math.pi:
        dir = -1*dir

    return dir


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
def lookup_tf(target_frame, source_frame, tf_buffer, time, logger, print_warn=True):
    t = None
    
    try:
        t = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            time)
    except TransformException as ex:
        if print_warn:
            logger.warn(f'Could not find frame transform: {target_frame} to {source_frame}: {ex}')
        pass
    
    return t


# Transform a position of an item (A) from one frame (B) into another (C) given the position of the item in frame B (p_BA), position of B rel C (p_CB)
# and the orientation of B rel C (q_CB)
#def transform_position(p_BA: np.ndarray[(3,), float], p_CB: np.ndarray[(3,), float], q_CB, logger):
def transform_position(p_BA, p_CB, q_CB, logger):
    # Perform translation between frames that are both rotated and translated 
    p_BA_rotated = q_CB*(np.quaternion(0, *p_BA))*q_CB.inverse()
    p_CA = np.array([p_BA_rotated.x, p_BA_rotated.y, p_BA_rotated.z]) + p_CB

    return p_CA

# Transform an orientation A (given by a quaternion) from frame B to frame C
def transform_orientation(q_BA, q_CB): 
    q_CA = q_CB*q_BA

    return q_CA


# Create a new state object that represents an input state transformed into frame2 
def transform_frames(state, frame2_name, tf_buffer, logger, cs_out_type=CS_type.XYZ, print_warn=True):
    state2 = State(f'{frame2_name}', cs_out_type)

    # Find the transform
    tf_f1_rel_f2 = lookup_tf(frame2_name, state.frame, tf_buffer, rclpy.time.Time(), logger, print_warn)

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

    return state2


## FEEDBACK
def get_drone_poses(num_drones, first_drone_num, tf_buffer, logger, print_warn=True):
    drone_positions = np.zeros((num_drones, 3))
    drone_orientations = np.array([np.quaternion(*q) for q in np.zeros((num_drones, 4))])

    # Store position and orientation of each drone relative to world
    count_tf = 0

    for i in range(num_drones):
        target_frame = 'world'
        source_frame = f'drone{i+first_drone_num}'
        
        t = lookup_tf(target_frame, source_frame, tf_buffer, rclpy.time.Time(), logger, print_warn)

        if t != None:
            drone_positions[i, :] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            drone_orientations[i] = np.quaternion(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z)

            count_tf += 1

    return drone_positions, drone_orientations, count_tf


## TRAJECTORY GENERATION
# Note trajectories sent to Pixhawk controller must be in NED co-ordinates relative to initial drone position. ENU -> NED and frame transformations handled here

# Project velocity, acceleration etc. setpoints onto straight lines connecting the previous and desired positions
# In ROS2 coordinates (ENU)
def gen_rate_setpoints_straight(drone_desired_local_state, drone_prev_local_state, v_scalar=None, a_scalar=None, yawspeed_scalar=None):
    rate_desired_setpoints_ros2 = [None, None, None]
    
    # Find direction of motion (straight line between previous and current position)
    dir_desired = get_norm_dir_vec(drone_prev_local_state.pos, drone_desired_local_state.pos)
        
    # Velocity
    if v_scalar is not None:
        v_local = v_scalar*dir_desired

        rate_desired_setpoints_ros2[0] = np.copy(v_local)
    
    # Acceleration
    if a_scalar is not None:
        a_local = a_scalar*dir_desired

        rate_desired_setpoints_ros2[1] = np.copy(a_local)
    
    # Yawspeed
    if yawspeed_scalar is not None:
        # Select direction based on the minimum angle to turn
        drone_prev_local_state_np = np.array([drone_prev_local_state.att_q.w, drone_prev_local_state.att_q.x, drone_prev_local_state.att_q.y, drone_prev_local_state.att_q.z])
        drone_desired_local_state_np = np.array([drone_desired_local_state.att_q.w, drone_desired_local_state.att_q.x, drone_desired_local_state.att_q.y, drone_desired_local_state.att_q.z])
        dir = find_shortest_yaw_dir(ft.quaternion_get_yaw(drone_prev_local_state_np), ft.quaternion_get_yaw(drone_desired_local_state_np))

        # Convert to PX4 coordinates (NED)
        rate_desired_setpoints_ros2[2] = -1*dir*yawspeed_scalar

    return rate_desired_setpoints_ros2


# Add rate setpoints to a trajectory message that has already been generated for a position
def traj_msg_add_rate_setpoints(traj_msg, rate_desired_setpoints_ros2):
    # Note: convert all ROS2 to PX4 coordinates
    # Velocity
    if rate_desired_setpoints_ros2[0] is not None:
        traj_msg.velocity[0] = rate_desired_setpoints_ros2[0][1]
        traj_msg.velocity[1] = rate_desired_setpoints_ros2[0][0]
        traj_msg.velocity[2] = -rate_desired_setpoints_ros2[0][2]
    # Acceleration
    if rate_desired_setpoints_ros2[1] is not None:
        traj_msg.acceleration[0] = rate_desired_setpoints_ros2[1][1]
        traj_msg.acceleration[1] = rate_desired_setpoints_ros2[1][0]
        traj_msg.acceleration[2] = -rate_desired_setpoints_ros2[1][2]
    # Angular velocity
    if rate_desired_setpoints_ros2[2] is not None:
        traj_msg.yawspeed = rate_desired_setpoints_ros2[2]
    
    return traj_msg


# Make drone follow desired load position, at the desired location relative to the load
def gen_traj_msg_circle_load(vehicle_desired_state_rel_load, load_desired_local_state, drone_name, tf_buffer, timestamp, logger, timestamp_msg=None, drone_prev_local_state=None, v_scalar=None, a_scalar=None, yawspeed_scalar=None, print_warn=True, tf_broadcaster=None):
    # Generate trajectory message
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp
    
    ## GET LOAD DESIRED STATE
    # Convert load desired state into world frame
    load_desired_state_rel_world = transform_frames(load_desired_local_state, 'world', tf_buffer, logger, cs_out_type=CS_type.ENU, print_warn=print_warn)
    
    if load_desired_state_rel_world == None:
        return None

    ## GET VEHICLE DESIRED STATE   
    # Note: cannot use transform_frames() directly as requires load desired, not actual current TF
    vehicle_desired_state_rel_world = State('world', CS_type.ENU)
    
    vehicle_desired_state_rel_world.pos = transform_position(vehicle_desired_state_rel_load.pos, load_desired_state_rel_world.pos, load_desired_state_rel_world.att_q, logger) #transform_position(load_desired_state_rel_world.pos, vehicle_desired_pos_rel_load_rot, )
    vehicle_desired_state_rel_world.att_q = transform_orientation(vehicle_desired_state_rel_load.att_q, load_desired_state_rel_world.att_q)                             #load_desired_state_rel_world.att_q, vehicle_desired_state_rel_load.att_q)

    if tf_broadcaster is not None:    
        broadcast_tf(timestamp_msg, vehicle_desired_state_rel_world.frame, f'{drone_name}_d', vehicle_desired_state_rel_world.pos, vehicle_desired_state_rel_world.att_q, tf_broadcaster)

    # Transform relative to drone_init
    vehicle_desired_state_rel_drone_init = transform_frames(vehicle_desired_state_rel_world, f'{drone_name}_init', tf_buffer, logger, cs_out_type=CS_type.ENU, print_warn=print_warn)
    

    ## CONVERT TO TRAJECTORY MSG
    if vehicle_desired_state_rel_drone_init == None:
        return None
        
    # Transform to PX4 coordinates (NED)
    pos_drone_desired_px4 = np.array([vehicle_desired_state_rel_drone_init.pos[1], vehicle_desired_state_rel_drone_init.pos[0], -vehicle_desired_state_rel_drone_init.pos[2]])
    q_drone_desired_px4 = ft.ros_to_px4_orientation(q_to_normalized_np(vehicle_desired_state_rel_drone_init.att_q))

    ## GENERATE TRAJECTORY MSG
    # Position
    trajectory_msg.position[0] = pos_drone_desired_px4[0] 
    trajectory_msg.position[1] = pos_drone_desired_px4[1] 
    trajectory_msg.position[2] = pos_drone_desired_px4[2] 

    # Yaw
    trajectory_msg.yaw = ft.quaternion_get_yaw(q_drone_desired_px4)

    # Add velocity and acceleration targets if applicable trajectory
    if drone_prev_local_state != None:
        rate_desired_setpoints_ros2 = gen_rate_setpoints_straight(vehicle_desired_state_rel_drone_init, drone_prev_local_state, v_scalar, a_scalar, yawspeed_scalar)
        trajectory_msg = traj_msg_add_rate_setpoints(trajectory_msg, rate_desired_setpoints_ros2)

    return trajectory_msg


# Generate trajectory message that gives setpoint at a specific height and orientation
def gen_traj_msg_straight_up(takeoff_height, takeoff_q, timestamp, takeoff_N=0.0, takeoff_E=0.0, drone_name = None, drone_prev_local_state=None, v_scalar=None, a_scalar=None, timestamp_msg = None, print_warn = True, logger = None, tf_buffer = None, tf_broadcaster=None):
    trajectory_msg = TrajectorySetpoint()
    trajectory_msg.timestamp = timestamp

    # Set position straight up
    trajectory_msg.position[0] = takeoff_N
    trajectory_msg.position[1] = takeoff_E
    trajectory_msg.position[2] = -takeoff_height

    # Get yaw in NED from takeoff_q
    q_px4 = ft.ros_to_px4_orientation(q_to_normalized_np(takeoff_q))
    trajectory_msg.yaw = ft.quaternion_get_yaw(q_px4)

    # Construct desired state (position as orientation is unchanged for straight up)
    vehicle_desired_local_state = State(f'droneX_init', CS_type.ENU)
    vehicle_desired_local_state.pos = np.array([takeoff_E, takeoff_N, takeoff_height])
    vehicle_desired_local_state.att_q = takeoff_q
    
    # Add rate targets if applicable
    if drone_prev_local_state is not None:
        rate_desired_setpoints_ros2 = gen_rate_setpoints_straight(vehicle_desired_local_state, drone_prev_local_state, v_scalar, a_scalar, None)
        trajectory_msg = traj_msg_add_rate_setpoints(trajectory_msg, rate_desired_setpoints_ros2)

    if tf_broadcaster is not None:    
        vehicle_desired_state_rel_world = transform_frames(vehicle_desired_local_state, 'world', tf_buffer, logger, cs_out_type=CS_type.ENU, print_warn=print_warn)
        
        if vehicle_desired_state_rel_world != None:
            broadcast_tf(timestamp_msg, vehicle_desired_state_rel_world.frame, f'{drone_name}_d', vehicle_desired_state_rel_world.pos, vehicle_desired_state_rel_world.att_q, tf_broadcaster)

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

# Can be used to send a phase change request to all clients in an array
# Often used for sending phase change requests directly to drones, or through the GCS to the drones
def change_phase_all(node, cli_array_phase_change, phase_desired, wait_for_response=True):
    num_clients = len(cli_array_phase_change)
    
    # Send request
    phase_future = [None] * num_clients

    for i in range(num_clients):
        phase_future[i] = phase_change(cli_array_phase_change[i], phase_desired)

    # Wait for response
    if wait_for_response:
        for i in range(num_clients):
            rclpy.spin_until_future_complete(node, phase_future[i])
            