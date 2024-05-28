import numpy as np
import pymap3d as pm
import quaternion
import utils
import swarm_load_carry.drone_offboard_ros as offboard_ros
from swarm_load_carry.state import State, CS_type
import frame_transforms as ft
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition
from swarm_load_carry_interfaces.msg import Phase, GlobalPose

# Class to handle the vehicle's pose and state, both global and local.
# This includes storing the state variables, publishing the TFs, and handling the callbacks for the vehicle's pose.
class PosePixhawk:
    def __init__(self, name, env, load_pose_type, evaluate, logger, tf_broadcaster, tf_static_broadcaster_init_pose, tf_static_broadcaster_item2_rel_item1=None, tf_static_broadcaster_item2_rel_item1_gt=None, tf_static_broadcaster_world_rel_gt=None):
        # PARAMETERS
        self.name = name
        self.id = int(name[-1])
        self.env = env
        self.load_pose_type = load_pose_type
        self.evaluate = evaluate

        # STATES
        self.global_origin_state = State('globe', CS_type.LLA)
        self.global_origin_state_prev = self.global_origin_state.copy()

        self.initial_global_state = State('globe', CS_type.LLA)
        self.initial_state_rel_world = State('world', CS_type.ENU)
        self.local_state = State(f'{self.name}_init', CS_type.ENU)

        ## TFS
        self.tf_broadcaster = tf_broadcaster

        self.tf_static_broadcaster_init_pose = tf_static_broadcaster_init_pose
        self.tf_static_broadcaster_world_rel_gt = tf_static_broadcaster_world_rel_gt
        self.tf_static_broadcaster_item2_rel_item1 = tf_static_broadcaster_item2_rel_item1 
        self.tf_static_broadcaster_item2_rel_item1_gt = tf_static_broadcaster_item2_rel_item1_gt

        # FLAGS
        self.flag_gps_home_set = False # GPS home set when vehicle armed
        self.flag_local_init_pose_set = False

        # MISC
        self.logger = logger


    ## CALLBACKS
    def clbk_vehicle_attitude(self, msg, current_time):
        # Original q from FRD->NED
        # Handles FRD->NED to FLU->ENU transformation 
        q_px4 = utils.q_to_normalized_np(np.quaternion(msg.q[0], msg.q[1], msg.q[2], msg.q[3]))
        q_ros = ft.px4_to_ros_orientation(q_px4)

        self.local_state.att_q.w = q_ros[0]
        self.local_state.att_q.x = q_ros[1] 
        self.local_state.att_q.y = q_ros[2]   
        self.local_state.att_q.z = q_ros[3] 

        if not self.flag_gps_home_set:
            # Set the initial attitude as the current attitude
            self.initial_global_state.att_q = self.local_state.att_q.copy()

        # Update tf
        # if not (np.isnan(self.local_state.pos[0])):
        #     utils.broadcast_tf(current_time, f'{self.name}_init', f'{self.name}', self.local_state.pos, self.local_state.att_q, self.tf_broadcaster)


    def clbk_vehicle_local_position(self, msg, current_time):
        # Handles NED->ENU transformation 
        self.local_state.pos[0] = msg.y 
        self.local_state.pos[1] = msg.x 
        self.local_state.pos[2] = -msg.z
        self.local_state.vel[0] = msg.vy 
        self.local_state.vel[1] = msg.vx 
        self.local_state.vel[2] = -msg.vz

        # Publish TF
        if not (np.isnan(self.local_state.att_q.x)):
            # Update tf
            utils.broadcast_tf(current_time, f'{self.name}_init', f'{self.name}', self.local_state.pos, self.local_state.att_q, self.tf_broadcaster)

            # If in the real world, and ground truth is on, use the vehicle local position to publish ground truth
            if self.env == 'phys' and (self.load_pose_type == 'ground_truth' or self.evaluate == True): #and self.flag_local_init_pose_set:
                # Note how the TF tree will look different in the real world because the ground truth is published relative to the local init pose rather than directly from the ground truth.
                # This is OK because the lookups will still work.
                utils.broadcast_tf(current_time, f'{self.name}_init', f'{self.name}_gt', self.local_state.pos, self.local_state.att_q, self.tf_broadcaster)
                

    def clbk_vehicle_global_position(self, msg, phase_true, current_time, pub_vehicle_command, pub_global_init_pose=None):
        # Set GPS/location home immediately prior to first arming/takeoff
        if not self.flag_gps_home_set and phase_true: #(phase == Phase.PHASE_SETUP_DRONE):          
            # Set the initial position as the current global position 
            # (could average over last few samples but GPS seems to have low frequency noise of about 0.1m -> can turn down in simulation if required)
            self.initial_global_state.pos[0] = msg.lat
            self.initial_global_state.pos[1] = msg.lon 
            self.initial_global_state.pos[2] = msg.alt

            offboard_ros.set_origin(pub_vehicle_command, msg.lat, msg.lon, msg.alt, current_time) #int(self.get_clock().now().nanoseconds/1000)

            # Publish this information if set to do so
            if pub_global_init_pose is not None:
                msg_global_pose = GlobalPose()

                msg_global_pose.global_pos.lat = float(self.initial_global_state.pos[0])
                msg_global_pose.global_pos.lon = float(self.initial_global_state.pos[1])
                msg_global_pose.global_pos.alt = float(self.initial_global_state.pos[2])

                msg_global_pose.global_att.q[0] = float(self.initial_global_state.att_q.w)
                msg_global_pose.global_att.q[1] = float(self.initial_global_state.att_q.x)
                msg_global_pose.global_att.q[2] = float(self.initial_global_state.att_q.y)
                msg_global_pose.global_att.q[3] = float(self.initial_global_state.att_q.z)

                pub_global_init_pose.publish(msg_global_pose)

            self.flag_gps_home_set = True   

    def clbk_global_origin(self, msg):
        self.global_origin_state.pos[0] = msg.global_pos.lat
        self.global_origin_state.pos[1] = msg.global_pos.lon
        self.global_origin_state.pos[2] = msg.global_pos.alt

        self.global_origin_state.att_q.w = msg.global_att.q[0]
        self.global_origin_state.att_q.x = msg.global_att.q[1]
        self.global_origin_state.att_q.y = msg.global_att.q[2]
        self.global_origin_state.att_q.z = msg.global_att.q[3]

        # Global origin updated - must update local initial poses
        self.flag_local_init_pose_set = False

    ## HELPER FUNCTIONS
    def reset(self):
        self.flag_gps_home_set = False
        self.flag_local_init_pose_set = False


    def broadcast_tf_init_pose(self, time, item2_name=None, t_item2_rel_item1=None, R_item2_rel_item1=None):
        # Publish static transform for init pose (relative to world)
        # As all init CS are in ENU, they are all aligned in orientation
        utils.broadcast_tf(time, 'world', f'{self.name}_init', self.initial_state_rel_world.pos, np.quaternion(1.0, 0.0, 0.0, 0.0), self.tf_static_broadcaster_init_pose) #self.get_clock().now().to_msg() self.tf_static_broadcaster_init_pose

        # Publish other static transforms
        # Camera relative to drone or marker relative to load etc.
        #if item2_name is not None and t_item2_rel_item1 is not None and R_item2_rel_item1 is not None:
        self.logger.info(f'item2_name: {item2_name}, t_item2_rel_item1: {t_item2_rel_item1}, R_item2_rel_item1: {R_item2_rel_item1}') # TODO: Remove
        self.logger.info('PUB ITEM 2 REL ITEM 1 TF -  START') # TODO: Remove

        q_list = ft.quaternion_from_euler(R_item2_rel_item1[0], R_item2_rel_item1[1], R_item2_rel_item1[2])
        r_cam_rel_pixhawk = np.quaternion(*q_list)
        utils.broadcast_tf(time, f'{self.name}', f'{item2_name}{self.id}', t_item2_rel_item1, r_cam_rel_pixhawk, self.tf_static_broadcaster_item2_rel_item1)
        
        # Also broadcast ground truth if evaluating or if this is the load and ground truth feedback is being used
        if self.evaluate == True or (('load' in self.name) and self.load_pose_type == 'ground_truth'):  
            utils.broadcast_tf(time, f'{self.name}_gt', f'{item2_name}{self.id}_gt', t_item2_rel_item1, r_cam_rel_pixhawk, self.tf_static_broadcaster_item2_rel_item1_gt)

            self.logger.info('PUB ITEM 2 REL ITEM 1 TF -  GT') # TODO: Remove

        self.logger.info('PUB ITEM 2 REL ITEM 1 TF -  END') # TODO: Remove

        # Send complete message
        self.flag_local_init_pose_set = True 
        self.logger.info('Local init pose set') #self.get_logger().info('Local init pose set')


    # Set initial pose relative to world and broadcasts tf
    # t_init: initial position in ENU (np.array)
    # q_init: initial orientation in quaternion (np.quaternion)
    def _set_local_init_pose(self, t_init, q_init, time, item2_name=None, t_item2_rel_item1=None, R_item2_rel_item1=None):
        # Set local initial state
        self.initial_state_rel_world.pos = np.copy(t_init)
        self.initial_state_rel_world.att_q = q_init.copy()

        # Broadcast tf
        self.broadcast_tf_init_pose(time, item2_name, t_item2_rel_item1, R_item2_rel_item1)

    
    # Set the local initial position of the Pixhawk used as the reference/world frame
    def set_local_init_pose_ref(self, time, cs_offset=np.array([0.0, 0.0, 0.0]), state_gt=None, item2_name=None, t_item2_rel_item1=None, R_item2_rel_item1=None):
        # Set local init pose (relative to base/world CS. Note that this defines the reference CS)
        self._set_local_init_pose(cs_offset, self.initial_global_state.att_q, time, item2_name, t_item2_rel_item1, R_item2_rel_item1) #np.array([0.0, 0.0, self.height_drone_cs_rel_gnd])

        # If using ground truth, set transform from ground truth to world
        if self.load_pose_type == 'ground_truth' or self.evaluate == True:
            # Ground truth origin set by Gazebo in simulation
            if self.env == 'sim':
                # World frame is set cooincident with the first drone's initial pose, which is oriented in ENU. 
                # Rotate drone 1's attitude measured in ground truth into ENU
                q_initial_state_rel_world_inv = self.initial_state_rel_world.att_q.inverse()
                state_gt_att_rotated = utils.transform_orientation(q_initial_state_rel_world_inv, state_gt.att_q)
                utils.broadcast_tf(time, 'ground_truth', 'world', state_gt.pos, state_gt_att_rotated, self.tf_static_broadcaster_world_rel_gt) #self.get_clock().now().to_msg() self.vehicle_state_gt.pos, self.vehicle_state_gt.att_q, self.tf_static_broadcaster_world_rel_gt)
            elif self.env == 'phys':
                # Set ground truth origin at world origin in physical environment
                utils.broadcast_tf(time, 'ground_truth', 'world', np.array([0.0, 0.0, 0.0]), np.quaternion(1.0, 0.0, 0.0, 0.0), self.tf_static_broadcaster_world_rel_gt)     
     
    # Set the local initial position for all other Pixhawks
    def set_local_init_pose_non_ref(self, time, initial_state_rel_world=None, cs_offset=np.array([0.0, 0.0, 0.0]), item2_name=None, t_item2_rel_item1=None, R_item2_rel_item1=None): #set_local_init_pose_later_drones(self):
        # If we are given the initial state relative to the world, use that
        if initial_state_rel_world is not None:
            self._set_local_init_pose(initial_state_rel_world.pos, initial_state_rel_world.att_q, time, item2_name, t_item2_rel_item1, R_item2_rel_item1)
        else:
            ## Set initial pose relative to first drone's initial pose
            origin_state_lla = self.global_origin_state 

            # Perform transformation
            trans_E, trans_N, trans_U = pm.geodetic2enu(self.initial_global_state.pos[0], self.initial_global_state.pos[1], self.initial_global_state.pos[2], origin_state_lla.pos[0], origin_state_lla.pos[1], origin_state_lla.pos[2]) 
            
            # Set local init pose (relative to base CS)
            initial_t = np.array([trans_E, trans_N, trans_U]) + cs_offset

            self._set_local_init_pose(initial_t, self.initial_global_state.att_q, time, t_item2_rel_item1, R_item2_rel_item1)
