import numpy as np
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
    def __init__(self, name, env, load_pose_type, evaluate, tf_broadcaster):
        # PARAMETERS
        self.name = name
        self.env = env
        self.load_pose_type = load_pose_type
        self.evaluate = evaluate

        # STATES
        self.initial_global_state = State('globe', CS_type.LLA)
        self.local_state = State('init', CS_type.ENU)

        ## TFS
        self.tf_broadcaster = tf_broadcaster

        # FLAGS
        self.flag_gps_home_set = False # GPS home set when vehicle armed
        self.flag_local_init_pose_set = False

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
            if self.env == 'phys' and (self.load_pose_type == 'ground_truth' or self.evaluate == True) and self.flag_local_init_pose_set:
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