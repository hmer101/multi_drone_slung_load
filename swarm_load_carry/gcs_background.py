# Contains the GCS (ground control station) background class and related methods.
# This class runs the commands to place the load and drones at the desired pose as determined by the gcs_user node.
#
# Author: Harvey Merton
# Date: 01/06/2023

import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

import numpy as np
import quaternion as quaternion

#import pymap3d as pm

import utils

import frame_transforms as ft
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from swarm_load_carry.state import State, CS_type

from swarm_load_carry_interfaces.srv import SetLocalPose, GetGlobalInitPose, PhaseChange
from swarm_load_carry_interfaces.msg import Phase

from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint

class GCSBackground(Node):

    def __init__(self):
        super().__init__('gcs_background')

        ## PARAMETERS
        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.declare_parameter('num_drones', 3)
        self.declare_parameter('first_drone_num', 1)
        self.declare_parameter('load_id', 1)
        self.declare_parameter('drone_order', [1, 2, 3])
        self.declare_parameter('kp_formation_load', 0.0)
        self.declare_parameter('r_drones_max_safety', 0.1)
        self.declare_parameter('auto_level', 0)
        self.declare_parameter('phase_change_requests_through_background', True)

        #self.declare_parameter('height_drone_rel_load', 1.5)
        self.declare_parameter('takeoff_height_load_max', 4.0)
        self.declare_parameter('r_drones_rel_load', 1.0)
        self.declare_parameter('d_drones_rel_min', 1.0)
        self.declare_parameter('d_drones_rel_max', 1.5)
        self.declare_parameter('drone_yaws_rel_load', [3.141592654, -1.0471975512, 1.0471975512])

        self.declare_parameter('cable_length', 1.0)
        self.declare_parameter('load_connection_point_r', 0.0)

        self.declare_parameter('mission_circle_r', 2.0)
        self.declare_parameter('mission_circle_v', 0.5)

        self.declare_parameter('vel_load_vertical_slow', 0.05)
        self.declare_parameter('vel_load_vertical_fast', 0.3)
        self.declare_parameter('yawspeed_load', 0.393)

        self.declare_parameter('timer_period_gcs_background', 0.2)

        self.declare_parameter('cnt_threshold_fully_auto_mission', 50)
        
        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value
        self.drone_order = self.get_parameter('drone_order').get_parameter_value().integer_array_value
        self.kp_formation_load = self.get_parameter('kp_formation_load').get_parameter_value().double_value
        self.r_drones_max_safety = self.get_parameter('r_drones_max_safety').get_parameter_value().double_value
        self.auto_level = self.get_parameter('auto_level').get_parameter_value().bool_value
        self.phase_change_requests_through_background = self.get_parameter('phase_change_requests_through_background').get_parameter_value().bool_value

        #self.height_drone_rel_load = self.get_parameter('height_drone_rel_load').get_parameter_value().double_value
        self.takeoff_height_load_max = self.get_parameter('takeoff_height_load_max').get_parameter_value().double_value
        self.r_drones_rel_load = self.get_parameter('r_drones_rel_load').get_parameter_value().double_value
        self.d_drones_rel_min = self.get_parameter('d_drones_rel_min').get_parameter_value().double_value
        self.d_drones_rel_max = self.get_parameter('d_drones_rel_max').get_parameter_value().double_value
        self.drone_yaws_rel_load = self.get_parameter('drone_yaws_rel_load').get_parameter_value().double_array_value

        self.cable_length = self.get_parameter('cable_length').get_parameter_value().double_value
        self.load_connection_point_r = self.get_parameter('load_connection_point_r').get_parameter_value().double_value

        self.mission_circle_r = self.get_parameter('mission_circle_r').get_parameter_value().double_value
        self.mission_circle_v = self.get_parameter('mission_circle_v').get_parameter_value().double_value

        self.vel_load_vertical_slow = self.get_parameter('vel_load_vertical_slow').get_parameter_value().double_value
        self.vel_load_vertical_fast = self.get_parameter('vel_load_vertical_fast').get_parameter_value().double_value
        self.yawspeed_load = self.get_parameter('yawspeed_load').get_parameter_value().double_value

        self.timer_period_gcs_background = self.get_parameter('timer_period_gcs_background').get_parameter_value().double_value

        self.cnt_threshold_fully_auto_mission = self.get_parameter('cnt_threshold_fully_auto_mission').get_parameter_value().integer_value

        # Calculate height drone rel load
        self.height_drone_rel_load = utils.drone_height_rel_load(self.cable_length, self.r_drones_rel_load, self.load_connection_point_r)


        ## Print information
        self.get_logger().info('GCS BACKGROUND NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## VARIABLES
        self.load_desired_local_state = State(f'load{self.load_id}_init', CS_type.ENU)
        self.load_desired_local_state_prev = State(f'load{self.load_id}_init', CS_type.ENU)
        self.load_initial_local_state = State(f'load{self.load_id}_init', CS_type.ENU)

        self.drone_phases = np.array([-1] * self.num_drones)
        self.mission_theta = 0.0 # rad

        self.futures_set_drone_poses_rel_load = [None] * self.num_drones

        # Timers
        self.timer = self.create_timer(self.timer_period_gcs_background, self.clbk_cmdloop)
        self.cnt_phase_ticks = 0

        ## PUBLISHERS
        self.pub_load_attitude_desired = self.create_publisher(VehicleAttitudeSetpoint, f'load_{self.load_id}/in/desired_attitude', qos_profile)
        self.pub_load_position_desired = self.create_publisher(VehicleLocalPositionSetpoint, f'load_{self.load_id}/in/desired_local_position', qos_profile)

        ## SUBSCRIBERS
        # Drone current phases
        self.sub_drone_phases = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            callback = lambda msg, drone_ind=(i-self.first_drone_num): self.clbk_update_phase(msg, drone_ind)
        
            self.sub_drone_phases[i-self.first_drone_num] = self.create_subscription(
                Phase,
                f'/px4_{i}/out/current_phase',
                callback,
                qos_profile)                 

        ## TFs
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ## FLAGS 
        self.flag_load_init_pose_set = False

        ## SERVICES
        # Intermediate service to take in a change request from the user and forward to the drones (as GCS background is on the local ethernet network)
        if self.phase_change_requests_through_background:
            self.srv_phase_change_request = self.create_service(
                PhaseChange,
                f'/gcs_background_{self.load_id}/phase_change_request',
                self.clbk_phase_change_request)

        ## CLIENTS
        # Set drone poses relative to load
        self.cli_set_drone_poses_rel_load = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            # Desired poses rel load
            self.cli_set_drone_poses_rel_load[i-self.first_drone_num] = self.create_client(SetLocalPose,f'/px4_{i}/desired_pose_rel_load')

            while not self.cli_set_drone_poses_rel_load[i-self.first_drone_num].wait_for_service(timeout_sec=3.0): #1.0
                self.get_logger().info(f'Waiting for set pose rel load service: drone {i}')

        # Phase change
        self.cli_phase_change = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            self.cli_phase_change[i-self.first_drone_num] = self.create_client(PhaseChange,f'/px4_{i}/phase_change')
            while not self.cli_phase_change[i-self.first_drone_num].wait_for_service(timeout_sec=3.0): #1.0
                self.get_logger().info(f'Waiting for phase change service: drone {i}')  
        
        self.get_logger().info('Setup complete')


    ## CALLBACKS
    def clbk_phase_change_request(self, request, response):
        # Change phase of all drones
        # Note: don't wait for response so that the GCS can continue to send commands - non-blocking (otherwise get deadlock with gcs_user calling this service)
        utils.change_phase_all(self, self.cli_phase_change, request.phase_request.phase, wait_for_response=False)

        response.success = True
        return response

    def clbk_cmdloop(self):
        # Store previous load pose
        self.load_desired_local_state_prev = self.load_desired_local_state
        
        # Publish different setpoints depending on what phase the drones are in. 
        # If drones phases don't match, simply hold position

        # SAFETY MEASURES
        if np.any(self.drone_phases == Phase.PHASE_KILL):
            pass

        elif np.any(self.drone_phases == Phase.PHASE_HOLD):
            self.load_desired_local_state.pos = self.load_desired_local_state.pos
            self.load_desired_local_state.att_q = self.load_desired_local_state.att_q

        # Only allow transitioning if not waiting to set drone arrangement
        if np.any(self.futures_set_drone_poses_rel_load != None):
            # Skip this command loop if any drone's position relative to the load has not been successfully set (i.e. service has not yet returned)
            for i in range(self.num_drones):
                if self.futures_set_drone_poses_rel_load[i] != None:
                    if not self.futures_set_drone_poses_rel_load[i].done():
                        self.get_logger().info(f'Waiting for drone {i}\'s pose rel load to be set')
                        return
        
        # PHASES
        # Reset GCS and set drones to takeoff once GCS setup is complete
        if np.all(self.drone_phases == Phase.PHASE_SETUP_GCS):
            self.reset_pre_arm()

        # TAKEOFF
        elif np.all(self.drone_phases == Phase.PHASE_TAKEOFF_POST_TENSION):
            # Rise slowly - tension will engage
            self.load_desired_local_state.pos = np.array([0.0, 0.0, min(self.load_desired_local_state.pos[2] + self.vel_load_vertical_slow*self.timer_period_gcs_background, self.takeoff_height_load_max)])

            # Reset phase ticks
            self.cnt_phase_ticks = 0

        elif np.all(self.drone_phases == Phase.PHASE_TAKEOFF_END) and (self.auto_level == 2):
            # In fully auto, set drones to mission start phase once takeoff complete
            utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_MISSION_START)

            self.get_logger().info(f'In fully auto mode. Takeoff complete. Starting mission.')

            # Reset phase ticks
            self.cnt_phase_ticks = 0

        elif np.all(self.drone_phases == Phase.PHASE_MISSION_START):
            ## Perform mission
            self.load_desired_local_state = self.mission_circle(self.mission_circle_r, self.mission_circle_v/self.mission_circle_r, self.timer_period_gcs_background)

            self.cnt_phase_ticks += 1

            # If fully auto, transition to land phase when mission complete
            if (self.auto_level==2) and self.cnt_phase_ticks > self.cnt_threshold_fully_auto_mission:
                utils.change_phase_all(self, self.cli_phase_change, Phase.PHASE_LAND_START)
                        
                self.get_logger().info(f'In fully auto mode. Mission complete. Transitioning to land phase.')
                self.cnt_phase_ticks = 0
            
        
        elif np.all(self.drone_phases == Phase.PHASE_LAND_DESCENT):
            # Lower slowly - tension will disengage
            self.load_desired_local_state.pos = np.array([self.load_desired_local_state.pos[0], self.load_desired_local_state.pos[1], self.load_desired_local_state.pos[2] - self.vel_load_vertical_fast*self.timer_period_gcs_background]) #- 0.1

            # Reset phase ticks
            self.cnt_phase_ticks = 0

        elif np.all(self.drone_phases == Phase.PHASE_LAND_POST_LOAD_DOWN):
            # Spread drones out
            self.set_drone_arrangement(self.r_drones_rel_load + 0.3, np.array([self.height_drone_rel_load-0.1, self.height_drone_rel_load-0.1, self.height_drone_rel_load-0.1]), np.array(self.drone_yaws_rel_load)) #[0, np.pi*(2/self.num_drones), -np.pi*(2/self.num_drones)]

            # Reset phase ticks
            self.cnt_phase_ticks = 0

        elif np.all(self.drone_phases == Phase.PHASE_LAND_DRONES):
            # Slowly land drones
            self.load_desired_local_state.pos = np.array([self.load_desired_local_state.pos[0], self.load_desired_local_state.pos[1], self.load_desired_local_state.pos[2] - self.vel_load_vertical_slow*self.timer_period_gcs_background])

            # Reset phase ticks
            self.cnt_phase_ticks = 0

        self.send_desired_pose()

        
    def clbk_update_phase(self, msg, drone_ind):
        self.drone_phases[drone_ind] = msg.phase


    ## HELPERS
    def mission_circle(self, r, omega, dt):
        # Increment for first count (so desired vel is not 0)
        if self.cnt_phase_ticks == 0:
            self.mission_theta += omega*dt

        # Use load feedback for formation control
        # Formation control slows desired load position change down if too far away
        x_load_d = np.array([r*(np.cos(self.mission_theta)-1), r*np.sin(self.mission_theta), self.load_desired_local_state.pos[2]]) # Desired position (x_load_d = x_load_d_prev + x_dot_load_d_nom*dt)
        load_desired_local_state, yaw_change_desired_altered = self.formation_control_load(x_load_d, self.load_desired_local_state_prev.pos, omega*dt, self.mission_theta, dt) # For turning while moving in circle 
        self.mission_theta = self.mission_theta + yaw_change_desired_altered*dt
        
        #load_desired_local_state, yaw_change_desired_altered = self.formation_control_load(x_load_d, self.load_desired_local_state_prev.pos, 0.0, 0.0, dt)
        #self.mission_theta = self.mission_theta + omega*dt #TODO: CHANGE BACK TO yaw_change_desired_altered*dt

        return load_desired_local_state



    def formation_control_load(self, x_load_desired, x_load_desired_prev, yaw_change_desired, yaw_desired_prev, dt):
        # Get current load position
        t_load = utils.lookup_tf(f'load{self.load_id}_init', f'load{self.load_id}', self.tf_buffer, rclpy.time.Time(), self.get_logger())
        x_load = np.array([t_load.transform.translation.x, t_load.transform.translation.y, t_load.transform.translation.z])

        # Calculate desired load velocity
        x_dot_load_desired_nom = (x_load_desired - x_load_desired_prev)/dt

        # Alter desired load velocity to maintain formation using current load position feedback 
        x_dot_load_desired = x_dot_load_desired_nom - self.kp_formation_load*(x_load_desired - x_load)

        ## SAFETY SWITCH - If drones are too far out of desired positions, stop load movement
        # Retrieve current drone pose information 
        drone_positions, _, count_tf = utils.get_drone_poses(self.num_drones, self.first_drone_num, self.tf_buffer, self.get_logger())

        # If drones are too close or too far from each other, stop load movement
        #drone_dist_expected = utils.regular_polygon_side_length(self.num_drones, self.r_drones_rel_load)

        if count_tf < self.num_drones:
            self.get_logger().info(f'Not all drones found. Stopping load movement.')
            x_dot_load_desired = np.array([0.0, 0.0, 0.0])
        else:
            for i in range(self.num_drones):
                for j in range(i+1, self.num_drones):
                    if utils.dist_euler_3D(drone_positions[i], drone_positions[j]) < self.d_drones_rel_min or utils.dist_euler_3D(drone_positions[i], drone_positions[j]) > self.d_drones_rel_max:
                        self.get_logger().info(f'Drones too close/far from each other at {utils.dist_euler_3D(drone_positions[i], drone_positions[j])} m. Stopping load movement.')
                        x_dot_load_desired = np.array([0.0, 0.0, 0.0])
                        break

        self.get_logger().info(f'x_dot_load_desired: {x_dot_load_desired}')
        x_load_desired_altered = x_load_desired_prev + x_dot_load_desired*dt


        # Attitude
        load_init_yaw = ft.quaternion_get_yaw([self.load_initial_local_state.att_q.w, self.load_initial_local_state.att_q.x, self.load_initial_local_state.att_q.y, self.load_initial_local_state.att_q.z])

        if np.linalg.norm(x_load_desired - x_load_desired_prev) == 0.0:
            yaw_change_desired_altered = 0.0
        else:
            yaw_change_desired_altered = (np.linalg.norm(x_load_desired_altered - x_load_desired_prev)/np.linalg.norm(x_load_desired - x_load_desired_prev))*yaw_change_desired
        
        q_list = ft.quaternion_from_euler(0.0, 0.0, load_init_yaw + yaw_desired_prev + yaw_change_desired_altered) 
        att_load_desired_altered = np.quaternion(*q_list)

        # Return altered desired load state
        load_desired_state = State(f'load{self.load_id}_init', CS_type.ENU, pos=x_load_desired_altered, att=att_load_desired_altered)

        return load_desired_state, yaw_change_desired_altered     


    def reset_pre_arm(self):
        # Get current load orientation
        tf_load_rel_load_init = utils.lookup_tf(f'load{self.load_id}_init', f'load{self.load_id}', self.tf_buffer, rclpy.time.Time(), self.get_logger())

        # Skip setting if load pose not yet available
        if tf_load_rel_load_init == None:
            self.get_logger().info('Waiting for load pose to be set. Skipping reset until available')
            return

        # Set load initial local state
        self.load_initial_local_state.pos = np.array([tf_load_rel_load_init.transform.translation.x,
                                                        tf_load_rel_load_init.transform.translation.y,
                                                        tf_load_rel_load_init.transform.translation.z])
        
        self.load_initial_local_state.att_q = np.quaternion(tf_load_rel_load_init.transform.rotation.w,
                                                        tf_load_rel_load_init.transform.rotation.x,
                                                        tf_load_rel_load_init.transform.rotation.y,
                                                        tf_load_rel_load_init.transform.rotation.z)

        # Set load desired state
        # As attitude is in ENU, initial desired local attitude must be the same as starting attitude
        self.load_desired_local_state.pos = np.array([0.0, 0.0, 0.0])
        self.load_desired_local_state.att_q = self.load_initial_local_state.att_q.copy() 

        # Set drone arrangement around load
        self.set_drone_arrangement(self.r_drones_rel_load, np.array([self.height_drone_rel_load, self.height_drone_rel_load, self.height_drone_rel_load]), np.array(self.drone_yaws_rel_load)) #[0, np.pi*(2/self.num_drones), -np.pi*(2/self.num_drones)]
        #self.set_drone_arrangement(self.r_drones_rel_load, np.array([self.height_drone_rel_load, self.height_drone_rel_load*2.0, self.height_drone_rel_load*3.0]), np.array([0, np.pi*(2/self.num_drones), -np.pi*(2/self.num_drones)])) 

        # Set variables related to mission
        self.mission_theta = 0.0
        self.cnt_phase_ticks = 0


    def set_drone_arrangement(self, r, z, yaw):
        ref_points = utils.generate_points_cylinder(self.num_drones, r, z)

        # Re-arrange reference points and yaws based on drones' connection order to load 
        drone_order_indices = np.array(self.drone_order) - self.first_drone_num
        ref_points_ordered = ref_points[drone_order_indices]
        yaw_ordered = yaw[drone_order_indices]
        

        # Set drone arrangements
        for i, next_cli_set_drone_pose in enumerate(self.cli_set_drone_poses_rel_load):
            pos_req = SetLocalPose.Request()
            pos_req.transform_stamped.header.frame_id = f'load{self.load_id}'
            pos_req.transform_stamped.child_frame_id = f'drone{i+self.first_drone_num}'

            # Set position
            pos_req.transform_stamped.transform.translation.x = float(ref_points_ordered[i][0])
            pos_req.transform_stamped.transform.translation.y = float(ref_points_ordered[i][1])
            pos_req.transform_stamped.transform.translation.z = float(ref_points_ordered[i][2])

            # Set yaw
            q_des = np.quaternion(*ft.quaternion_from_euler(0.0, 0.0, yaw_ordered[i]))

            pos_req.transform_stamped.transform.rotation.x = float(q_des.x)
            pos_req.transform_stamped.transform.rotation.y = float(q_des.y)
            pos_req.transform_stamped.transform.rotation.z = float(q_des.z)
            pos_req.transform_stamped.transform.rotation.w = float(q_des.w)

            self.futures_set_drone_poses_rel_load[i] = next_cli_set_drone_pose.call_async(pos_req)



    def send_desired_pose(self):
        # Send position setpoint
        setpoint_msg_pos = VehicleLocalPositionSetpoint()
        setpoint_msg_pos.x = self.load_desired_local_state.pos[0] 
        setpoint_msg_pos.y = self.load_desired_local_state.pos[1] 
        setpoint_msg_pos.z = self.load_desired_local_state.pos[2]
        self.pub_load_position_desired.publish(setpoint_msg_pos)

        # Send orientation setpoint
        q_d = self.load_desired_local_state.att_q
        setpoint_msg_att = VehicleAttitudeSetpoint()
        setpoint_msg_att.q_d = [float(q_d.w), float(q_d.x), float(q_d.y), float(q_d.z)]
        self.pub_load_attitude_desired.publish(setpoint_msg_att)

        # Print desired pose
        self.get_logger().info(f'Desired load pose: {self.load_desired_local_state.pos}, {self.load_desired_local_state.att_q}')


def main(args=None):
    # Create node
    rclpy.init(args=args)
    gcs_background = GCSBackground()
    
    # Maintain node
    rclpy.spin(gcs_background)

    # Destroy node
    gcs_background.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
