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

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
DEFAULT_LOAD_ID=1
DEFAULT_FULLY_AUTO=False

HEIGHT_DRONE_REL_LOAD=2 #m
MAX_LOAD_TAKEOFF_HEIGHT=4 #m

MAIN_TIMER_PERIOD=0.2 # s

FULLY_AUTO_MISSION_CNT_THRESHOLD=10/MAIN_TIMER_PERIOD

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

        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)
        self.declare_parameter('load_id', DEFAULT_LOAD_ID)
        self.declare_parameter('drone_order', [1, 2, 3])
        self.declare_parameter('fully_auto', DEFAULT_FULLY_AUTO)
        
        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value
        self.fully_auto = self.get_parameter('fully_auto').get_parameter_value().bool_value
        self.load_id = self.get_parameter('load_id').get_parameter_value().integer_value
        self.drone_order = self.get_parameter('drone_order').get_parameter_value().integer_array_value

        ## Print information
        self.get_logger().info('GCS BACKGROUND NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## VARIABLES
        self.load_desired_local_state = State(f'load{self.load_id}_init', CS_type.ENU)
        self.load_initial_local_state = State(f'load{self.load_id}_init', CS_type.ENU)

        self.drone_phases = np.array([-1] * self.num_drones)
        self.mission_theta = 0.0 # rad

        self.futures_set_drone_poses_rel_load = [None] * self.num_drones

        # Timers
        self.timer = self.create_timer(MAIN_TIMER_PERIOD, self.clbk_cmdloop)
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
        ## CLIENTS
        # Set drone poses relative to load
        self.cli_set_drone_poses_rel_load = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            # Desired poses rel load
            self.cli_set_drone_poses_rel_load[i-self.first_drone_num] = self.create_client(SetLocalPose,f'/px4_{i}/desired_pose_rel_load')

            while not self.cli_set_drone_poses_rel_load[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for set pose rel load service: drone {i}')

        # Phase change
        self.cli_phase_change = [None] * self.num_drones
        #self.phase_future = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            self.cli_phase_change[i-self.first_drone_num] = self.create_client(PhaseChange,f'/px4_{i}/phase_change')
            while not self.cli_phase_change[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for offboard ROS start service {i}')

        self.get_logger().info('Setup complete')


    ## CALLBACKS
    def clbk_cmdloop(self):
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
            self.load_desired_local_state.pos = np.array([0.0, 0.0, min(self.load_desired_local_state.pos[2] + 0.1*MAIN_TIMER_PERIOD, MAX_LOAD_TAKEOFF_HEIGHT)])

        elif np.all(self.drone_phases == Phase.PHASE_TAKEOFF_END) and self.fully_auto:
            # In fully auto, set drones to mission start phase once takeoff complete
            utils.change_phase_all_drones(self, self.num_drones, self.cli_phase_change, Phase.PHASE_MISSION_START)

            self.get_logger().info(f'In fully auto mode. Takeoff complete. Starting mission.')

        elif np.all(self.drone_phases == Phase.PHASE_MISSION_START):
            # Perform mission - currently move load in circle
            v_lin = 0.5 # m/s

            r = 2 #10 # m
            omega = v_lin/r # rad/s
            dt = MAIN_TIMER_PERIOD # s

            # Move load in circle
            self.load_desired_local_state.pos = np.array([r*(np.cos(self.mission_theta)-1), r*np.sin(self.mission_theta), self.load_desired_local_state.pos[2]])
            
            load_init_yaw = ft.quaternion_get_yaw([self.load_initial_local_state.att_q.w, self.load_initial_local_state.att_q.x, self.load_initial_local_state.att_q.y, self.load_initial_local_state.att_q.z])
            q_list = ft.quaternion_from_euler(0.0, 0.0, load_init_yaw + self.mission_theta)
            self.load_desired_local_state.att_q = np.quaternion(*q_list)

            # Update theta
            self.mission_theta = self.mission_theta + omega*dt

            self.cnt_phase_ticks += 1

            # If fully auto, transition to land phase when mission complete
            if self.fully_auto and self.cnt_phase_ticks > FULLY_AUTO_MISSION_CNT_THRESHOLD:
                for i in range(self.num_drones):
                    utils.phase_change(self.cli_phase_change[i], Phase.PHASE_LAND_START)
                        
                self.get_logger().info(f'In fully auto mode. Mission complete. Transitioning to land phase.')
                self.cnt_phase_ticks = 0
            
        
        elif np.all(self.drone_phases == Phase.PHASE_LAND_DESCENT):
            # Lower slowly - tension will disengage
            self.load_desired_local_state.pos = np.array([self.load_desired_local_state.pos[0], self.load_desired_local_state.pos[1], self.load_desired_local_state.pos[2] - 0.3*MAIN_TIMER_PERIOD]) #- 0.1

        elif np.all(self.drone_phases == Phase.PHASE_LAND_POST_LOAD_DOWN):
            self.set_drone_arrangement(1.3, np.array([1, 1, 1]), np.array([0, -np.pi*(1-2/self.num_drones), np.pi*(1-2/self.num_drones)]))

        self.send_desired_pose()

        
    def clbk_update_phase(self, msg, drone_ind):
        self.drone_phases[drone_ind] = msg.phase


    ## HELPERS
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
        self.set_drone_arrangement(1, np.array([HEIGHT_DRONE_REL_LOAD, HEIGHT_DRONE_REL_LOAD, HEIGHT_DRONE_REL_LOAD]), np.array([0, -np.pi*(1-2/self.num_drones), np.pi*(1-2/self.num_drones)]))

        # Set variables related to mission
        self.mission_theta = 0.0


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
