# Contains the Load class and related methods
#
# Author: Harvey Merton
# Date: 01/26/2023

import numpy as np
#import quaternionic as quaternion
import quaternion
import utils
import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint

from swarm_load_carry.state import State, CS_type
from swarm_load_carry_interfaces.msg import Phase

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1

PUB_LOOP_TIMER_PERIOD=0.1

HEIGHT_DRONE_REL_LOAD=2 #m

# Could make subclasses for different load types (e.g. camera etc.)
class Load(Node):
    def __init__(self):
        super().__init__('load9')

        ## Print information
        self.load_id = int(str(self.get_name())[-1])

        self.get_logger().info('LOAD NODE')
        self.get_logger().info(f'Namespace: {self.get_namespace()}')
        self.get_logger().info(f'Name: {self.get_name()}')

        ## PARAMETERS
        self.declare_parameter('num_drones', DEFAULT_DRONE_NUM)
        self.declare_parameter('first_drone_num', DEFAULT_FIRST_DRONE_NUM)

        self.num_drones = self.get_parameter('num_drones').get_parameter_value().integer_value
        self.first_drone_num = self.get_parameter('first_drone_num').get_parameter_value().integer_value

        qos_profile = QoSProfile(
            reliability=qos.ReliabilityPolicy.BEST_EFFORT,
            durability=qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        ## VARIABLES
        self.load_desired_state = State(f'{self.get_name()}_init', CS_type.ENU)

        self.load_initial_state_rel_world = State('world', CS_type.ENU)
        self.load_state_rel_world = State('world', CS_type.ENU)

        self.drone_phases = np.array([-1] * self.num_drones)

        ## TIMERS
        self.timer = self.create_timer(PUB_LOOP_TIMER_PERIOD, self.clbk_publoop)

        ## PUBLISHERS
        
        ## SUBSCRIBERS
        self.sub_load_attitude_desired = self.create_subscription(
            VehicleAttitudeSetpoint,
            f'load_{self.load_id}/in/desired_attitude',
            self.clbk_desired_load_attitude,
            qos_profile)

        self.sub_load_position_desired = self.create_subscription(
            VehicleLocalPositionSetpoint,
            f'load_{self.load_id}/in/desired_local_position',
            self.clbk_desired_load_local_position,
            qos_profile)

        # Drone current phases
        self.sub_drone_phases = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            callback = lambda msg, drone_ind=(i-self.first_drone_num): self.clbk_update_drone_phase(msg, drone_ind)
        
            self.sub_drone_phases[i-self.first_drone_num] = self.create_subscription(
                Phase,
                f'/px4_{i}/out/current_phase',
                callback,
                qos_profile)

        ## SERVICES
        ## CLIENTS

        ## TFS
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster_init_pose = StaticTransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Initialization complete')


    ## CALLBACKS
    def clbk_desired_load_attitude(self, msg):
        # Update stored setpoint
        self.load_desired_state.att_q = np.quaternion(*[msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])

        # TEMP: Assume load attitude moves directly to desired attitude (TODO: add dynamics or sensing/estimation. Publish actual attitude in timer clbk instead)


    def clbk_desired_load_local_position(self, msg):
        # Update stored setpoint
        self.load_desired_state.pos = np.array([msg.x, msg.y, msg.z])
        
        # TEMP: Assume load position moves directly to desired position (TODO: add dynamics or sensing/estimation. Publish actual pos in timer clbk instead)

    def clbk_update_drone_phase(self, msg, drone_ind):
        self.drone_phases[drone_ind] = msg.phase

    # Loop on timer to publish actual load pose
    def clbk_publoop(self):       
        # Retrieve drone information 
        drone_positions = np.zeros((self.num_drones, 3))
        drone_orientations = np.array([np.quaternion(*q) for q in np.zeros((self.num_drones, 4))])

        # Store position and orientation of each drone relative to world
        count_tf = 0

        for i in range(self.num_drones):
            target_frame = 'world'
            source_frame = f'drone{i+self.first_drone_num}'
            
            t = utils.lookup_tf(target_frame, source_frame, self.tf_buffer, rclpy.time.Time(), self.get_logger())

            if t != None:
                drone_positions[i, :] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                drone_orientations[i] = np.quaternion(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z)

                count_tf += 1


        # Only broadcast load position if all drone positions can be found to estimate it
        if count_tf == self.num_drones:
            # Estimate load position as average of drone positions 
            self.load_state_rel_world.pos  = np.average(drone_positions, axis=0)

            # TODO: Better height estimate
            if np.all(self.drone_phases >= Phase.PHASE_TAKEOFF_POST_TENSION):
                self.load_state_rel_world.pos[2] -= HEIGHT_DRONE_REL_LOAD 
                self.load_state_rel_world.pos[2] = max(self.load_state_rel_world.pos[2], 0.0) # Ensure load doesn't go below ground
            else: 
                self.load_state_rel_world.pos[2] = 0.0

            self.get_logger().info(f'Load position: {self.load_state_rel_world.pos}')

            # Estimate load orientation #TODO: Better orientation estimation method
            self.load_state_rel_world.att_q = drone_orientations[0] #np.quaternion(*drone_orientations[0, :])

            # If all drones are in load setup phase, reset load's init pose
            if np.all(self.drone_phases == Phase.PHASE_SETUP_LOAD):
                self.set_tf_init_pose()

            # Publish estimate load relative to load initial position
            load_rel_load_init = utils.transform_frames(self.load_state_rel_world, f'{self.get_name()}_init', self.tf_buffer, self.get_logger())

            if load_rel_load_init != None:
                utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', self.get_name(), load_rel_load_init.pos, load_rel_load_init.att_q, self.tf_broadcaster)          


    ## HELPER FUNCTIONS
    def set_tf_init_pose(self):
        # Set initial pose
        self.load_initial_state_rel_world.pos = np.copy(self.load_state_rel_world.pos)
        self.load_initial_state_rel_world.att_q = self.load_state_rel_world.att_q.copy()
        
        # Publish static transform for init pose (relative to world)
        # As CS is in ENU, always aligned
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'world', f'{self.get_name()}_init', self.load_initial_state_rel_world.pos, np.quaternion(*[1.0, 0.0, 0.0, 0.0]), self.tf_static_broadcaster_init_pose)

        self.get_logger().info('Initial pose TF set')


def main():
    # Create node
    rclpy.init()
    load = Load()

    # Maintain node
    rclpy.spin(load)

    # Destroy node
    load.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()