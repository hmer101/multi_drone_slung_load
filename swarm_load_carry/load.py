# Contains the Load class and related methods
#
# Author: Harvey Merton
# Date: 01/26/2023

import numpy as np
import quaternionic as qt
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

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1

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
      

        ## TIMERS
        self.timer = self.create_timer(0.02, self.clbk_publoop)

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

        ## SERVICES
        ## CLIENTS

        ## TFS
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster_init_pose = StaticTransformBroadcaster(self)

        self.flag_tf_init_set = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Setup complete')


    ## CALLBACKS
    def clbk_desired_load_attitude(self, msg):
        # Update stored setpoint
        self.load_desired_state.att_q = qt.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])

        # TEMP: Assume load attitude moves directly to desired attitude (TODO: add dynamics or sensing/estimation. Publish actual attitude in timer clbk instead)


    def clbk_desired_load_local_position(self, msg):
        # Update stored setpoint
        self.load_desired_state.pos = np.array([msg.x, msg.y, msg.z])
        
        # TEMP: Assume load position moves directly to desired position (TODO: add dynamics or sensing/estimation. Publish actual pos in timer clbk instead)


    # Loop on timer to publish actual load pose
    def clbk_publoop(self):
        # Retrieve drone information 
        drone_positions = np.zeros((self.num_drones, 3))
        drone_orientations = qt.array(np.zeros((self.num_drones, 4))) 

        # Store position and orientation of each drone relative to world
        count_tf = 0

        for i in range(self.num_drones):
            from_frame_rel = 'world'
            to_frame_rel = f'drone{i+self.first_drone_num}'
            
            if self.tf_buffer.can_transform(from_frame_rel, to_frame_rel, rclpy.time.Time(), rclpy.duration.Duration(seconds=1)):
                t = utils.lookup_tf(from_frame_rel, to_frame_rel, self.tf_buffer, rclpy.time.Time(), self.get_logger())

                if t != None:
                    drone_positions[i, :] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                    drone_orientations[i, :] = [t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z]

                    count_tf += 1
            else:
                self.get_logger().warn(f'Cannot transform from: {from_frame_rel} to {to_frame_rel}')

        # Only broadcast load position if all drone positions can be found to estimate it
        if count_tf == self.num_drones:
            # Estimate load position as average of drone positions 
            self.load_state_rel_world.pos  = np.average(drone_positions, axis=0)

            # Estimate load orientation #TODO: Better orientation estimation method
            self.load_state_rel_world.att_q = qt.array(drone_orientations[0, :])

            # Set and publish initial load position TF
            if self.flag_tf_init_set == False:
                self.set_tf_init_pose()

            # Publish estimate load relative to load initial position
            load_rel_load_init = State(f'{self.get_name()}_init', CS_type.ENU)
            load_rel_load_init.pos = self.load_state_rel_world.pos - self.load_initial_state_rel_world.pos
            load_rel_load_init.att_q = self.load_state_rel_world.att_q*(1/self.load_initial_state_rel_world.att_q)

            utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', self.get_name(), load_rel_load_init.pos, load_rel_load_init.att_q, self.tf_broadcaster)          



    ## HELPER FUNCTIONS
    def set_tf_init_pose(self):
        # Set initial pose
        self.load_initial_state_rel_world.pos = np.copy(self.load_state_rel_world.pos)
        self.load_initial_state_rel_world.att_q = self.load_state_rel_world.att_q.copy()
        
        # Publish static transform for init pose (relative to world)
        utils.broadcast_tf(self.get_clock().now().to_msg(), 'world', f'{self.get_name()}_init', self.load_initial_state_rel_world.pos, self.load_initial_state_rel_world.att_q, self.tf_static_broadcaster_init_pose)

        self.flag_tf_init_set = True 
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