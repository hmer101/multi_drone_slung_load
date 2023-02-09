# Contains the Load class and related methods
#
# Author: Harvey Merton
# Date: 01/26/2023

import numpy as np
import pymap3d as pm
import quaternionic as qt
import utils
import rclpy
import rclpy.qos as qos
from rclpy.qos import QoSProfile
from rclpy.node import Node

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint, VehicleLocalPosition, VehicleLocalPositionSetpoint

from swarm_load_carry_interfaces.srv import GetGlobalInitPose, SetLocalPose
from swarm_load_carry.state import State, CS_type

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1

# Could make subclasses for different load types (e.g. camera etc.)
class Load(Node):
    def __init__(self):
        super().__init__('load')

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
        self.load_desired_state = State(f'{self.load_id}_init', CS_type.ENU)

        self.load_local_state = State(f'{self.load_id}_init', CS_type.ENU)

        self.load_initial_global_state = State('world', CS_type.LLA)

        self.flag_tfs_set = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        ## TIMERS
        self.timer = self.create_timer(0.02, self.clbk_publoop)

        ## PUBLISHERS
        self.pub_load_attitude = self.create_publisher(VehicleAttitude, f'load_{self.load_id}/out/attitude', qos_profile)
        self.pub_load_position = self.create_publisher(VehicleLocalPosition, f'load_{self.load_id}/out/local_position', qos_profile)
        
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
        self.cli_get_drone_init_global_poses = [None] * self.num_drones
        self.cli_set_drone_init_local_poses = [None] * self.num_drones

        for i in range(self.first_drone_num, self.num_drones+self.first_drone_num):
            # Global initial poses
            self.cli_get_drone_init_global_poses[i-self.first_drone_num] = self.create_client(GetGlobalInitPose,f'/px4_{i}/global_initial_pose')

            while not self.cli_get_drone_init_global_poses[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for global initial pose service drone {i}')

            # Local initial poses
            self.cli_set_drone_init_local_poses[i-self.first_drone_num] = self.create_client(SetLocalPose,f'/px4_{i}/local_initial_pose')

            while not self.cli_set_drone_init_local_poses[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for local initial pose service drone {i}')


        ## TFS
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.send_initial_pose_tfs()


    ## CALLBACKS
    def clbk_desired_load_attitude(self, msg):
        # Update stored setpoint
        self.load_desired_state.att_q = qt.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])


        # TEMP: Assume load attitude moves directly to desired attitude (TODO: add dynamics or sensing/estimation. Publish actual attitude in timer clbk instead)
        # load_attitude = VehicleAttitude()
        # load_attitude.q = msg.q_d

        # self.broadcast_load_local_attitude(load_attitude) 

    
    def clbk_desired_load_local_position(self, msg):
        # Update stored setpoint
        self.load_desired_state.pos = np.array([msg.x, msg.y, msg.z])
        

        # TEMP: Assume load position moves directly to desired position (TODO: add dynamics or sensing/estimation. Publish actual pos in timer clbk instead)
        # load_position = VehicleLocalPosition()
        # load_position.x = msg.x
        # load_position.y = msg.y
        # load_position.z = msg.z

        # self.broadcast_load_local_position(load_position)   


    # Loop on timer to publish actual load pose
    def clbk_publoop(self):
        # Retrieve drone information 
        drone_positions = np.zeros((self.num_drones, 3))
        drone_orientations = qt.array(np.zeros((self.num_drones, 4))) 

        if self.flag_tfs_set:
            # Store position and orientation of each drone relative to load initial position
            for i in range(self.num_drones):
                from_frame_rel = f'{self.get_name()}_init'
                to_frame_rel = f'drone{i+self.first_drone_num}'
                    
                t = utils.lookup_tf(from_frame_rel, to_frame_rel, self.tf_buffer, rclpy.time.Time(), self.get_logger())

                if t != None:
                    drone_positions[i, :] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                    drone_orientations[i, :] = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]

            # Estimate load position as average of drone positions 
            self.load_local_state.pos  = np.average(drone_positions, axis=0)

            # Estimate load orientation #TODO: Better orientation estimation method
            self.load_local_attitude = drone_orientations[1, :]

            # Publish estimate load 
            self.broadcast_load_local_attitude()
            self.broadcast_load_local_position()



    ## HELPER FUNCTIONS
    def broadcast_load_local_attitude(self):
        # Generate message
        msg = VehicleAttitude()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.q = [float(self.load_local_attitude.x), float(self.load_local_attitude.y), float(self.load_local_attitude.z), float(self.load_local_attitude.w)]

        # Publish
        self.pub_load_attitude.publish(msg)
        #self.get_logger().info(f'Load at att: {msg.q[0], msg.q[1], msg.q[2], msg.q[3]}')

        # Update tf
        utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', self.get_name(), self.load_local_state.pos, self.load_local_state.att_q, self.tf_broadcaster)

    def broadcast_load_local_position(self):
        # Generate message
        msg = VehicleLocalPosition()
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        msg.x = self.load_local_state.pos[0]
        msg.y = self.load_local_state.pos[1]
        msg.z = self.load_local_state.pos[2]

        # Publish
        self.pub_load_position.publish(msg)
        #self.get_logger().info(f'Load at pos: {msg.x, msg.y, msg.z}')

        # Update tf
        utils.broadcast_tf(self.get_clock().now().to_msg(), f'{self.get_name()}_init', self.get_name(), self.load_local_state.pos, self.load_local_state.att_q, self.tf_broadcaster)

    def send_initial_pose_tfs(self):
        ## Get drone initial positions
        # Prepare request
        init_global_pose_req = GetGlobalInitPose.Request()

        # Send request
        init_global_pose_future = [None] * self.num_drones

        for i in range(self.num_drones):
            init_global_pose_future[i] = self.cli_get_drone_init_global_poses[i].call_async(init_global_pose_req)

        # Wait for response and accumulate result
        drone_global_init_poses = [None] * self.num_drones

        for i, next_future in enumerate(init_global_pose_future):
            rclpy.spin_until_future_complete(self, next_future)
            drone_global_init_poses[i] = next_future.result()

            # Position
            self.load_initial_global_state.pos[0] += drone_global_init_poses[i].global_pos.lat
            self.load_initial_global_state.pos[1] += drone_global_init_poses[i].global_pos.lon
            self.load_initial_global_state.pos[2] += drone_global_init_poses[i].global_pos.alt

            # Orientation (currently simply take orientaiton of last drone as orientation of load) #TODO: Get better initial estimate of load orientation
            self.load_initial_global_state.att_q.x = drone_global_init_poses[i].global_att.q[0]
            self.load_initial_global_state.att_q.y = drone_global_init_poses[i].global_att.q[1]
            self.load_initial_global_state.att_q.z = drone_global_init_poses[i].global_att.q[2]
            self.load_initial_global_state.att_q.w = drone_global_init_poses[i].global_att.q[3]


        ## Calculate load initial position and orientation
        # Estimate load initial position (centre of circle) and inital orientation (average orentation of drones)
        self.load_initial_global_state.pos = np.divide(self.load_initial_global_state.pos, self.num_drones)
        #self.load_initial_global_attitude = np.divide(self.load_initial_global_attitude, self.num_drones)

        ## Set drone local poses relative to load (allowing the drones to publish local tfs)       
        # Send request
        future_drone_init_local_pose = [None] * self.num_drones

        for i, next_drone_init_pose in enumerate(drone_global_init_poses):
            req = SetLocalPose.Request()

            req.transform_stamped.header.stamp = self.get_clock().now().to_msg()
            req.transform_stamped.header.frame_id = f'{self.get_name()}_init' 
            #req.child_frame_id

            # Drone translation relative to load
            req.transform_stamped.transform.translation.x, req.transform_stamped.transform.translation.y, req.transform_stamped.transform.translation.z = pm.geodetic2enu(next_drone_init_pose.global_pos.lat, next_drone_init_pose.global_pos.lon, next_drone_init_pose.global_pos.alt, self.load_initial_global_state.pos[0], self.load_initial_global_state.pos[1], self.load_initial_global_state.pos[2]) 

            # Drone orientation relative to load
            q_drone_global_init = qt.array([next_drone_init_pose.global_att.q[0], next_drone_init_pose.global_att.q[1], next_drone_init_pose.global_att.q[2], next_drone_init_pose.global_att.q[3]])
            q_load_global_init = qt.array([self.load_initial_global_state.att_q.x, self.load_initial_global_state.att_q.y, self.load_initial_global_state.att_q.z, self.load_initial_global_state.att_q.w])

            q_drone_init_rel_load_init = q_drone_global_init*q_load_global_init.inverse

            req.transform_stamped.transform.rotation.x = float(q_drone_init_rel_load_init.x)
            req.transform_stamped.transform.rotation.y = float(q_drone_init_rel_load_init.y)
            req.transform_stamped.transform.rotation.z = float(q_drone_init_rel_load_init.z)
            req.transform_stamped.transform.rotation.w = float(q_drone_init_rel_load_init.w)

            future_drone_init_local_pose[i] = self.cli_set_drone_init_local_poses[i].call_async(req)


        # Wait for response 
        for i, next_future in enumerate(future_drone_init_local_pose):
            rclpy.spin_until_future_complete(self, next_future)

            self.get_logger().info(f'Local TF set for drone {i+1}')

        # Set TF set flag
        
        self.flag_tfs_set = True


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