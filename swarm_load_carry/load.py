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

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint, VehicleLocalPosition, VehicleLocalPositionSetpoint

from swarm_load_carry_interfaces.srv import GetGlobalInitPose, SetLocalInitPose

DEFAULT_DRONE_NUM=1
DEFAULT_FIRST_DRONE_NUM=1
FRAME_BASE='world'
FRAME_LOCAL_REF='load1_init'
#FRAME_LOAD='load1'

# Could make subclasses for different load types (e.g. camera etc.)
class Load(Node):
    def __init__(self):
        super().__init__('load')

        self.load_setpoint_local_position = np.array([0.0, 0.0, 0.0])           # ENU relative to starting pos
        self.load_setpoint_local_attitude = qt.array([0.0, 0.0, 0.0, 1.0])      # ROS convention quaternion (ENU)

        self.load_local_position = np.array([0.0, 0.0, 0.0])                    # ENU relative to starting pos
        self.load_local_attitude = qt.array([0.0, 0.0, 0.0, 1.0])               # ROS convention quaternion (ENU) 
        #self.load_local_velocity = np.array([0.0, 0.0, 0.0])

        self.load_initial_global_position = np.array([0.0, 0.0, 0.0])           # In lat, lon, alt (lla)
        self.load_initial_global_attitude = qt.array([0.0, 0.0, 0.0, 1.0])      # ROS convention quaternion (ENU)

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

        ## TIMERS

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
            self.cli_set_drone_init_local_poses[i-self.first_drone_num] = self.create_client(SetLocalInitPose,f'/px4_{i}/local_initial_pose')

            while not self.cli_set_drone_init_local_poses[i-self.first_drone_num].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for local initial pose service drone {i}')


        ## TFS
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        #self.tf_static_broadcasters_init_poses = [StaticTransformBroadcaster(self)] * self.num_drones

        self.send_initial_pose_tf()


    ## CALLBACKS
    def clbk_desired_load_attitude(self, msg):
        # Update stored setpoint
        self.load_setpoint_local_attitude = qt.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]])


        # TEMP: Assume load attitude moves directly to desired attitude (TODO: add dynamics or sensing/estimation. Publish actual attitude in timer clbk instead)
        load_attitude = VehicleAttitude()
        load_attitude.q = msg.q_d

        self.broadcast_load_local_attitude(load_attitude) 

    
    def clbk_desired_load_local_position(self, msg):
        # Update stored setpoint
        self.load_setpoint_local_position = np.array([msg.x, msg.y, msg.z])
        

        # TEMP: Assume load position moves directly to desired position (TODO: add dynamics or sensing/estimation. Publish actual pos in timer clbk instead)
        load_position = VehicleLocalPosition()
        load_position.x = msg.x
        load_position.y = msg.y
        load_position.z = msg.z

        self.broadcast_load_local_position(load_position)   


    ## HELPER FUNCTIONS
    def broadcast_load_local_attitude(self, msg):
        self.pub_load_attitude.publish(msg)

        # Update stored attitude
        self.load_local_attitude = qt.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

        # Update tf
        utils.broadcast_tf(self.get_clock().now().to_msg(), FRAME_LOCAL_REF, self.get_name(), self.load_local_position, self.load_local_attitude, self.tf_broadcaster)

    def broadcast_load_local_position(self, msg):
        self.pub_load_position.publish(msg)
        #self.get_logger().info(f'Load at pos: {msg.x, msg.y, msg.z}')

        # Update stored position
        self.load_local_position = np.array([msg.x, msg.y, msg.z])

        # Update tf
        utils.broadcast_tf(self.get_clock().now().to_msg(), FRAME_LOCAL_REF, self.get_name(), self.load_local_position, self.load_local_attitude, self.tf_broadcaster)

    # def broadcast_tf(self):
    #     # Broadcast frame transform
    #     t = TransformStamped()

    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = FRAME_LOCAL_REF
    #     t.child_frame_id = self.get_name()

    #     t.transform.translation.x = self.load_local_position[0]
    #     t.transform.translation.y = self.load_local_position[1]
    #     t.transform.translation.z = self.load_local_position[2]

    #     t.transform.rotation.x = float(self.load_local_attitude.x)
    #     t.transform.rotation.y = float(self.load_local_attitude.y)
    #     t.transform.rotation.z = float(self.load_local_attitude.z)
    #     t.transform.rotation.w = float(self.load_local_attitude.w)

    #     self.tf_broadcaster.sendTransform(t)

    


    def send_initial_pose_tf(self):
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
            self.load_initial_global_position[0] += drone_global_init_poses[i].global_pos.lat
            self.load_initial_global_position[1] += drone_global_init_poses[i].global_pos.lon
            self.load_initial_global_position[2] += drone_global_init_poses[i].global_pos.alt

            # Orientation (currently simply take orientaiton of last drone as orientation of load) #TODO: Get better initial estimate of load orientation
            self.load_initial_global_attitude.x = drone_global_init_poses[i].global_att.q[0]
            self.load_initial_global_attitude.y = drone_global_init_poses[i].global_att.q[1]
            self.load_initial_global_attitude.z = drone_global_init_poses[i].global_att.q[2]
            self.load_initial_global_attitude.w = drone_global_init_poses[i].global_att.q[3]


        ## Calculate load initial position and orientation
        # Estimate load initial position (centre of circle) and inital orientation (average orentation of drones)
        self.load_initial_global_position = np.divide(self.load_initial_global_position, self.num_drones)
        #self.load_initial_global_attitude = np.divide(self.load_initial_global_attitude, self.num_drones)

        
        ## Send static TF
        utils.broadcast_tf(self.get_clock().now().to_msg(), FRAME_BASE, FRAME_LOCAL_REF, self.load_initial_global_position, self.load_initial_global_attitude, self.tf_static_broadcaster)


        ## Set drone local poses relative to load (allowing the drones to publish local tfs)       
        # Send request
        # init_local_pose_future = [None] * self.num_drones

        # for i, next_drone_init_pose in enumerate(drone_global_init_poses):
        #     req = SetLocalInitPose.Request()

        #     req.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        #     req.transform_stamped.header.frame_id = 'load1_init' 
        #     #req.child_frame_id = 

        #     req.transform_stamped.transform.translation.x, req.transform_stamped.transform.translation.y, req.transform_stamped.transform.translation.z = pm.geodetic2enu(next_drone_init_pose.global_pos.lat, next_drone_init_pose.global_pos.lon, next_drone_init_pose.global_pos.alt, self.load_initial_global_position[0], self.load_initial_global_position[1], self.load_initial_global_position[2]) 

        #     q_drone_global_init = qt.array([next_drone_init_pose.att.q[0], next_drone_init_pose.att.q[1], next_drone_init_pose.att.q[2], next_drone_init_pose.att.q[3]])
        #     q_load_global_init = qt.array([self.load_initial_global_attitude.x, self.load_initial_global_attitude.y, self.load_initial_global_attitude.z, self.load_initial_global_attitude.w])

        #     q_drone_init_rel_load_init = q_drone_global_init*q_load_global_init.inverse
        #     q_drone_init_rel_load_init = q_drone_init_rel_load_init*



        #     ### HEREEEEEE - INITIAL QUATERNION RELATIVE ORIENTATION (do quart math)

        #     # req.transform_stamped.transform.rotation.x = float(next_drone_init_pose.att.q[0])
        #     # req.transform_stamped.transform.rotation.y = float(next_drone_init_pose.att.q[1])
        #     # req.transform_stamped.transform.rotation.z = float(next_drone_init_pose.att.q[2])
        #     # req.transform_stamped.transform.rotation.w = float(next_drone_init_pose.att.q[3])

        #     init_local_pose_future[i] = self.cli_set_drone_init_local_poses[i].call_async(req)


        # # Wait for response 
        # for i, next_future in enumerate(init_local_pose_future):
        #     rclpy.spin_until_future_complete(self, next_future)

        #     self.get_logger().info(f'Local TF set for drone {i+1}')


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