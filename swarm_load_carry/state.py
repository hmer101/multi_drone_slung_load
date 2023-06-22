# Contains the state class, co-ordinate system type enum and related methods
#
# Author: Harvey Merton
# Date: 02/28/2023

import numpy as np
import quaternionic as qt

from enum import Enum

# Co-ordinate system enum
# https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-px4-frame-conventions
class CS_type(Enum):
    LLA=0,         # Planetary co-ordinate system: Latitude, Longitude, Altitude. Frame can only be 'world'
    ENU=1,                  # (world) Local tangent plane body CS: East, North, Up (ROS2 default) 
    NED=2,                  # (world) Local tangent plane body CS: North, East, Down (PX4 default)
    # FLU=3,                  # (body) Body frame: Forward, Left, Up (ROS2 default) 
    # FRD=4                   # (body) Body frame: Forward, Right, Down (PX4 default)

# Class to store robot state. 
# Like geometry_msgs/PoseStamped message but with different background datastructures allowing different co-ordinate system types and other calculations
class State():
    def __init__(self, frame, cs_type, pos=np.array([0.0, 0.0, 0.0]), att=qt.array([1.0, 0.0, 0.0, 0.0]), vel=np.array([0.0, 0.0, 0.0])):
        self.frame = frame
        self.cs_type = cs_type

        self.pos = pos      # Position relative to frame in given co-ordinate system type
        self.att_q = att    # Attitude relative to frame as a quaternion [w, x, y, z] (note quaternionic and PX4 default have w first. ROS quaternion msg doesn't have a vector - must specify .x,.y,.z,.w components)
        self.vel = vel      # Velocity relative to frame in given co-ordinate system type

       