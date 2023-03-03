# Contains the state class, co-ordinate system type enum and related methods
#
# Author: Harvey Merton
# Date: 02/28/2023

import numpy as np
import quaternionic as qt

from enum import Enum

# Co-ordinate system enum
class CS_type(Enum):
    #CART_INERTIAL=0,        # Inertial cartesian co-ordiante system (x, y, z)
    #CART_NON_INERTIAL=1,    # Non-inertial cartesian co-ordinate system (x, y, z)
    LLA=0,         # Planetary co-ordinate system: Latitude, Longitude, Altitude. Frame can only be 'world'
    ENU=1,                  # Local tangent plane body CS: East, North, Up (ROS2 default) 
    NED=2                   # Local tangent plane body CS: North, East, Down (Pixhawk default)

# Class to store robot state. 
# Like geometry_msgs/PoseStamped message but with different background datastructures allowing different co-ordinate system types and other calculations
class State():
    def __init__(self, frame, cs_type, pos=np.array([0.0, 0.0, 0.0]), att=qt.array([1.0, 0.0, 0.0, 0.0]), vel=np.array([0.0, 0.0, 0.0])):
        self.frame = frame
        self.cs_type = cs_type

        self.pos = pos      # Position relative to frame in given co-ordinate system type
        self.att_q = att    # Attitude relative to frame as a quaternion [x, y, z, w] (note this is ROS default but Pixhawk default has w first)
        self.vel = vel      # Velocity relative to frame in given co-ordinate system type

       