# Contains the Pose class and related methods
#
# Author: Harvey Merton
# Date: 02/28/2023

import numpy as np
import quaternionic as qt

class Pose():
    def __init__(self, frame, pos=np.array([0.0, 0.0, 0.0]), att=qt.array([0.0, 0.0, 0.0, 1.0]), vel=np.array([0.0, 0.0, 0.0])):
        self.frame = frame

        self.pos_cart = pos      # Position relative to frame in cartesian co-ordinates
        self.att_q = att         # Attitude relative to frame as a quaternion
        self.vel_cart = vel      # Velocity relative to frame in cartesian co-ordinates

       