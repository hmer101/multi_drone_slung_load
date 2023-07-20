# Script to generate spawn drone points to use for simulation
#
# Author: Harvey Merton 
# Date: 07/19/2023

import numpy as np
import quaternionic as qt

import utils



def main(args=None):
    # Transform orientation
    # q_Bd = qt.array([0.5, 0.0, 0.5, 0.0])
    # q_BC = qt.array([0.71, 0.25, 0.0, 0.0])
    # q_Cd = utils.transform_orientation(q_BC, q_Bd)
    # print(f'q_Cd: {q_Cd} \n')
    
    # Transform position #### TODO: HEREE!!! Remember to edit the function in utils
    p_Bd = np.array([1.0, 0.0, 0.0])
    
    p_BC = np.array([0.0, 0.0, 0.0]) #np.array([-1.0, 0.5, 1.0])
    q_BC = qt.array([0.0, 0.0, 0.0, -1.0])
    
    p_Cd = utils.transform_position(p_Bd, -p_BC, 1/q_BC)
    
    print(f'p_Cd: {p_Cd}')
    

if __name__ == '__main__':
    main()