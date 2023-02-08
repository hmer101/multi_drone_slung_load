import numpy as np


if __name__=='__main__':
    t_array = np.array([[1, 1, 1],
              [2,2,2],
              [3,3,3]])
    
    print(np.average(t_array, axis=0))
    