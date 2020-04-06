import numpy as np
import sys
from fk import *

if __name__ == "__main__":
    # joints values in radian
    q = [0, 0, 0, 0, 0, 0]
    # get 4*4 matrix of the end-effector pose
    # this function only give you the final result of the Homogeneous matrix from base_link to end_efffector
    # if you want to get intermediate result, you can use fwd_kin function
    np_T = forward(q)
    # np_T = fwd_kin(q)
    
    # transfer the format into ros msg type
    ros_T = matrix2ros(np_T)
    # transfer the format into x, y, z, euler angles type
    ur_T = ros2euler(ros_T)
    print("\n 4*4 matrix:\n", np_T)

    print("Ros msg type:\n", ros_T)

    print("Euler angles type:\n", ur_T)
