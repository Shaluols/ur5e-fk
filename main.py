import numpy as np
import sys
from fk import *

if __name__ == "__main__":
    # joints values in radian
    q = [1.6094996624070816, -0.014978588575139895, -1.6244180820206449, -2.5102484798106426, -0.25636845564632615,
         -1.5114861234682682]
    # get 4*4 matrix of the end-effector pose
    np_T = forward(q)
    # transfer the format into ros msg type
    ros_T = matrix2ros(np_T)
    # transfer the format into x, y, z, euler angles type
    ur_T = ros2euler(ros_T)
    print("\n 4*4 matrix:\n", np_T)

    print("Ros msg type:\n", ros_T)

    print("Euler angles type:\n", ur_T)
