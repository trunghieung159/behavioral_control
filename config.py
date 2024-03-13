import numpy as np
import time
import matplotlib.pyplot as plt
import math 

DT = 0.05
DRONE_R = 0.3
SENSOR_R = 3.0
EPSILON = 0.1

NUM_UAV = 3

VREF = 1.0
UREF = np.array([1,0,0])
DREF = 1.5

HEIGHT_BOUNDS = np.array([2.0, 20.0])

CONTROL_BOUNDS = np.array([[-2.0, 2.0],
                           [-2.0, 2.0],
                           [-1.0, 1.0]])

VELO_BOUNDS = np.array([[-2.0, 2.0],
                        [-2.0, 2.0],
                        [-0.5, 0.5]])
VMAX = 2.0


BETA = 1.0

W_sep = 8.0/20
W_dir = 2.0/20
W_nav = 1.0
W_u = 4e-1
W_obs = 4.5/20
W_col = 1.0/20

# Position x, y ,z
INIT_POSITIONS = np.array([[1.5, 0.3, 5.0],
                           [0.0, 0.1, 5.0],
                           [0.4, 1.2, 5.0]])

# Velocity v_x, v_y, v_z
INIT_VELOS = np.array([[0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0]])

INIT_STATES = np.concatenate([INIT_POSITIONS, INIT_VELOS], axis=1)

X_GOAL = 12.0
# Obstacle x, y, r

# #CASE1
# OBSTACLES = np.array([[4.0, 2.0, 0.1],
#                       [5.0,-1.0, 0.1],
#                       [6.0, 1.0, 0.1],
#                       [6.0, 4.0, 0.2],
#                       [8.0,-1.0, 0.1],
#                       [8.0, 2.0, 0.1],
#                       [9.0, 0.0, 0.1],])

# #CASE2
# OBSTACLES = np.array([[4.0, 0.0, 0.1],
#                       [4.0,-2.0, 0.1],
#                       [6.0,-1.5, 0.1],
#                       [6.0, 1.0, 0.1],
#                       [6.0, 2.0, 0.1],
#                       [7.0, 3.0, 0.1],
#                       [8.0, 4.5, 0.1],
#                       [9.0, 6.0, 0.1],])

#CASE3
OBSTACLES = np.array([[4.0,-2.0, 0.1],
                      [4.0, 1.0, 0.1],
                      [4.0, 4.0, 0.1],
                      [5.0,-3.0, 0.1],
                      [5.0, 2.0, 0.1],
                      [6.0, 6.0, 0.1],
                      [7.0,-1.0, 0.1],
                      [8.0, 0.0, 0.1]])
