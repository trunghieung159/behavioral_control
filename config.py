import numpy as np
import time
import matplotlib.pyplot as plt
import math 

TIMESTEP = 0.05
ROBOT_RADIUS = 0.3
SENSING_RADIUS = 3.0
EPSILON = 0.1
TOPOLOGY = np.array([[-np.sqrt(3)/2,-0.5, 0.0],
                     [ np.sqrt(3)/2,-0.5, 0.0],
                     [          0.0, 1.0, 0.0]])
NUM_UAV = TOPOLOGY.shape[0]

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

# Scenario
STARTS = np.array([[1.5, 0.3, 5.0, 0, 0, 0],
                   [0.0, 0.1, 5.0, 0, 0, 0],
                   [0.4, 1.2, 5.0, 0, 0, 0]])

X_GOAL = 14.0
# Obstacle x, y, r

##CASE1
# OBSTACLES = np.array([[4.0, 2.0, 0.1],
#                       [5.0,-1.0, 0.1],
#                       [6.0, 1.0, 0.1],
#                       [6.0, 4.0, 0.2],
#                       [8.0,-1.0, 0.1],
#                       [8.0, 2.0, 0.1],
#                       [9.0, 0.0, 0.1],])

#CASE2
OBSTACLES = np.array([[4.0, 0.0, 0.1],
                      [4.0,-2.0, 0.1],
                      [6.0,-1.5, 0.1],
                      [6.0, 1.0, 0.1],
                      [6.0, 2.0, 0.1],
                      [7.0, 3.0, 0.1],
                      [8.0, 4.5, 0.1],
                      [9.0, 6.0, 0.1],])

##CASE3
# OBSTACLES = np.array([[4.0,-2.0, 0.1],
#                       [4.0, 1.0, 0.1],
#                       [4.0, 4.0, 0.1],
#                       [5.0,-3.0, 0.1],
#                       [5.0, 2.0, 0.1],
#                       [6.0, 6.0, 0.1],
#                       [7.0,-1.0, 0.1],
#                       [8.0, 0.0, 0.1]])
