import numpy as np
import math
import time

from enum import Enum
from sklearn.cluster import DBSCAN
from config import *

class Mode(Enum):
    FORMATION = 0
    TAILGATING = 1

class Drone:
    def __init__(self, index:int, state:np.array, control:np.array, radius, known_obs):
        self.index = index
        
        # Drone state and control
        self.time_stamp = 0.0
        self.state = state
        self.control = control

        self.n_state = 6
        self.n_control = 3

        self.gt = self.state

        # Drone radius
        self.radius = radius
        
        # Store drone path
        self.path = [np.concatenate([[self.time_stamp], self.state, self.control])]
        self.__update_known_obs(known_obs)

    def updateState(self, control:np.array, dt:float, known_obs):
        """
        Computes the states of drone after applying control signals
        """
        # Update
        position = self.state[:3]
        velocity = self.state[3:]
        
        next_velocity = velocity + control*dt
        avg_velo = velocity + 0.5 * control *dt
        # Control signal alignment
        next_position = position + avg_velo*dt

        self.state = np.concatenate([next_position, next_velocity])
        self.control = control
        self.time_stamp = self.time_stamp + dt

        self.path.append(np.concatenate([[self.time_stamp], self.state, self.control]))
        self.__update_known_obs(known_obs)

    def setupController(self, dt=0.1):
        # nmpc timestep
        self.timestep = dt

    def computeControlSignal(self, drones, known_obs):
        """
        Computes control velocity of the copter
        """
        # state = self.state.copy()
        observed_obstacles = self.observerObstacles(known_obs)

        v_m = self.behaviorMigration()
        v_f = self.behaviorSeparation(drones)
        v_o = self.behaviorObstacle(observed_obstacles)
        v_c = self.behaviorCollision(drones)
        vel = W_nav*v_m + W_sep*v_f + W_obs*v_o + W_col*v_c
        if np.linalg.norm(vel) > VMAX:
            vel = vel * VMAX / np.linalg.norm(vel)
        control = 2*(vel - self.state[3:])/self.timestep
        constrained_control = self.__apply_control_constrains(control)
        # print(constrained_control)
        return constrained_control

    def __update_known_obs(self, known_obs):
        '''Update known obstacles by sensor'''
        for i in range(OBSTACLES.shape[0]):
            distance = np.linalg.norm(self.state[:2] - OBSTACLES[i, :2])
            if distance - OBSTACLES[i, 2] < SENSING_RADIUS:
                known_obs.add(i) 

    def __apply_control_constrains(self, control:np.array):
        smaller = control < CONTROL_BOUNDS[:, 0]
        greater = control > CONTROL_BOUNDS[:, 1]
        constrained_control = np.zeros(self.n_control)
        for i in range(self.n_control):
            if smaller[i]:
                constrained_control[i] = CONTROL_BOUNDS[i, 0]
            elif greater[i]:
                constrained_control[i] = CONTROL_BOUNDS[i, 1]
            else:
                constrained_control[i] = control[i]
        return constrained_control
        


    @staticmethod
    def behaviorMigration():
        return VREF*UREF
    
    def behaviorSeparation(self, drones):
        v_s = np.zeros(self.n_control)
        for i in range(NUM_UAV):
            if i == self.index:
                continue
            pos_rel = drones[i].state[:3]-self.state[:3]
            dir = pos_rel/np.linalg.norm(pos_rel)
            # v_s += (drones[i].state[:3]-self.state[:3]) - DREF*dir
            v_s += (np.linalg.norm(pos_rel)-DREF)*dir
        return v_s

    def behaviorObstacle(self, obstacles):
        vel_obs = np.zeros(self.n_control)
        if obstacles.shape[0] == 0:
            return vel_obs
        for i in range(obstacles.shape[0]):
            obs = obstacles[i]
            obs_rel = self.state[:2] - obs[:2]
            rs = np.linalg.norm(obs_rel)
            dir = np.concatenate([obs_rel,[0]])/np.linalg.norm(rs)
            # vel_obs += dir*np.exp(-BETA*(rs-ROBOT_RADIUS-obs[2]))/(rs-ROBOT_RADIUS-obs[2])
            vel_obs += math.exp(rs - (ROBOT_RADIUS+obs[2]))/(rs - (ROBOT_RADIUS+obs[2]))*dir
        return vel_obs/obstacles.shape[0]
    
    def behaviorCollision(self, drones):
        vel_col = np.zeros(self.n_control)
        for j in range(NUM_UAV):
            if j == self.index:
                continue
            pos_rel = drones[j].state[:3] - self.state[:3]
            rs = np.linalg.norm(pos_rel)
            if rs >= DREF:
                continue
            dir = -pos_rel/rs
            # vel_col += dir*(1 - np.tanh(BETA*(rs - 2*ROBOT_RADIUS)))/2
            # vel_col += -dir*np.exp(-BETA*(rs-2*ROBOT_RADIUS))/(rs-2*ROBOT_RADIUS)
            vel_col += math.exp(rs - 2*ROBOT_RADIUS)/(rs - 2*ROBOT_RADIUS)*dir
        vel_col /= (NUM_UAV-1)
        return vel_col

    def observerObstacles(self, known_obs):
        observed_obstacles = []
        for j in known_obs:
            obs = OBSTACLES[j,:]
            if np.linalg.norm(self.state[:2]-obs[:2]) <= SENSING_RADIUS:
                observed_obstacles.append(obs)
            # observed_obstacles.append(obs)
        return np.array(observed_obstacles)