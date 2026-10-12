import numpy as np
from roboticstoolbox import quintic


class TrajectoryGeneratorXYZ:
    def __init__(self, x_init: np.ndarray, x_final: np.ndarray, v_init: np.ndarray =  [0,0,0], v_final: np.ndarray = [0,0,0], T: float = 1.0, dt: float  = 0.001):
        '''Initialize the TrajectoryGenerator with initial and final positions in cartesian space, total time, and time step.'''
        self.x_init = np.asanyarray(x_init)
        self.x_final = np.asanyarray(x_final)
        self.v_init = np.asanyarray(v_init)
        self.v_final = np.asanyarray(v_final)
        self.T = T
        self.dt = dt
        self.t = np.arange(0, T + dt, dt)
        self.s = quintic(0, 1, self.t)


    def getdesired_trajectory(self):
        '''Generate the desired trajectory in cartesian space.'''
        x_desired = np.outer(self.s.q[:,None], np.diff([self.x_init, self.x_final], axis=0)[0]) + self.x_init
        v_desired = np.outer(self.s.qd[:,None], np.diff([self.v_init, self.v_final], axis=0)[0]) + self.v_init
        return x_desired, v_desired

    
    