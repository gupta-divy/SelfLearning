import numpy as np
from roboticstoolbox import quintic
from spatialmath import SE3
from roboticstoolbox.tools.trajectory import ctraj
from scipy.spatial.transform import Rotation


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


class TrajectoryGeneratorSE3:
    """
    Generate a Cartesian SE(3) trajectory between two poses.

    Inputs:
        x_init  : Initial homogeneous transformation (4x4)
        x_final : Final homogeneous transformation (4x4)
        T       : Total trajectory duration (seconds)
        dt      : Sampling time (seconds)

    Outputs:
        T_desired     : (N, 4, 4) desired poses
        v_desired     : (N, 3) linear velocities (world frame)
        omega_desired : (N, 3) angular velocities (world frame)
    """

    def __init__(
        self,
        x_init: np.ndarray,
        x_final: np.ndarray,
        T: float = 1.0,
        dt: float = 0.001
    ):
        self.x_init = SE3(x_init)
        self.x_final = SE3(x_final)

        self.T = T
        self.dt = dt

        self.t = np.linspace(
            0, T, int(round(T / dt)) + 1
        )

    def getdesired_trajectory(self):

        # Generate interpolated SE(3) poses
        trajectory = ctraj(
            self.x_init,
            self.x_final,
            t=self.t
        )
        

        # Convert SE3 objects into homogeneous matrices
        T_desired = np.array([pose.A for pose in trajectory])

        # Extract position
        p_desired = T_desired[:, :3, 3]

        # Linear velocity
        v_desired = np.gradient(
            p_desired,
            self.t,
            axis=0
        )

        # Angular velocity (world frame)
        omega_desired = np.zeros((len(self.t), 3))

        for i in range(len(self.t) - 1):

            R_current = T_desired[i, :3, :3]
            R_next = T_desired[i + 1, :3, :3]

            # Relative rotation in world frame
            R_delta = R_next @ R_current.T

            rotvec = Rotation.from_matrix(
                R_delta
            ).as_rotvec()

            omega_desired[i] = rotvec / (
                self.t[i + 1] - self.t[i]
            )

        # Endpoint angular velocity
        omega_desired[-1] = omega_desired[-2]

        return T_desired, v_desired, omega_desired