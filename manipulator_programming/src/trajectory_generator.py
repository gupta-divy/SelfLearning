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

class FourierTrajectory:
    def __init__(
        self,
        harmonics: int = 5,
        NJoints: int = 7,
        base_period: np.ndarray = None,
        q_init=None,
        a=None,
        b=None,
    ):
        self.N = NJoints
        self.l = harmonics
        self.T = (
            np.asarray(base_period, dtype=float)
            if base_period is not None
            else np.full(self.N, 10.0, dtype=float)
        )

        # Shape: (NJoints, harmonics)
        self.F = (
            2 * np.pi / self.T[:, None]
        ) * np.arange(1, self.l + 1)[None, :]

        self.q0 = (
            np.zeros(self.N)
            if q_init is None
            else np.asarray(q_init, dtype=float)
        )

        shape = (self.N, self.l)
        self.a = np.zeros(shape) if a is None else np.asarray(a, dtype=float)
        self.b = np.zeros(shape) if b is None else np.asarray(b, dtype=float)

    def getdesired_trajectory(self, t):

        t = np.atleast_1d(np.asarray(t,dtype=float))
        phase = t[:,None,None] * self.F[None,:,:]
        s_phase = np.sin(phase)
        c_phase = np.cos(phase)

        q = self.q0 + np.sum(
            (self.a / self.F) * s_phase +
            (self.b / self.F) * (1-c_phase),
            axis = 2
        )

        qd = np.sum(
            (self.a*c_phase)+
            (self.b*s_phase),
            axis = 2
        )

        qdd = np.sum(
            self.F*(
                (-self.a*s_phase)+
                (self.b*c_phase)
            ),
            axis=2
        )

        return q,qd,qdd