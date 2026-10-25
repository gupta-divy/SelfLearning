import numpy as np
from dataclasses import dataclass
from src import rigid_body_maths as rbm

@dataclass
class LinkKinematics:
    """Kinematics of all links; vectors expressed in the base frame."""

    poses: np.ndarray          # (n, 4, 4)
    jacobians: np.ndarray      # (n, 6, n), [angular; linear]
    omega: np.ndarray          # (n, 3)
    velocity: np.ndarray       # (n, 3), link-origin velocity
    alpha: np.ndarray          # (n, 3)
    acceleration: np.ndarray   # (n, 3), gravity not subtracted

class RobotModel:
    def __init__(self, model):
        self.n = model.n
        self.M = model.M
        self.Slist = model.Slist
        self.Blist = model.Blist
        self.joint_limits = model.joint_limits
        self.M_links = getattr(model, "M_links", None)

    def _joint_vector(self, value, name):
        value = np.asarray(value, dtype=float)
        if value.shape != (self.n,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must be a finite array of shape ({self.n},)")
        return value

    def get_link_pose(self, q):
        """Return base -> link body-frame poses, shape (n,4,4).

        Index 0 is link1, not the fixed base. Link n is not the tool frame.
        T_i(q) = exp(S1*q1) ... exp(Si*qi) M_links[i].
        """
        q = self._joint_vector(q, "q")
        if self.M_links is None:
            raise ValueError("The analytical model must define M_links")
        poses = np.empty((self.n, 4, 4))
        T = np.eye(4)
        for i in range(self.n):
            T = T @ rbm.expTwist(self.Slist[:, i], q[i])
            poses[i] = T @ self.M_links[i]
        return poses

    def get_link_jacobians(self, q):
        """Return geometric Jacobians (n,6,n), ordered [angular; linear].
        Linear rows refer to each link BODY ORIGIN. All rows are expressed
        in the base frame. Columns for joints downstream of a link are zero.
        """
        poses = self.get_link_pose(q)
        Js = self.SpaceJacobian(q)
        jacobians = np.zeros((self.n, 6, self.n))
        for i in range(self.n):

            jacobians[i, :3, :i+1] = Js[:3, :i+1]
            p = poses[i, :3, 3]
            jacobians[i, 3:, :i+1] = (
                Js[3:, :i+1] - rbm.VecToso3(p) @ Js[:3, :i+1]
            )
        return jacobians

    def get_link_kinematics(self, q, qd, qdd)->LinkKinematics:
        """Return link motion for a fixed base, with no simulator dependency.

        Dictionary entries:
          poses: (n,4,4), base -> link body frames
          jacobians: (n,6,n), [angular; linear], expressed in base
          omega, velocity, alpha, acceleration: (n,3), expressed in base

        Velocity/acceleration refer to body origins, NOT centers of mass.
        Acceleration is physical translational acceleration: gravity has NOT
        been subtracted. For an inertial regressor use R.T @ (a - gravity).
        Kinematics accepts any finite q, including a geometric zero pose;
        motion feasibility/joint-limit checks belong to the trajectory layer.
        """
        q = self._joint_vector(q, "q")
        qd = self._joint_vector(qd, "qd")
        qdd = self._joint_vector(qdd, "qdd")
        poses = self.get_link_pose(q)
        jacobians = self.get_link_jacobians(q)
        Js = self.SpaceJacobian(q)
        # d(Js[:,j])/dt = ad(V_upstream) Js[:,j].
        # This differentiates the spatial screw Jacobian analytically.
        Jsdot = np.zeros_like(Js)
        upstream_twist = np.zeros(6)
        for j in range(self.n):
            Jsdot[:, j] = rbm.ad(upstream_twist) @ Js[:, j]
            upstream_twist += Js[:, j] * qd[j]

        omega, velocity = np.zeros((self.n, 3)), np.zeros((self.n, 3))
        alpha, acceleration = np.zeros_like(omega), np.zeros_like(velocity)
        for i in range(self.n):
            # Spatial twist linear component differs from origin velocity:
            # p_dot = v_spatial + omega x p.
            active = slice(0, i+1)
            twist = Js[:, active] @ qd[active]
            twist_dot = (Js[:, active] @ qdd[active]
                         + Jsdot[:, active] @ qd[active])
            p = poses[i, :3, 3]
            omega[i], alpha[i] = twist[:3], twist_dot[:3]
            velocity[i] = twist[3:] + np.cross(omega[i], p)
            acceleration[i] = (twist_dot[3:] + np.cross(alpha[i], p)
                               + np.cross(omega[i], velocity[i]))
        return LinkKinematics(
            poses=poses,
            jacobians=jacobians,
            omega=omega,
            velocity=velocity,
            alpha=alpha,
            acceleration=acceleration,
        )

    def get_link_wrench(self, pi, omega, alpha, a):
        '''
        Returns force and moment about the link-frame origin.
        Given all vectors (omega, alpha and a) are in link frame
        and a is origin acceleration.

        pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        hx = m*cx where cx is COM x position wrt link-frame origin
        '''
        m = pi[0]
        h = pi[1:4]
        I = np.array([
            [pi[4], pi[7], pi[8]],
            [pi[7], pi[5], pi[9]],
            [pi[8], pi[9], pi[6]]
        ])

        force = (
            m*a + 
            np.cross(alpha, h) +
            np.cross(omega, np.cross(omega,h))
        )

        moment = (
            I @ alpha
            + np.cross(omega, I@omega)
            + np.cross(h, a)
        )

        return force, moment

    def FK(self, q):
        """
        Forward Kinematics: Computes the end-effector pose given joint angles q.
        """
        # chek if q size matches the number of joints
        if len(q) != self.n:
            raise ValueError(f"Expected {self.n} joint angles, but got {len(q)}.")
        
        # Check if q is within joint limits
        for i in range(len(q)):
            if q[i] < self.joint_limits["lower"][i] or q[i] > self.joint_limits["upper"][i]:
                raise ValueError(f"Joint angle {q[i]} is out of limits for joint {i}.")
        
        T = np.eye(4)
        for i in range(self.n):
            T = T @ rbm.expTwist(self.Slist[:,i], q[i])
        T = T @ self.M
        return T

    def SpaceJacobian(self, q):
        """
        Space Jacobian:
        Js[:, i] = Ad(T_i)*S_i
        where:
        T_i = e^[S1]θ1 ... e^[S_{i-1}]θ_{i-1}
        """

        T = np.eye(4)
        Js = np.zeros((6,self.n))
        Js[:,0] = self.Slist[:,0]
        for i in range(1,self.n):
            T = T @ rbm.expTwist(self.Slist[:,i-1], q[i-1])
            Js[:,i] = rbm.Adjoint(T)@self.Slist[:,i]
        return Js

    def RobotJacobian(self, q):
        """
        Returns the Jacobian in the space frame.
        """
        JSpace = self.SpaceJacobian(q)
        eePos = self.FK(q)[:3, 3]

        j_omega = JSpace[0:3,:]
        j_v = JSpace[3:6,:]

        j_v = j_v - rbm.VecToso3(eePos)@j_omega

        J = np.vstack((j_omega, j_v))
        return J
    
    def InverseKinematics(self, T_sd, q0, epsilon=1e-3, max_iterations=1000):
        """
        Inverse Kinematics: Computes the joint angles q that achieve the desired end-effector pose T_sd.
        Uses an iterative method to find a solution.
        """

        q = np.array(q0, dtype=float)
        for i in range(max_iterations):
            T_sb = self.FK(q)
            T_bd = rbm.TransInv(T_sb) @ T_sd
            Vb = rbm.se3ToVec(rbm.MatrixLog6(T_bd))

            if np.linalg.norm(Vb) < epsilon:
                return q
            
            Jb = rbm.Adjoint(rbm.TransInv(T_sb)) @ self.SpaceJacobian(q)
            damp = 0.05
            dq = Jb.T @ np.linalg.inv(Jb@Jb.T + damp**2 * np.eye(6)) @ Vb

            max_step = 0.1
            dq = np.clip(dq, -max_step, max_step)

            q = np.clip(q+dq,self.joint_limits["lower"],self.joint_limits["upper"])
        
        raise ValueError("Inverse kinematics did not converge within the maximum number of iterations.")
        
    

