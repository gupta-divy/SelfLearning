import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.rigid_body_maths import *

class ThreeDOFManipulator:
    """
    3-DOF Manipulator
        Joint 1 : Revolute (Yaw)
        Joint 2 : Revolute (Shoulder)
        Joint 3 : Revolute (Elbow)
    """

    def __init__(self):

        # -------------------------
        # Link Parameters
        # -------------------------
        self.n = 3
        self.h1 = 0.05
        self.l1 = 1.0          # Link 1 length (m)
        self.h2 = 0.05
        self.l2 = 0.5          # Link 2 length (m)
        

        self.lc1 = 0.5         # COM of link 1 from joint (m)
        self.lc2 = 0.4         # COM of link 2 from joint (m)

        # -------------------------
        # Mass Properties
        # -------------------------
        self.m1 = 2.0          # Link 1 mass (kg)
        self.m2 = 1.5          # Link 2 mass (kg)

        self.J1 = 0.08         # Link 1 inertia about COM
        self.J2 = 0.04         # Link 2 inertia about COM

        # -------------------------
        # Gravity
        # -------------------------
        self.g = 9.81

        # -------------------------
        # Joint States
        # -------------------------
        self.q = np.zeros(3)       # [theta1, theta2, theta3]
        self.qd = np.zeros(3)      # Joint velocities
        self.qdd = np.zeros(3)     # Joint accelerations

        # -------------------------
        # Dynamic Quantities
        # -------------------------
        self.Mi = np.zeros((3, 3))  # Mass matrix
        self.Cr = np.zeros((3, 3))  # Coriolis matrix
        self.Gr = np.zeros(3)       # Gravity vector
        self.tau = np.zeros(3)      # Joint torques

        # -------------------------
        # Frame Definitions
        # -------------------------
        s1 = ScrewToAxis(q=[0,0,0], s=[0,0,1], h = 0)
        s2 = ScrewToAxis(q=[0,0,self.h1], s=[0,-1,0], h = 0)
        s3 = ScrewToAxis(q=[self.l1, self.h2, self.h1], s=[0,-1,0], h = 0)
        self.Slist = np.column_stack((s1, s2, s3))

        b3 = ScrewToAxis(q=[0,0,-self.l2], s=[-1,0,0], h=0)
        b2 = ScrewToAxis(q=[-self.h2,0,-self.l2-self.l1], s=[-1,0,0], h=0)
        b1 = ScrewToAxis(q=[-self.h2,-self.h1,-self.l2-self.l1], s=[0,1,0], h=0)  
        self.Blist = np.column_stack((b1, b2, b3))

        self.M = np.array([
            [0, 0, 1, self.l1 + self.l2],
            [1, 0, 0, self.h2],
            [0, 1, 0, self.h1],
            [0, 0, 0, 1]
        ]) # EE at Home Position

        self.vdd0 = [0,0,0,0,0,self.g]

    def fKinSpace(self, thetaList):
        '''
        Computes the end-effector frame given the zero position of the end-effector M,
        the list of joint screws Slist expressed in the fixed-space frame, and the list of
        joint values thetalist.
        '''
        T = np.eye(4)
        for i in range(self.n):
            T = T @ expTwist(self.Slist[:,i], thetaList[i])
        T = T @ self.M
        return T
    
    def getSpaceJacobian(self, thetaList):
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
            T = T @ expTwist(self.Slist[:,i-1], thetaList[i-1])
            Js[:,i] = Adjoint(T)@self.Slist[:,i]
        return Js
    
    def getBodyJacobian(self, thetaList):
        """
        Body Jacobian:
        Jb[:, i] = Ad(T_i)^-1 @ B_i
        where:
        T_i = e^[B_{i+1}]theta_{i+1} ... e^[B_n]theta_n
        """
        T = np.eye(4)
        Jb = np.zeros((6, self.n))
        Jb[:, self.n - 1] = self.Blist[:, self.n - 1]

        for i in range(self.n - 2, -1, -1):
            T = T @ expTwist(self.Blist[:, i + 1], thetaList[i + 1])
            Jb[:, i] = Adjoint(TransInv(T)) @ self.Blist[:, i]

        return Jb

    def getEndEffectorPositionJacobian(self, thetaList):
        """
        Position Jacobian for the end-effector origin in the space frame.

        The space Jacobian column is a twist [omega, v]. The linear velocity
        of a point at position p is p_dot = omega x p + v.
        """
        thetaList = np.asarray(thetaList, dtype=float).reshape(self.n)
        T_sb = self.fKinSpace(thetaList)
        p = T_sb[:3, 3]
        Js = self.getSpaceJacobian(thetaList)

        Jp = np.zeros((3, self.n))
        for i in range(self.n):
            omega = Js[:3, i]
            v = Js[3:, i]
            Jp[:, i] = np.cross(omega, p) + v

        return Jp
    
    def iKinBody(self, T_sd, thetaZero, ev, eomg, maxIterations=100):
        k = 0
        thetaNew = np.array(thetaZero, dtype=float).copy()
        thetaOld = thetaNew.copy()
        isErrCriteriaMet = False

        while k < maxIterations and not isErrCriteriaMet:
            T_sb = self.fKinSpace(thetaOld)
            T_bs = TransInv(T_sb)
            T_bd = T_bs @ T_sd
            Vb = se3ToVec(MatrixLog6(T_bd))

            Jb = self.getBodyJacobian(thetaOld)
            thetaNew = thetaOld + np.linalg.pinv(Jb) @ Vb

            isErrCriteriaMet = (np.linalg.norm(Vb[:3]) < eomg) and (np.linalg.norm(Vb[3:]) < ev)
            thetaOld = thetaNew
            k += 1

        return [thetaNew, isErrCriteriaMet, k]

    def iKinPositionBodyMR(
        self,
        p_d,
        thetaZero,
        ep=0.01,
        maxIterations=100,
        damping=1e-3,
        stepScale=0.5,
        wrapAngles=True,
    ):
        """
        Solve position-only IK using the Modern Robotics body-frame error idea.

        At each iteration, the desired transform keeps the current end-effector
        orientation and only changes the end-effector position. This makes the
        MatrixLog6 error a pure body-frame translation error.
        """
        p_d = np.asarray(p_d, dtype=float).reshape(3)
        thetaNew = np.array(thetaZero, dtype=float).reshape(self.n).copy()
        thetaOld = thetaNew.copy()
        isErrCriteriaMet = False
        bestTheta = thetaOld.copy()
        bestErr = np.inf
        k = 0

        while k < maxIterations and not isErrCriteriaMet:
            T_sb = self.fKinSpace(thetaOld)
            R_sb = T_sb[:3, :3]
            p_current = T_sb[:3, 3]
            p_err = p_d - p_current
            errNorm = np.linalg.norm(p_err)

            if errNorm < bestErr:
                bestErr = errNorm
                bestTheta = thetaOld.copy()

            isErrCriteriaMet = errNorm < ep
            if isErrCriteriaMet:
                break

            T_sd = RpToTrans(R_sb, p_d)
            T_bd = TransInv(T_sb) @ T_sd
            Vb = se3ToVec(MatrixLog6(T_bd))

            Jb = self.getBodyJacobian(thetaOld)
            Jb_pos = Jb[3:, :]
            thetaStep = Jb_pos.T @ np.linalg.solve(
                Jb_pos @ Jb_pos.T + (damping ** 2) * np.eye(3),
                Vb[3:],
            )
            thetaNew = thetaOld + stepScale * thetaStep

            if wrapAngles:
                thetaNew = (thetaNew + np.pi) % (2.0 * np.pi) - np.pi

            thetaOld = thetaNew
            k += 1

        if not isErrCriteriaMet:
            thetaNew = bestTheta

        return [thetaNew, isErrCriteriaMet, k]

    def iKinPosition(
        self,
        p_d,
        thetaZero,
        ep=0.01,
        maxIterations=100,
        damping=1e-3,
        stepScale=0.5,
        wrapAngles=True,
    ):
        """
        Solve inverse kinematics for end-effector position only.

        Parameters
        ----------
        p_d : array-like, shape (3,)
            Desired end-effector position in the world frame.
        thetaZero : array-like, shape (n,)
            Initial joint-angle guess in radians.
        ep : float
            Position error tolerance in meters.
        maxIterations : int
            Maximum number of Newton-style iterations.
        damping : float
            Damping used in the least-squares inverse for better numerical stability.
        stepScale : float
            Step-size scaling factor in (0, 1].
        wrapAngles : bool
            If True, wrap revolute joint angles to [-pi, pi].
        """
        p_d = np.asarray(p_d, dtype=float).reshape(3)
        thetaNew = np.array(thetaZero, dtype=float).reshape(self.n).copy()
        thetaOld = thetaNew.copy()
        isErrCriteriaMet = False
        bestTheta = thetaOld.copy()
        bestErr = np.inf
        k = 0

        while k < maxIterations and not isErrCriteriaMet:
            T_sb = self.fKinSpace(thetaOld)
            p_current = T_sb[:3, 3]
            p_err = p_d - p_current
            errNorm = np.linalg.norm(p_err)

            if errNorm < bestErr:
                bestErr = errNorm
                bestTheta = thetaOld.copy()

            isErrCriteriaMet = errNorm < ep
            if isErrCriteriaMet:
                break

            Jp = self.getEndEffectorPositionJacobian(thetaOld)

            # Damped least-squares update for position-only IK.
            JT = Jp.T
            thetaStep = JT @ np.linalg.solve(
                Jp @ JT + (damping ** 2) * np.eye(3),
                p_err,
            )
            thetaNew = thetaOld + stepScale * thetaStep

            if wrapAngles:
                thetaNew = (thetaNew + np.pi) % (2.0 * np.pi) - np.pi

            thetaOld = thetaNew
            k += 1

        if not isErrCriteriaMet:
            thetaNew = bestTheta

        return [thetaNew, isErrCriteriaMet, k]
