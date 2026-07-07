import numpy as np
from rigidBodyMaths import *

def pseudoInv(J):
    return np.pseudoInv(J)

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
        self.l2 = 0.8          # Link 2 length (m)

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
        s2 = ScrewToAxis(q=[0,0,self.h1], s=[0,1,0], h = 0)
        s3 = ScrewToAxis(q=[self.l1, self.h2, self.h1], s=[0,1,0], h = 0)
        self.Slist = np.column_stack((s1, s2, s3))

        self.M = np.array([
            [0, 0, 1, self.l1 + self.l2],
            [1, 0, 0, self.h2],
            [0, 1, 0, self.h1],
            [0, 0, 0, 1]
        ]) # EE at Home Position

        self.vdd0 = [0,0,0,0,0,self.g]

    def fKinSpace(self, Slist, thetaList):
        '''
        Computes the end-effector frame given the zero position of the end-effector M,
        the list of joint screws Slist expressed in the fixed-space frame, and the list of
        joint values thetalist.
        '''
        T = np.eye(4)
        for i in range(self.n):
            T = T @ expTwist(Slist[:,i], thetaList[i])
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
    
    def iKinSpace(self, Slist):
        pass
    
    def iKinBody(self,Blist):
        pass