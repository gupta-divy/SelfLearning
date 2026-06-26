import numpy as np

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
        self.h1 = 0.05
        self.l1 = 1.0          # Link 1 length (m)
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
        self.tau = np.zeros(3)     # Joint torques

        # -------------------------
        # Frame Definitions
        # -------------------------
        omega1 = np.asarray([0,0,1])
        p1 = np.asarray([0,0,0])
        v1 = -np.cross(omega1, p1)
        s1 = np.concatenate((omega1, v1))

        omega2 = np.asarray([0,1,0])
        # p2 = np.asarray([0,0,self.h1])
        p2 = np.asarray([0,0,0])
        v2 = -np.cross(omega2, p2)
        s2 = np.concatenate((omega2, v2))

        omega3 = np.asarray([0,1,0])
        # p3 = np.asarray([self.l1,self.h2,self.h1])
        p3 = np.asarray([self.l1,0,0])
        v3 = -np.cross(omega3, p3)
        s3 = np.concatenate((omega3, v3))

        self.Slist = np.column_stack((s1, s2, s3))
        self.M = np.array([
            [1, 0, 0, self.l1 + self.l2],
            [0, 0,-1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])