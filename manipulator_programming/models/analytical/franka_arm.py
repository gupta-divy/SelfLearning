import numpy as np


class FrankaArm:
    """
    Kinematic parameters for the Franka Emika Panda robot.

    Screw axes follow the Modern Robotics convention:
        S = [omega; v]
        B = [omega; v]

    Frames are based on the Panda URDF zero configuration.
    """

    def __init__(self):
        
        self.n = 7  # Number of joints

        # -------------------------
        # Geometric parameters [m]
        # -------------------------
        self.d1 = 0.333
        self.d3 = 0.316
        self.a4 = 0.0825
        self.d5 = 0.384
        self.a5 = -0.0825
        self.a7 = 0.088
        self.d8 = 0.107
        self.d9 = 0.0584 # figertip and hand offset

        # -------------------------
        # Joint limits [rad]
        # -------------------------
        self.joint_limits = {
            "lower": np.array([
                -2.8973,
                -1.7628,
                -2.8973,
                -3.0718,
                -2.8973,
                -0.0175,
                -2.8973
            ]),

            "upper": np.array([
                 2.8973,
                 1.7628,
                 2.8973,
                 0.0000,
                 2.8973,
                 3.7525,
                 2.8973
            ])
        }

        # -------------------------
        # Space screw axes
        # -------------------------
        self.Slist = np.array([
            [0,  0,  1,  0,       0,      0],
            [0,  1,  0, -self.d1, 0,      0],
            [0,  0,  1,  0,       0,      0],
            [0, -1,  0, self.d1 + self.d3, 0, -self.a4],
            [0,  0,  1,  0,       0,      0],
            [0, -1,  0, self.d1 + self.d3 + self.d5, 0, 0],
            [0,  0, -1,  0,       self.a7, 0]
        ]).T

        # Base -> each moving link's body frame at q = 0. These are NOT
        # center-of-mass frames or the tool frame M. Store geometry here so
        # RobotModel can compute link kinematics without a simulator.
        offsets = [
            [0, 0, self.d1], [0, 0, 0], [0, -self.d3, 0],
            [self.a4, 0, 0], [self.a5, self.d5, 0],
            [0, 0, 0], [self.a7, 0, 0],
        ]
        x_angles = [0, -np.pi/2, np.pi/2, np.pi/2,
                    -np.pi/2, np.pi/2, np.pi/2]
        self.M_links = np.zeros((self.n, 4, 4))
        T_link = np.eye(4)
        for i, (offset, angle) in enumerate(zip(offsets, x_angles)):
            c, s = np.cos(angle), np.sin(angle)
            T_fixed = np.eye(4)
            T_fixed[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
            T_fixed[:3, 3] = offset
            T_link = T_link @ T_fixed
            self.M_links[i] = T_link

        # -------------------------
        # Home configuration
        # base -> end effector
        # -------------------------
        self.M = np.array([
            [0.70710681,  0.70710681,  0,  self.a7],
            [0.70710681, -0.70710681,  0,  0],
            [0,  0, -1,  self.d1 + self.d3 + self.d5 - self.d8 - self.d9],
            [0,  0,  0,  1]
        ])

        # -------------------------
        # Body screw axes
        # -------------------------
        self.Blist = np.array([
            [0,  0, -1,  0,      -self.a7, 0],
            [0, -1,  0,  self.d3 + self.d5 - self.d8, 0, self.a7],
            [0,  0, -1,  0,      -self.a7, 0],
            [0,  1,  0, -(self.d5 - self.d8), 0, -(self.a7 - self.a4)],
            [0,  0, -1,  0,      -self.a7, 0],
            [0,  1,  0,  self.d8, 0, -self.a7],
            [0,  0,  1,  0,       0, 0]
        ]).T


if __name__ == "__main__":
    robot = FrankaArm()

    print("Slist:\n", robot.Slist)
    print("\nM:\n", robot.M)
    print("\nBlist:\n", robot.Blist)
