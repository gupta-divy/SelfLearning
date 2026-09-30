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
                -0.0698,
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

        # -------------------------
        # Home configuration
        # base -> end effector
        # -------------------------
        self.M = np.array([
            [1,  0,  0,  self.a7],
            [0, -1,  0,  0],
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