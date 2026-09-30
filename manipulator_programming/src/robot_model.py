import numpy as np

from src import rigid_body_maths as rbm

class RobotModel:
    def __init__(self, model):
        self.n = model.n
        self.M = model.M
        self.Slist = model.Slist
        self.Blist = model.Blist
        self.joint_limits = model.joint_limits


    def FK(self, q):
        """
        Forward Kinematics: Computes the end-effector pose given joint angles q.
        """
        # chek if q size matches the number of joints
        if len(q) != self.n:
            raise ValueError(f"Expected {self.n} joint angles, but got {len(q)}.")
        
        # Check if q is within joint limits
        for i in range(len(q)):
            if q[i] <= self.joint_limits["lower"][i] or q[i] >= self.joint_limits["upper"][i]:
                raise ValueError(f"Joint angle {q[i]} is out of limits for joint {i}.")
        
        T = np.eye(4)
        for i in range(self.n):
            T = T @ rbm.expTwist(self.Slist[:,i], q[i])
        T = T @ self.M
        return T
