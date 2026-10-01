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

