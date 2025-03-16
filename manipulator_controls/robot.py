from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml

class Robot:
    def __init__(self, robotConfigPath):
        with open(robotConfigPath,'r') as file:
            config = yaml.safe_load(file)

        self.dh_matrix = np.array(config["dh_matrix"], dtype=float)
        self.joint_limits = np.array(config["joint_limits"], dtype=float)
        self.xml_file = config["xml_file"]
    
    def forward_kinematics(self, joint_ang):
        '''Update end-effector pos based on joint angles'''
        assert len(joint_ang)==len(self.dh_matrix)

        T = np.eye(4)
        for i, (a,alpha,d,_) in enumerate(self.dh_matrix):
            theta = joint_ang[i]
            T_i = self.__dh_transform(a,alpha,d,theta)
            T = np.dot(T,T_i)
        eef_position = T[:3,3]
        eef_orientation = T[:3,:3]
        eef_rpy = R.from_matrix(eef_orientation).as_euler('xyz', degrees=True)
        return np.hstack((eef_position,eef_rpy))

    def inverse_kinematics(self, eef_pose):
        '''Update joint angles for the given end-effector pose'''
        pass

    def is_colliding_with_obstacles(self, joint_config):
        '''Will use MuJoCo collision detector to see if it happens for particular joint configuration'''
        return False
    
    def __dh_transform(self, a, alpha, d, theta):
        """ Compute the DH transformation matrix for a single joint """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

if __name__=="main":
    ur5_robot = Robot("ur5_config.yaml")
    print("DH Matrix: ", ur5_robot.dh_matrix)
    print("Joint Limits: ", ur5_robot.joint_limits)

