from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml        
import os
from simulation_env import MujocoEnvironment

class Robot:
    def __init__(self, robotConfigPath):
        with open(robotConfigPath,'r') as file:
            config = yaml.safe_load(file)
        self.dh_matrix = np.array(config["dh_matrix"], dtype=float)
        self.joint_limits = np.array(config["joint_limits"], dtype=float)
        self.xml_file = config["xml_file"]
        self.name = config["robot_name"]
        self.sim_env = MujocoEnvironment(self.xml_file)

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
        return self.sim_env.initialize_and_check_collision(joint_angles=joint_config)
    
    def __dh_transform(self, a, alpha, d, theta):
        """ Compute the DH transformation matrix for a single joint """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        
if __name__=="__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    robot = Robot("ur5_config.yaml")

    # Example joint configuration to test
    test_joint_angles = [0, -np.pi/4, np.pi/4, 0, np.pi/2, 0]
    collision = robot.is_colliding_with_obstacles(test_joint_angles)