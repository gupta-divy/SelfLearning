from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
import numpy as np
import yaml        
import os
from simulation_env import MujocoEnvironment
workspace_limits = {
    "x_min": -0.5, "x_max": 0.5,
    "y_min": -0.5, "y_max": 0.5,
    "z_min": 0.1,  "z_max": 1.0  # Ensure it stays above ground
}
class Robot:
    def __init__(self, robotConfigPath):
        with open(robotConfigPath,'r') as file:
            config = yaml.safe_load(file)
        self.dh_matrix = np.array(config["dh_matrix"], dtype=float)
        self.joint_limits = np.array(config["joint_limits"], dtype=float)
        self.xml_file = config["xml_file"]
        self.name = config["robot_name"]
        self.sim_env = MujocoEnvironment(self.xml_file)

    def get_eef_pose(self,jointAng, method="mujoco"):
        '''
        Gets end-effector pose for the given jointAng
        Inputs: eef_pose, and method='mujoco' for MuJoCo API else, internal fn
        '''
        if method=="mujoco":
            self.sim_env.set_joint_angles(joint_angles=jointAng)
            return self.sim_env.get_eef_pos()
        else:
            T_B_E = self.__forward_kinematics(jointAng=jointAng)
            T = T_B_E
            eef_position = T[:3,3]
            eef_orientation = T[:3,:3]
            eef_rpy = R.from_matrix(eef_orientation).as_euler('xyz', degrees=False)
            return np.hstack((eef_position,eef_rpy))
 
    def get_jacobian(self, jointAng, method="mujoco"):
        '''
        Gets jacobian for the end-effector given jointAng configuration
        Inputs: 'jointAng' current joint_angles, method="mujoco" for MuJoCo API else, internal fn 
        '''
        if method=="mujoco":
            return self.sim_env.get_jacobian(jointAng=jointAng)
        else:
            assert len(jointAng)==len(self.dh_matrix)
            z_axes, positions = self.__forward_kinematics(jointAng=jointAng, forJacobian=True)
            # End-effector position
            p_end = positions[-1]
            J_v = np.zeros((3, len(jointAng)))
            J_omega = np.zeros((3, len(jointAng)))

            for i in range(len(jointAng)):
                J_v[:, i] = np.cross(z_axes[i], p_end - positions[i])
                J_omega[:, i] = z_axes[i]
            return np.vstack((J_v, J_omega))


    def bounding_box_constraint(self,jointAng):
        """ Ensure EEF stays inside the bounding box. """
        pos = self.get_eef_pose(jointAng, return_rot=True)  # Get EEF position
        pos = pos[:3]
        # Check if position is inside the workspace
        x_ok = workspace_limits["x_min"] <= pos[0] <= workspace_limits["x_max"]
        y_ok = workspace_limits["y_min"] <= pos[1] <= workspace_limits["y_max"]
        z_ok = workspace_limits["z_min"] <= pos[2] <= workspace_limits["z_max"]

        return float(x_ok and y_ok and z_ok) - 1  # Must be >= 0 to satisfy the constraint

    def do_inverse_kinematics(self, eefPose, currentJointAng, method='mujoco'):
        '''
        Update joint angles for the given end-effector pose
        Inputs: eef_pose, and method='mujoco' for MuJoCo API else, internal fn
        '''
        initial_guess = currentJointAng
        self.target_pose = eefPose
        constraints = [{"type": "ineq", "fun": self.bounding_box_constraint}]
        result = minimize(self.__ik_cost, initial_guess, method="L-BFGS-B", bounds=self.joint_limits, constraints=constraints)
        return result.x

    def is_colliding_with_obstacles(self, jointConfig):
        '''Will use MuJoCo collision detector to see if it happens for particular joint configuration'''
        return self.sim_env.initialize_and_check_collision(joint_angles=jointConfig)
    
    
    def __ik_cost(self,jointAng):
        pos = self.get_eef_pose(jointAng)
        cost = np.linalg.norm(pos - self.target_pose)
        if self.sim_env.check_collision(): 
            cost+=2.25
        return cost

    def __forward_kinematics(self, jointAng, forJacobian = False):
        '''Update end-effector pos based on joint angles'''
        assert len(jointAng)==len(self.dh_matrix)
        z_axes = []
        positions = [np.zeros(3)]
        T = np.eye(4)
        for i, (a,alpha,d,_) in enumerate(self.dh_matrix):
            theta = jointAng[i]
            T_i = self.__dh_transform(a,alpha,d,theta)
            T = T@T_i
            z_axes.append(T[:3,2])
            positions.append(T[:3,3])
        if not forJacobian:
            return T
        else:
            return z_axes,positions

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

    # Test robot script's functions
    test_joint_angles = [np.pi/2, -np.pi/4, np.pi/4, 0, np.pi/2, 0]
    print("Is Colliding: ", robot.is_colliding_with_obstacles(test_joint_angles))
    print("End-effector Pose: ", robot.get_eef_pose(test_joint_angles, method="internal"))
    print(robot.get_jacobian(test_joint_angles, method="internal"))