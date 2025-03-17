import mujoco
import mujoco._functions
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import time

class MujocoEnvironment:
    def __init__(self, xml_file):
        self.xml_file = xml_file
        self.model = mujoco.MjModel.from_xml_path(self.xml_file)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer

    def set_joint_angles(self, joint_angles):
        """Set joint angles and update simulation state"""        
        for i, angle in enumerate(joint_angles):
            if self.model.jnt_range.shape[0] > 0:  
                min_limit, max_limit = self.model.jnt_range[i]
                self.data.qpos[i] = np.clip(angle, min_limit, max_limit)
            else:
                self.data.qpos[i] = angle 
        mujoco.mj_forward(self.model, self.data) 
    
    def check_collision(self):
        """Check for collisions in the current configuration"""
        mujoco.mj_forward(self.model, self.data)
        return any(self.data.contact[i].dist < 0 for i in range(self.data.ncon))
    
    def initialize_and_check_collision(self, joint_angles):
        """Set joint angles and check if there is a collision"""
        self.set_joint_angles(joint_angles)
        collision = self.check_collision()
        return collision
    
    def get_joint_info(self):
        joint_info = [self.model.joint(i) for i in range(self.model.njnt)]
        return joint_info
    
    def get_eef_pos(self,site="end_effector"):
        '''get end_effector site (b/w finger tips) pose-x,y,z,euler_ang'''
        mujoco.mj_forward(self.model, self.data)
        eef = self.model.site(name=site)
        ee_pos = self.data.site_xpos[eef.id].copy()
        ee_orient = self.data.site_xmat[eef.id].copy()   
        ee_euler = R.from_matrix(ee_orient.reshape(3,3)).as_euler('xyz', degrees=False)
        pos = np.hstack((ee_pos, ee_euler)).flatten()
        return pos

    def get_link_pos(self, link = "gripper"):
        '''input=body_name, get link pose - x,y,z,euler angles'''
        mujoco.mj_forward(self.model, self.data)
        link = self.model.body(name=link)
        link_pos = self.data.xpos[link.id].copy()
        link_orient = self.data.xmat[link.id].copy()
        link_euler = R.from_matrix(link_orient.reshape(3,3)).as_euler('xyz', degrees=False)
        pos = np.hstack((link_pos, link_euler)).flatten()
        return pos

    def get_gripper_position(self):
        '''Get gripper left and right tips locations'''
        return self.data.qpos[6:8]
    
    def get_jacobian(self, jointAng):
        self.set_joint_angles(joint_angles=jointAng)
        jacp = np.zeros((3, self.model.nv))  
        jacr = np.zeros((3, self.model.nv))  
        eef = self.model.body(name="tips")
        eef_pos = self.data.xpos[eef.id].copy()
        mujoco.mj_jac(self.model,self.data,jacp, jacr, eef_pos, eef.id)
        J = np.vstack((jacp,jacr))
        J = J[:, :6]
        return J

    def launch_viewer(self):
        """Launch MuJoCo viewer to visualize the robot"""
        with self.viewer.launch_passive(self.model, self.data):
            while self.viewer.is_running():
                self.viewer.sync()
    
    def run_simulation(self, joint_trajectory, time_step=1, loop = True):
        """Run simulation with a sequence of joint configurations over time"""
        with self.viewer.launch_passive(self.model, self.data) as viewer:
            while loop:
                for joint_angles in joint_trajectory:
                    if not viewer.is_running():  # Check if the viewer was closed
                        print("Viewer closed. Exiting simulation.")
                        return
                    self.set_joint_angles(joint_angles)
                    mujoco.mj_step(self.model, self.data)
                    viewer.sync()
                    mujoco.mj_step(self.model, self.data)
                    time.sleep(time_step)

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    from robot import Robot
    robot = Robot("ur5_config.yaml")
    sim_env = MujocoEnvironment(xml_file=robot.xml_file)

    # Example joint trajectory to test
    start_state = np.array([-1.5708,-1.5708, 1.5708, -1.5708, -1.5708,0])
    goal_state = np.array([-1.05, -1.2, 1.8, -2.5, -1.57, 0])
    joint_trajectory = [start_state, goal_state]

    # # test collision detection fn
    # sim_env.initialize_and_check_collision(start_state)

    # # Test getting end effector pose fn from robot model and mujoco
    # print(sim_env.get_link_pos(link="tips"))
    # print(robot.get_eef_pose(jointAng=start_state))

    # # Test getting end effector Jacobian fn from robot model and mujoco
    # print(sim_env.get_jacobian(start_state))
    # print(robot.get_jacobian(start_state))

    # Test simulation running
    sim_env.run_simulation(joint_trajectory)