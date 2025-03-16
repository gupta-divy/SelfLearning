import mujoco
import mujoco.viewer
import numpy as np
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
    
    # Example joint angles to test collision detection
    test_joint_angles = [0, -np.pi/4, np.pi/4, 0, np.pi/2, 0]
    sim_env.initialize_and_check_collision(test_joint_angles)
    
    # Example joint trajectory to test
    joint_trajectory = [
        [0, -np.pi/4, np.pi/4, 0, np.pi/2, 0],
        [np.pi/6, -np.pi/6, np.pi/3, 0, np.pi/4, -np.pi/4],
        [np.pi/3, -np.pi/3, np.pi/6, np.pi/6, np.pi/3, -np.pi/6]
    ]
    
    sim_env.run_simulation(joint_trajectory, time_step=2)