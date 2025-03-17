import random
import numpy as np
from robot import Robot
import os
from scipy.interpolate import interp1d

class RobotState():
    robot = None  # Class variable to store a single Robot instance
    
    @classmethod
    def set_robot(cls, robot_instance: Robot):
        """Sets the shared robot instance for all RobotState objects."""
        cls.robot = robot_instance

    def __init__(self, state, mode='c-space'):
        if RobotState.robot==None:
            raise ValueError("Robot instance not set. Call RobotState.set_robot(robot) first.")
        
        self.joint_ang = None if mode != "c-space" else np.array(state)
        self.eef_pos = None if mode == "c-space" else np.array(state)
    
    def get_eef_pose(self):
        self.eef_pos = RobotState.robot.get_eef_pose(self.joint_ang) if self.eef_pos is None else self.eef_pos
        return self.eef_pos

    def get_joint_ang(self):
        self.joint_ang = RobotState.robot.do_inverse_kinematics(self.eef_pos) if self.joint_ang is None else self.joint_ang
        return self.joint_ang

    def in_collision(self):
        return RobotState.robot.is_colliding_with_obstacles(jointConfig=self.get_joint_ang())

class Node:
    '''Node is defined as a tree node in C-space'''
    def __init__(self, robot_state: RobotState, mode='c-space', parent: 'Node'= None):
        self.state: np.ndarray = robot_state.get_joint_ang() if mode=='c-space' else robot_state.get_eef_pose()
        self.children: list = []
        self.parent: 'Node' = parent
        self.is_valid: bool = not robot_state.in_collision()

class RRTAlgorithm:
    def __init__(self, start: RobotState, goal: RobotState, tolerance: np.ndarray, samplingSpace: np.ndarray, stepSize: float = 0.1, moveStepSize: float = 0.01, goalBias: float = 0.2, mode: str='c-space', numIterations: int = 500):
        init_check = True
        self.root = Node(robot_state=start, mode=mode)
        self.goal = Node(robot_state=goal, mode=mode)

        if(not self.root.is_valid): 
            init_check = False
            print("Invalid Start, Obstacle in Contact")
        if(not self.goal.is_valid): 
            init_check = False
            print("Invalid Goal, Obstacle in Contact")
        if not init_check: raise ValueError("Invalid Start / Goal")

        self.mode = mode
        self.nearest_node = None
        self.iterations = numIterations
        self.sampling_space = samplingSpace
        self.step_size = stepSize
        self.move_step_size = moveStepSize
        self.goal_bias = goalBias
        self.goal_tolerance = tolerance
        self.path_distance = 0
        self.nearest_distance = float('inf')
        self.num_waypoints = 0
        self.waypoints = []
        self.goal_found = False
    
    def solve(self):
        for i in range(self.iterations):
            # print("Iteration: ", i, " of ", self.iterations)
            self.reset_nearest_values()
            sample_pt = self.sample_point()
            self.find_nearest(self.root,vectorx=sample_pt)
            new_pt = self.steer_to_sampled_point(self.nearest_node.state, sample_pt)
            if not self.through_obstacle(self.nearest_node.state,new_pt):
                self.add_child(new_pt)
                if self.is_goal(new_pt):
                    self.add_child(self.goal.state)
                    print("Goal Found at iteration: ", i)
                    break
        if self.goal_found: self.retrace_path(atNode=self.goal)
    
    def get_path(self):
        if not self.goal_found:
            if len(self.waypoints)==0: self.solve()
        if self.goal_found:
            path = [point.state for point in self.waypoints]
            return np.vstack(path)
        else:
            print("No Path found in these iterations")
            return None

    def add_child(self, vectorx):
        if self.is_goal(vectorx):
            self.nearest_node.children.append(self.goal)
            self.goal.parent = self.nearest_node
        else:
            robot_state = RobotState(state=vectorx,mode=self.mode)
            self.nearest_node.children.append(Node(robot_state=robot_state,mode=self.mode,parent=self.nearest_node))

    def sample_point(self):
        if random.random()<self.goal_bias:
            return self.goal.state
        else:
            sampled_vector = np.array([random.uniform(self.sampling_space[i][0],self.sampling_space[i][1]) for i in range(len(self.sampling_space))])
            return sampled_vector

    def steer_to_sampled_point(self, vectorx0, vectorx1):
        offset = self.step_size*self.get_unit_vector(vectorx0,vectorx1)
        new_point = np.clip(np.add(vectorx0, offset), a_min=self.sampling_space[:,0], a_max=self.sampling_space[:,1])
        return new_point

    def get_unit_vector(self, vectorx0, vectorx1):
        temp_v = vectorx1-vectorx0
        norm_v = np.linalg.norm(temp_v, ord=2)
        unit_vector =  temp_v/norm_v if norm_v!=0 else np.zeros_like(temp_v)
        return unit_vector

    def through_obstacle(self, vectorx0, vectorx1):
        u_hat = self.move_step_size*self.get_unit_vector(vectorx0,vectorx1)
        checkpoints = int(self.step_size / self.move_step_size) + 1
        for i in range(1,checkpoints):
            vectorx = vectorx0 + i * u_hat
            robot_state_at_vecx = RobotState(state=vectorx, mode=self.mode)
            in_collision = robot_state_at_vecx.in_collision()
            if in_collision: return True
        return False

    def find_nearest(self, atNode: Node, vectorx):
        if atNode is None:
            return
        
        vectorx0 = atNode.state
        curr_dist = self.get_distance(vectorx0=vectorx0, vectorx1=vectorx)
        if curr_dist < self.nearest_distance: 
            self.nearest_distance = curr_dist
            self.nearest_node = atNode

        for child in atNode.children:            
            self.find_nearest(atNode=child, vectorx=vectorx)

    def get_distance(self, vectorx0, vectorx1):
        return np.linalg.norm(vectorx1-vectorx0,ord=2)
    
    def is_goal(self, vectorx):
        goal_vector = self.goal.state
        if np.all(np.abs(vectorx-goal_vector)<=self.goal_tolerance): 
            self.goal_found=True
            return True
        return False

    def reset_nearest_values(self):
        self.nearest_distance = float('Inf')
        self.nearest_node = None

    def retrace_path(self,atNode: Node):
        self.waypoints.insert(0,atNode)
        self.num_waypoints+=1
        self.path_distance += self.step_size
        if np.all(atNode.state == self.root.state):
            return
        else:
            self.retrace_path(atNode=atNode.parent)

def interpolate_trajectory(joint_trajectory, num_points=50):
    """Interpolates between given joint waypoints to create a smooth trajectory."""
    joint_trajectory = np.array(joint_trajectory)
    num_waypoints, num_joints = joint_trajectory.shape
    
    # Generate a time vector for the given waypoints
    time_waypoints = np.linspace(0, 1, num_waypoints)
    time_interp = np.linspace(0, 1, num_points)

    # Interpolated trajectory storage
    interpolated_trajectory = np.zeros((num_points, num_joints))

    for j in range(num_joints):
        # Create interpolation function for each joint
        interp_func = interp1d(time_waypoints, joint_trajectory[:, j], kind='cubic')
        interpolated_trajectory[:, j] = interp_func(time_interp)

    return interpolated_trajectory


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    ur5_robot = Robot(robotConfigPath="ur5_config.yaml")
    RobotState.set_robot(ur5_robot)
    ur5_robot.sim_env.set_joint_angles(joint_angles= [-1.5708,-1.5708, 1.5708, -1.5708, -1.5708,0,0,0])
    # Define start and goal joint angles within limits
    start_state = np.array([-1.5708,-1.5708, 1.5708, -1.5708, -1.5708,0])
    # goal_state = np.array([np.pi/3, -np.pi/4, -np.pi/8, -np.pi/6, -np.pi/3, np.pi/4])
    goal_tolerance = np.array([0.1] * 6)
    target_obj_pose = ur5_robot.sim_env.get_link_pos(link="box")
    target_pos = np.hstack((target_obj_pose[:3], [np.pi, 0, 0]))
    target_pos = target_pos.reshape(1, -1) 
    goal_state = ur5_robot.do_inverse_kinematics(eefPose=target_pos, currentJointAng=[-1.05, -1.2, 1.8, -2.5, -1.57, 0])
    print("Goal State: ", goal_state)

    # Initialize RRT algorithm
    start_robot = RobotState(state=start_state, mode="c-space")
    goal_robot = RobotState(state=goal_state, mode="c-space")
    rrt = RRTAlgorithm(
        start=start_robot, 
        goal=goal_robot, 
        tolerance=goal_tolerance, 
        samplingSpace=ur5_robot.joint_limits,
        stepSize=0.1,
        moveStepSize=0.01,
        goalBias=0.05,
        numIterations=10000
    )

    path = rrt.get_path()
    if path is None: exit()
    else: print("Path found with", rrt.num_waypoints, "waypoints. Now Simulating: ")
    
    gripper_tips = np.array([[0, 0]])
    gripper_trajectory = np.repeat(gripper_tips, rrt.num_waypoints, axis=0)
    gripper_trajectory[-(path.shape[0] // 2):] = [1, 1]
    path = np.hstack((path,gripper_trajectory))
    
    trajectory = interpolate_trajectory(path,num_points=500)
    ur5_robot.sim_env.run_simulation(joint_trajectory=trajectory, time_step=0.02)
    