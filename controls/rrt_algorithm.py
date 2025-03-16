import random
import numpy as np

class Robot:
    def __init__(self, dhMatrix, jointLimits):
        self.dh_matrix = dhMatrix
        self.joint_limits = jointLimits

class RobotState(Robot):
    def __init__(self, state, mode='c-space'):
        self.joint_ang = None if mode != "c-space" else np.array(state)
        self.eef_pos = None if mode == "c-space" else np.array(state)
    
    def get_eef_pose(self):
        self.eef_pos = self.__forward_kinematics() if self.eef_pos is None else self.eef_pos
        return self.eef_pos

    def get_joint_ang(self):
        self.joint_ang = self.__inverse_kinematics() if self.joint_ang is None else self.joint_ang
        return self.joint_ang

    def in_collision(self):
        return False

    def __forward_kinematics(self):
        '''Update end-effector pos based on joint angles'''
        pass

    def __inverse_kinematics(self):
        '''Update joint angles for the given end-effector pose'''
        pass


class Node:
    '''Node is defined as a tree node in C-space'''
    def __init__(self, robot_state: RobotState, mode='c-space', parent: 'Node'= None):
        self.state: np.ndarray = robot_state.get_joint_ang() if mode=='c-space' else robot_state.get_eef_pose()
        self.children: list = []
        self.parent: 'Node' = parent
        self.is_valid: bool = not robot_state.in_collision()

class RRTAlgorithm:
    def __init__(self, start: RobotState, goal: RobotState, tolerance: np.ndarray, samplingSpace: np.ndarray, stepSize: float = 0.1, moveStepSize: float = 0.01, goalBias: float = 0.2, mode: str='c-space', numIterations: int = 500):
        self.root = Node(robot_state=start, mode=mode)
        self.goal = Node(robot_state=goal, mode=mode)
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
        self.solved = False
    
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
                    print("Goal Found")
                    self.solved=True
                    break
        self.retrace_path(atNode=self.goal)
    
    def get_path(self):
        if not self.solved:
            self.solve()
        path = [point.state for point in self.waypoints]
        return np.vstack(path)

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
        if np.all(atNode.state == self.root.state):
            self.waypoints.insert(0,atNode)
            return
        else:
            self.waypoints.insert(0,atNode)
            self.num_waypoints+=1
            self.path_distance += self.step_size
            self.retrace_path(atNode=atNode.parent)