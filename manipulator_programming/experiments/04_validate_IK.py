import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
from models.analytical.franka_arm import FrankaArm
from src.robot_model import RobotModel

def main():
    franka_arm = FrankaArm()
    rbt_model = RobotModel(franka_arm)
    q_init = np.array([0.2, -np.pi/4, 0.1, -np.pi/4, 0.1, np.pi/2, -np.pi/3])
    q = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
    tee_analytical = rbt_model.FK(q)
    q_solution = rbt_model.InverseKinematics(tee_analytical, q_init, max_iterations=100, epsilon=1e-4)
    T_solution = rbt_model.FK(q_solution)
    
    print("End-Effector Angles from FK:\n", np.round(Rotation.from_matrix(tee_analytical[:3, :3]).as_euler('xyz', degrees=True), 4))
    print("End-Effector Position from FK:\n", np.round(tee_analytical[:3, 3],4))
    print("End-Effector Angles from IK solution:\n", np.round(Rotation.from_matrix(T_solution[:3, :3]).as_euler('xyz', degrees=True), 4))
    print("End-Effector Position from IK solution:\n", np.round(T_solution[:3, 3], 4))

if __name__ == "__main__":
    main()
