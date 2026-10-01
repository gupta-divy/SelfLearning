import mujoco
import mujoco.viewer
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from models.analytical.franka_arm import FrankaArm
from src.paths import mujoco_model_path
from src.robot_model import RobotModel


def main():
    franka_arm = FrankaArm()
    rbt_model = RobotModel(franka_arm)

    model_xml = str(mujoco_model_path("franka_emika_panda", "scene.xml"))
    model = mujoco.MjModel.from_xml_path(model_xml)
    data = mujoco.MjData(model)

    # q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/2])
    q = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
    dq = np.array([0.1, 0.2, 0.1, 0.05, 0.1, 0.15, 0.05])

    data.qpos[:7] = q
    mujoco.mj_forward(model, data)
    jacp = np.zeros((3, 9))
    jacr = np.zeros((3, 9))
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    jacobian_mujoco = np.vstack((jacr[:, :7], jacp[:, :7]))
    print("End-Effector Jacobian from Mujoco:\n", jacobian_mujoco)

    jacobian_analytical = rbt_model.RobotJacobian(q)
    print("End-Effector Jacobian from Analytical:\n", jacobian_analytical)

    joint_velocities_analytical = jacobian_analytical @ dq
    print("Joint Velocities from Analytical Jacobian:\n", joint_velocities_analytical)
    joint_velocities_mujoco = jacobian_mujoco @ dq
    print("Joint Velocities from Mujoco Jacobian:\n", joint_velocities_mujoco)

    diff = np.linalg.norm(joint_velocities_analytical - joint_velocities_mujoco)
    print(f"Difference between analytical and Mujoco joint velocities: {diff:.6f}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while True:
            # mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()