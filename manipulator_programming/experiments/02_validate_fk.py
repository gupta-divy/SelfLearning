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

    q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/2])

    tee_analytical = rbt_model.FK(q)
    print("End-Effector Pose from FK:\n", tee_analytical)

    data.qpos[:7] = q
    mujoco.mj_forward(model, data)
    tee_mujoco = np.eye(4)
    tee_mujoco[:3, :3] = data.site_xmat[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")].reshape(3, 3)
    tee_mujoco[:3, 3] = data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")]
    print("End-Effector Position from Mujoco:\n", tee_mujoco)


    with mujoco.viewer.launch_passive(model, data) as viewer:
        while True:
            # mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
