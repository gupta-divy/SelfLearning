import mujoco
import mujoco.viewer
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.paths import mujoco_model_path


def main():
    model_xml = str(mujoco_model_path("franka_emika_panda", "scene.xml"))
    model = mujoco.MjModel.from_xml_path(model_xml)
    data = mujoco.MjData(model)


    print("Number of generalized coordinates:", model.nq)
    print("Number of generalized velocities:", model.nv)
    print("Number of actuators:", model.nu)
    print("Number of joints:", model.njnt)

    print("\nInitial joint positions:", data.qpos)
    print("Initial joint velocities:", data.qvel)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while True:
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
