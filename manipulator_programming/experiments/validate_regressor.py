import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from models.analytical.franka_arm import FrankaArm
from src.robot_model import RobotModel
from src.paths import mujoco_model_path
from src.system_identification_utility import build_inertial_regressor
from src.mujoco_regressor_utility import inertial_parameters


if __name__ == "__main__":
    robot = RobotModel(FrankaArm())
    model = mujoco.MjModel.from_xml_path(str(mujoco_model_path("franka_emika_panda", "panda.xml")))

    # these are disabled just to make sure we are comparing same values, not have any friction, damping, or gear ratios at the joint. 
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    data = mujoco.MjData(model)
    joints = [model.joint(f"joint{i+1}").id for i in range(robot.n)]
    dofs = model.jnt_dofadr[joints]
    links = [model.body(f"link{i+1}").id for i in range(robot.n)]
    pi = inertial_parameters(model, data, links)

    q = np.array([0, -0.4, 0.2, -1.5, 0.1, 1.2, -0.3])
    qd = np.full(robot.n, 0.2)
    qdd = np.full(robot.n, 0.5)
    Y = build_inertial_regressor(robot, q, qd, qdd, gravity=1)
    print(Y.shape)
    data.qpos[model.jnt_qposadr[joints]] = q
    data.qvel[:] = 0
    data.qacc[:] = 0 
    data.qvel[dofs] = qd
    data.qacc[dofs] = qdd
    mujoco.mj_inverse(model, data)
    tau = data.qfrc_inverse[dofs]
    print("Regressor:", Y @ pi)
    print("MuJoCo:   ", tau)
    np.testing.assert_allclose(Y @ pi, tau, atol=1e-8, rtol=1e-9)
    print("Validation passed")
