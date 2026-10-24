import mujoco
import mujoco.viewer
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
from src.paths import mujoco_model_path
from src.trajectory_generator import TrajectoryGeneratorSE3

## Tunable PD gains
Kp_p = np.diag([4500, 4500, 4500])
Kd_p = np.diag([120, 120, 120])
Kp_r = np.diag([50, 60, 50])
Kd_r = np.diag([4, 2, 2])


# Initialize Panda at home configuration
model_xml = str(mujoco_model_path("franka_emika_panda", "scene_dd.xml"))
model = mujoco.MjModel.from_xml_path(model_xml)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,"home"))
joint_id = model.joint("box_freejoint").id
adr = model.jnt_qposadr[joint_id]
data.qpos[adr:adr+7] = model.qpos0[adr:adr+7]
mujoco.mj_forward(model, data)
nv_arm = 7
jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))
ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box")

# Trajectory Generator
T_init = np.eye(4)
T_init[:3, :3] = data.site_xmat[ee_site_id].reshape(3, 3)
T_init[:3, 3] = data.site_xpos[ee_site_id]
T_preGrasp = T_init.copy()

# pre-grasp pose:
box_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"box")
p_box = data.xpos[box_id].copy()
r_box = data.xmat[box_id].reshape(3, 3)
# T_preGrasp[:3, :3] = r_box @ Rotation.from_euler('xyz', [0, 0, np.pi]).as_matrix()
T_preGrasp[:3, 3] = p_box + np.array([0.0, 0.0, 0.1])

# Grasp pose:
T_grasp = T_preGrasp.copy()
T_grasp[:3, 3] = p_box + np.array([0.0, 0.0, 0.05])

# Generate trajectory
traj_preGrasp = TrajectoryGeneratorSE3(
    x_init=T_init,
    x_final=T_preGrasp,
    T=5.0,
    dt=model.opt.timestep
)

traj_grasp = TrajectoryGeneratorSE3(
    x_init=T_preGrasp,
    x_final=T_grasp,
    T=2.0,
    dt=model.opt.timestep
)


T_desired_preGrasp, v_desired_preGrasp, omega_desired_preGrasp = traj_preGrasp.getdesired_trajectory()
T_desired_grasp, v_desired_grasp, omega_desired_grasp = traj_grasp.getdesired_trajectory()
T_desired = np.concatenate((T_desired_preGrasp, T_desired_grasp), axis=0)
v_desired = np.concatenate((v_desired_preGrasp, v_desired_grasp), axis=0)
omega_desired = np.concatenate((omega_desired_preGrasp, omega_desired_grasp), axis=0)

# Gripper Contact and Hold:
gripper_id = model.actuator("actuator8").id

# Keep fingers open during pre-grasp and approach.
gripper_desired = np.full(len(T_desired), 255.0)

# Hold the final arm pose while closing over 1 second,
# then maintain the grip for another second.
n_close = int(np.ceil(1.0 / model.opt.timestep))
n_hold = int(np.ceil(1.0 / model.opt.timestep))
n_extra = n_close + n_hold
T_desired = np.concatenate((T_desired, np.repeat(T_desired[-1:], n_extra, axis=0)))
v_desired = np.concatenate((v_desired, np.zeros((n_extra, 3))))
omega_desired = np.concatenate((omega_desired, np.zeros((n_extra, 3))))
gripper_desired = np.concatenate((gripper_desired,np.linspace(255.0, 0.0, n_close),np.zeros(n_hold)))

# Move to initial pose:
T_desired_lift,v_desired_lift,omega_desired_lift = TrajectoryGeneratorSE3(x_init=T_desired[-1], x_final=T_init, T=3.0, dt=model.opt.timestep).getdesired_trajectory()
gripper_desired_lift = np.zeros(len(T_desired_lift))

T_desired = np.concatenate((T_desired, T_desired_lift), axis=0)
v_desired = np.concatenate((v_desired, v_desired_lift), axis=0)
omega_desired = np.concatenate((omega_desired, omega_desired_lift), axis=0)
gripper_desired = np.concatenate((gripper_desired, gripper_desired_lift), axis=0)

T_desired = np.concatenate((T_desired, np.repeat(T_desired[-1:], n_hold, axis=0)))
v_desired = np.concatenate((v_desired, np.zeros((n_hold, 3))))
omega_desired = np.concatenate((omega_desired, np.zeros((n_hold, 3))))
gripper_desired = np.concatenate((gripper_desired,np.zeros(n_hold // 2),np.full(n_hold // 2, 255.0)))

# Logging Variables
tau_actuator_log = np.zeros((len(T_desired), nv_arm+1))
time_log = np.arange(0, len(T_desired) * model.opt.timestep, model.opt.timestep)

with mujoco.viewer.launch_passive(model, data) as viewer:
    for i in range(len(T_desired)):
        x_actual = data.site_xpos[ee_site_id].copy()
        qdot = data.qvel[:nv_arm].copy()
        tau_bias = data.qfrc_bias[:nv_arm].copy()

        mujoco.mj_jacSite(model, data, jacp, jacr, ee_site_id)
        Jv = jacp[:, :nv_arm]
        Jw = jacr[:, :nv_arm]
        v_actual = Jv @ qdot

        # Position tracking errors
        des_pos = T_desired[i, :3, 3]
        e_pos = des_pos - x_actual
        e_vel = v_desired[i] - v_actual

        # Orientation tracking errors
        R_desired = T_desired[i, :3, :3]
        R_actual = data.site_xmat[ee_site_id].reshape(3, 3)        
        rot_des = Rotation.from_matrix(R_desired).as_euler('xyz', degrees=True)

        R_error = R_desired @ R_actual.T
        e_rot = Rotation.from_matrix(R_error).as_rotvec()
        e_omega = omega_desired[i] - (Jw @ qdot)

        F_cmd = Kp_p @ e_pos + Kd_p @ e_vel
        M_cmd = Kp_r @ e_rot + Kd_r @ e_omega
        
        tau_pd = Jv.T @ F_cmd + Jw.T @ M_cmd


        tau_cmd = tau_pd + tau_bias
        data.ctrl[:nv_arm] = tau_cmd
        data.ctrl[gripper_id] = gripper_desired[i]

        # Advance simulation
        mujoco.mj_step(model, data)

        # Log actual actuator output
        tau_actuator_log[i] = data.actuator_force[:nv_arm+1].copy()

        viewer.sync()
        time.sleep(model.opt.timestep)


# Plotting
plt.plot(time_log, tau_actuator_log)
plt.xlabel("Time (s)")
plt.ylabel("Actuator Torque (Nm)")
plt.title("Joint Actuator Torques")
plt.grid(True)
plt.show()