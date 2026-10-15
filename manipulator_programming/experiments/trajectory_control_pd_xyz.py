import mujoco
import mujoco.viewer
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
from src.paths import mujoco_model_path
from src.trajectory_generator import TrajectoryGeneratorXYZ

## Tunable PD gains
Kp = np.diag([4500, 4500, 4500])
Kd = np.diag([120, 120, 120])


# Initialize Panda at home configuration
model_xml = str(mujoco_model_path("franka_emika_panda", "scene.xml"))
model = mujoco.MjModel.from_xml_path(model_xml)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,"home"))
mujoco.mj_forward(model, data)
ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
nv_arm = 7


# Trajectory Generator
x_init = data.site_xpos[ee_site_id].copy()
traj_gen = TrajectoryGeneratorXYZ(x_init=x_init, x_final=[0.5, 0.3, 0.15], T=5.0, dt=model.opt.timestep)
x_desired, v_desired = traj_gen.getdesired_trajectory()

# Logging Variables
jacp = np.zeros((3, 9))
jacr = np.zeros((3, 9))
error_pos_log = np.zeros((len(x_desired), 3))
error_vel_log = np.zeros((len(x_desired), 3))
tau_cmd_log = np.zeros((len(x_desired), nv_arm))
tau_actuator_log = np.zeros((len(x_desired), nv_arm))
time_log = np.arange(0, len(x_desired) * model.opt.timestep, model.opt.timestep)

with mujoco.viewer.launch_passive(model, data) as viewer:
    for i in range(len(x_desired)):
        x_actual = data.site_xpos[ee_site_id].copy()
        qdot = data.qvel[:nv_arm].copy()
        tau_bias = data.qfrc_bias[:nv_arm].copy()

        mujoco.mj_jacSite(model, data, jacp, jacr, ee_site_id)
        J = jacp[:, :nv_arm]
        v_actual = J @ qdot

        # Cartesian tracking errors
        eq = x_desired[i] - x_actual
        ev = v_desired[i] - v_actual

        F_cmd = Kp @ eq + Kd @ ev
        tau_pd = J.T @ F_cmd

        tau_cmd = tau_pd + tau_bias
        data.ctrl[:nv_arm] = tau_cmd

        # Log pre-step state
        error_pos_log[i] = eq.copy()
        error_vel_log[i] = ev.copy()
        tau_cmd_log[i] = tau_cmd.copy()

        # Advance simulation
        mujoco.mj_step(model, data)

        # Log actual actuator output
        tau_actuator_log[i] = data.actuator_force[:nv_arm].copy()

        viewer.sync()
        time.sleep(model.opt.timestep)


# Plotting
fig = plt.figure(figsize=(14, 7), layout="constrained")
gs = fig.add_gridspec(2, 2, width_ratios=[1.3, 1])

ax_tau = fig.add_subplot(gs[0, 0])
ax_tau_actuator = fig.add_subplot(gs[1, 0], sharex=ax_tau)
ax_pos = fig.add_subplot(gs[0, 1])
ax_vel = fig.add_subplot(gs[1, 1], sharex=ax_pos)

# PD Commanded torques
for j in range(tau_cmd_log.shape[1]):
    ax_tau.plot(time_log, tau_cmd_log[:, j], label=f"Joint {j+1}")

ax_tau.set_title("Joint Torques")
ax_tau.set_xlabel("Time (s)")
ax_tau.set_ylabel("Torque (Nm)")
ax_tau.legend()
ax_tau.grid(True)

# Joint actuator torques
for j in range(tau_actuator_log.shape[1]):
    ax_tau_actuator.plot(time_log, tau_actuator_log[:, j], label=f"Joint {j+1}")
ax_tau_actuator.set_title("Joint Actuator Torques")
ax_tau_actuator.set_xlabel("Time (s)")
ax_tau_actuator.set_ylabel("Torque (Nm)")
ax_tau_actuator.legend()
ax_tau_actuator.grid(True)


# Cartesian position error
for j, axis in enumerate(["X", "Y", "Z"]):
    ax_pos.plot(time_log, error_pos_log[:, j], label=axis)

ax_pos.set_title("Cartesian Position Error")
ax_pos.set_ylabel("Position Error (m)")
ax_pos.legend()
ax_pos.grid(True)

# Cartesian velocity error
for j, axis in enumerate(["X", "Y", "Z"]):
    ax_vel.plot(time_log, error_vel_log[:, j], label=axis)

ax_vel.set_title("Cartesian Velocity Error")
ax_vel.set_xlabel("Time (s)")
ax_vel.set_ylabel("Velocity Error (m/s)")
ax_vel.legend()
ax_vel.grid(True)

plt.show()







