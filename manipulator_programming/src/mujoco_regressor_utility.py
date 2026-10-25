"""Convert MuJoCo inertias to the regressor parameter convention."""
import mujoco
import numpy as np


def inertial_parameters(model, data, link_ids):
    """Extract [m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz] per link.
    MuJoCo stores principal COM inertia. Rotate into the analytical link frame
    and apply the parallel-axis theorem to obtain inertia about its origin.
    Assign each body to its closest arm-link ancestor, including the gripper.
    """
    mujoco.mj_forward(model, data)
    blocks = np.zeros((len(link_ids), 10))
    owners = {body_id: i for i, body_id in enumerate(link_ids)}
    for body in range(1, model.nbody):
        ancestor = body
        while ancestor and ancestor not in owners:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor == 0:
            continue  # Fixed base contributes no arm joint torque.
        i = owners[ancestor]
        R = data.xmat[ancestor].reshape(3, 3)
        c = R.T @ (data.xipos[body] - data.xpos[ancestor])
        R_inertia = R.T @ data.ximat[body].reshape(3, 3)
        mass = model.body_mass[body]
        inertia = R_inertia @ np.diag(model.body_inertia[body]) @ R_inertia.T
        inertia += mass * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
        blocks[i] += np.r_[mass, mass * c, np.diag(inertia),
                           inertia[0, 1], inertia[0, 2], inertia[1, 2]]
    return blocks.ravel()

