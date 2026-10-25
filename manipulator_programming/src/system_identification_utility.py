from pathlib import Path
import sys

import numpy as np
from scipy.linalg import qr, solve_triangular

# Support running this file directly as well as importing it as a package.
if __package__ in (None, ""):
    project_root = str(Path(__file__).resolve().parents[1])
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

from src.robot_model import RobotModel

def extract_base_regressor(W, rank):
    """Return independent column indices and the full-to-base parameter map.

    W_base = W[:, indices]; W = W_base @ parameter_map;
    theta_base = parameter_map @ theta. Validate this numerical relation
    on independently sampled states before using it for identification.
    """
    W = np.asarray(W, dtype=float)
    if W.ndim != 2 or not np.all(np.isfinite(W)):
        raise ValueError("W must be a finite 2D matrix")
    if not isinstance(rank, (int, np.integer)) or not 0 < rank <= min(W.shape):
        raise ValueError("rank must be an integer between 1 and min(W.shape)")

    _, R, pivots = qr(W, mode="economic", pivoting=True)
    indices = pivots[:rank]
    coefficients = solve_triangular(R[:rank, :rank], R[:rank, rank:])
    parameter_map = np.zeros((rank, W.shape[1]))
    parameter_map[:, indices] = np.eye(rank)
    parameter_map[:, pivots[rank:]] = coefficients
    return indices, parameter_map


def build_inertial_regressor(robot: RobotModel, q, qd, qdd, gravity = 1):
    """
        Build the full rigid-body inertial regressor.
        gravity = 1 (gravity on) 0 (gravity off)

        Returns
        -------
        Y : shape (robot.n, 10 * robot.n)

        Parameter blocks follow link order:
            [link1's 10 parameters, link2's 10 parameters, ...]
            Each block: [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
            h = m * COM; inertia is about the link origin, expressed
            in the link body frame (not inertia about the COM).
            Joint torque prediction: tau = Y @ pi.

        Excludes friction, motor inertia and external forces.
        """

    if gravity==1:
        g0 = np.asarray([0.0, 0.0, -9.81])
    else:
        g0 = np.asarray([0.0, 0.0, 0.0])

    Y = np.zeros((robot.n, 10 * robot.n))
    basis = np.eye(10)     # basis vector to activate one inertial parameter at a time

    link_kinematics = robot.get_link_kinematics(q,qd,qdd)
    for i in range(robot.n):
        R = link_kinematics.poses[i,:3,:3]
        # Motion in link frame
        omega = R.T @ link_kinematics.omega[i]
        alpha = R.T @ link_kinematics.alpha[i]
        a = R.T @ (link_kinematics.acceleration[i] - g0)

        # Jacobians in base frame
        Jw = link_kinematics.jacobians[i][:3,:]
        Jv = link_kinematics.jacobians[i][3:,:]

        for k in range(10):
            # effect of only pi[k] component on torque
            pi_unit = basis[k]

            # force moment in link origin frame
            force, moment = robot.get_link_wrench(pi_unit, omega, alpha, a)


            tau = (Jv.T @ (R @ force) +
                   Jw.T @ (R @ moment))

            Y[:, 10*i + k] = tau
    
    return Y

def sample_state(robot: RobotModel, max_vel=1.0, max_acc=2.0, rng = None):
    n = robot.n
    if rng is None:
        rng = np.random.default_rng()

    vel_limits = _joint_limits(max_vel, "max_vel", n)
    acc_limits = _joint_limits(max_acc, "max_acc", n)

    q = rng.uniform(
        robot.joint_limits["lower"],
        robot.joint_limits["upper"],
    )
    qd = rng.uniform(-vel_limits, vel_limits)
    qdd = rng.uniform(-acc_limits, acc_limits)

    return q, qd, qdd

def _joint_limits(value, name, n):
        limits = np.asarray(value, dtype=float)

        if limits.ndim == 0:
            limits = np.full(n, limits.item())
        elif limits.shape != (n,):
            raise ValueError(
                f"{name} must be a scalar or an array of shape ({n},)"
            )

        if not np.all(np.isfinite(limits)):
            raise ValueError(f"{name} must contain finite values")

        return np.abs(limits)
