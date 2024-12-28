import numpy as np

# Robot Parameters
m = [1.0, 1.0]                # Masses of the links [m1, m2] (kg)
l = [1.0, 1.0]                # Lengths of the links [l1, l2] (m)
lc = [l[0] / 2, l[1] / 2]     # Center of mass positions [lc1, lc2] (m)
I = [0.1, 0.1]                # Inertia of the links [I1, I2] (kg.m^2)
g = 9.81                      # Gravitational acceleration (m/s^2)

def forward_recursion(i, q, qd, parent_omega, parent_omegadot, parent_accel):
    """
    Recursive forward computation of velocities and accelerations.
    
    Args:
        i: Current link index (0-based)
        q: Joint angles [q1, q2, ...] (rad)
        qd: Joint velocities [qd1, qd2, ...] (rad/s)
        parent_omega: Angular velocity of the parent link (array)
        parent_omegadot: Angular acceleration of the parent link (array)
        parent_accel: Linear acceleration of the parent link (array)
    
    Returns:
        omega: Angular velocity of the current link
        omegadot: Angular acceleration of the current link
        accel: Linear acceleration of the current link's center of mass
    """
    # Compute angular velocity and acceleration
    omega = parent_omega + np.array([0, 0, qd[i]])
    omegadot = parent_omegadot + np.array([0, 0, q[i]])

    # Position vector from parent to current center of mass
    r = np.array([lc[i] * np.cos(q[i]), lc[i] * np.sin(q[i]), 0.0])

    # Compute linear acceleration of the center of mass
    accel = parent_accel + np.cross(omegadot, r) + np.cross(omega, np.cross(omega, r))
    
    return omega, omegadot, accel

def backward_recursion(i, q, omega, omegadot, accel, child_force, child_torque):
    """
    Recursive backward computation of forces and torques.
    
    Args:
        i: Current link index (0-based, starting from the end-effector)
        q: Joint angles [q1, q2, ...] (rad)
        omega: Angular velocity of the current link
        omegadot: Angular acceleration of the current link
        accel: Linear acceleration of the current link's center of mass
        child_force: Force from the child link (array)
        child_torque: Torque from the child link (array)
    
    Returns:
        force: Force exerted by this link on the parent link
        torque: Torque exerted by this link on the parent joint
    """
    # Compute the force on the current link
    force = m[i] * accel + child_force

    # Position vector from the joint to the center of mass
    r = np.array([lc[i] * np.cos(q[i]), lc[i] * np.sin(q[i]), 0.0])

    # Compute the torque on the current link
    torque = np.cross(r, force) + I[i] * omegadot + child_torque

    return force, torque

def newton_euler_recursive(q, qd, tau):
    """
    Computes joint accelerations (forward dynamics) for an n-link manipulator
    using recursive Newton-Euler equations.
    
    Args:
        q: Joint angles [q1, q2, ...] (rad)
        qd: Joint velocities [qd1, qd2, ...] (rad/s)
        tau: Joint torques [tau1, tau2, ...] (Nm)
    
    Returns:
        qdd: Joint accelerations [qdd1, qdd2, ...] (rad/s^2)
    """
    n = len(q)
    
    # Forward recursion: Compute velocities and accelerations
    omega = np.zeros((n, 3))  # Angular velocity for each link
    omegadot = np.zeros((n, 3))  # Angular acceleration for each link
    accel = np.zeros((n, 3))  # Linear acceleration for each link
    
    parent_omega = np.array([0.0, 0.0, 0.0])
    parent_omegadot = np.array([0.0, 0.0, 0.0])
    parent_accel = np.array([0.0, 0.0, g])
    
    for i in range(n):
        omega[i], omegadot[i], accel[i] = forward_recursion(
            i, q, qd, parent_omega, parent_omegadot, parent_accel
        )
        parent_omega, parent_omegadot, parent_accel = omega[i], omegadot[i], accel[i]
    
    # Backward recursion: Compute forces and torques
    force = np.zeros((n, 3))  # Force for each link
    torque = np.zeros((n, 3))  # Torque for each link

    child_force = np.array([0.0, 0.0, 0.0])
    child_torque = np.array([0.0, 0.0, 0.0])
    
    for i in reversed(range(n)):
        force[i], torque[i] = backward_recursion(
            i, q, omega[i], omegadot[i], accel[i], child_force, child_torque
        )
        child_force, child_torque = force[i], torque[i]
    
    # Compute joint accelerations (qdd)
    qdd = np.zeros(n)
    for i in range(n):
        qdd[i] = (tau[i] - torque[i][2]) / I[i]
    
    return qdd

# Example usage
q = [np.pi / 4, np.pi / 4]    # Joint angles (rad)
qd = [0.0, 0.0]               # Joint velocities (rad/s)
tau = [5.0, 5.0]              # Joint torques (Nm)

# Forward dynamics: Compute joint accelerations
qdd = newton_euler_recursive(q, qd, tau)
print("Joint Accelerations (qdd):", qdd)
