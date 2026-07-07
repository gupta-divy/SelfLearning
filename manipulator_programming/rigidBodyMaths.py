import numpy as np
from math import cos, sin, acos, atan2, pi


# ============================================================
# Basic helpers
# ============================================================

def nearZero(x: float, eps: float = 1e-6) -> bool:
    """Return True if x is close to zero."""
    return abs(x) < eps


def normalize(v):
    """Return unit vector in the direction of v."""
    v = np.asarray(v, dtype=float).reshape(-1)
    n = np.linalg.norm(v)

    if nearZero(n):
        raise ValueError("Cannot normalize a zero vector.")

    return v / n


def sq(x: float):
    """Return square of x."""
    return x**2


# ============================================================
# so(3) / SO(3): angular velocity and rotation matrices
# ============================================================
def VecToso3(omg):
    """Modern Robotics name for skew()."""
    w = omg

    return np.array([
        [0.0,   -w[2],  w[1]],
        [w[2],   0.0, -w[0]],
        [-w[1],  w[0],  0.0]
    ])


def so3ToVec(so3mat):
    """Convert a 3x3 skew-symmetric matrix into a 3-vector."""
    so3mat = np.asarray(so3mat, dtype=float)

    return np.array([
        so3mat[2, 1],
        so3mat[0, 2],
        so3mat[1, 0]
    ])


def AxisAng3(expc3):
    """
    Convert a 3-vector of exponential coordinates into
    unit rotation axis omega_hat and angle theta.
    """
    expc3 = np.asarray(expc3, dtype=float).reshape(3)
    theta = np.linalg.norm(expc3)

    if nearZero(theta):
        return expc3, 0.0

    return expc3 / theta, theta


def RotInv(R):
    """Inverse of a rotation matrix."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    return R.T


def expRot(omega, theta: float):
    """
    Rotation matrix exponential using Rodrigues' formula.

    omega should be a unit axis. theta is in radians.
    Same idea as MatrixExp3(VecToso3(omega * theta)).
    """
    omega = normalize(omega)
    omgmat = VecToso3(omega)

    return (
        np.eye(3)
        + sin(theta) * omgmat
        + (1.0 - cos(theta)) * (omgmat @ omgmat)
    )

def MatrixLog3(R):
    """
    Matrix logarithm from SO(3) to so(3).
    Returns [omega] * theta.
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    acos_input = (np.trace(R) - 1.0) / 2.0
    acos_input = np.clip(acos_input, -1.0, 1.0)

    if acos_input >= 1.0:
        return np.zeros((3, 3))

    if acos_input <= -1.0:
        # theta = pi special case
        if not nearZero(1.0 + R[2, 2]):
            omega = (1.0 / np.sqrt(2.0 * (1.0 + R[2, 2]))) * np.array([
                R[0, 2],
                R[1, 2],
                1.0 + R[2, 2]
            ])
        elif not nearZero(1.0 + R[1, 1]):
            omega = (1.0 / np.sqrt(2.0 * (1.0 + R[1, 1]))) * np.array([
                R[0, 1],
                1.0 + R[1, 1],
                R[2, 1]
            ])
        else:
            omega = (1.0 / np.sqrt(2.0 * (1.0 + R[0, 0]))) * np.array([
                1.0 + R[0, 0],
                R[1, 0],
                R[2, 0]
            ])

        return VecToso3(pi * omega)

    theta = acos(acos_input)
    return theta / (2.0 * sin(theta)) * (R - R.T)


# ============================================================
# SE(3): homogeneous transformation matrices
# ============================================================

def RpToTrans(R, p):
    """Build homogeneous transform T from rotation R and position p."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    p = np.asarray(p, dtype=float).reshape(3)

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p

    return T


def TransToRp(T):
    """Split homogeneous transform T into rotation R and position p."""
    T = np.asarray(T, dtype=float).reshape(4, 4)
    R = T[0:3, 0:3]
    p = T[0:3, 3]

    return R, p


def TransInv(T):
    """Inverse of a homogeneous transform."""
    R, p = TransToRp(T)
    R_T = R.T

    return RpToTrans(R_T, -R_T @ p)


# ============================================================
# se(3), twists, adjoint
# ============================================================

def VecTose3(V):
    """
    Convert a spatial velocity vector V = [omega, v] into an se(3) matrix.
    """
    V = np.asarray(V, dtype=float).reshape(6)
    omega = V[0:3]
    v = V[3:6]

    se3mat = np.zeros((4, 4))
    se3mat[0:3, 0:3] = VecToso3(omega)
    se3mat[0:3, 3] = v

    return se3mat

def se3ToVec(se3mat):
    """
    Convert an se(3) matrix into a spatial velocity vector V = [omega, v].
    """
    se3mat = np.asarray(se3mat, dtype=float).reshape(4, 4)
    omega = so3ToVec(se3mat[0:3, 0:3])
    v = se3mat[0:3, 3]

    return np.r_[omega, v]


def Adjoint(T):
    """
    Calculate 6x6 adjoint representation [Ad_T].

    If V_b is a twist in frame b and T_ab is pose of b in a,
    then V_a = Adjoint(T_ab) @ V_b.
    """
    R, p = TransToRp(T)

    return np.r_[
        np.c_[R, np.zeros((3, 3))],
        np.c_[VecToso3(p) @ R, R]
    ]


def ad(V):
    """
    Lie bracket matrix ad_V for twists.

    For V = [omega, v],
    ad(V) = [[skew(omega), 0],
             [skew(v),     skew(omega)]]
    """
    V = np.asarray(V, dtype=float).reshape(6)
    omega = V[0:3]
    v = V[3:6]

    return np.r_[
        np.c_[VecToso3(omega), np.zeros((3, 3))],
        np.c_[VecToso3(v),     VecToso3(omega)]
    ]


def ScrewToAxis(q, s, h):
    """
    Build a screw axis from:
    q = any point on the screw axis,
    s = unit direction of screw axis,
    h = pitch.

    Returns S = [s, -s x q + h*s].
    """
    q = np.asarray(q, dtype=float).reshape(3)
    s = normalize(s)

    return np.r_[s, -np.cross(s, q) + h * s]


def AxisAng6(expc6):
    """
    Convert 6-vector exponential coordinates into screw axis S and theta.

    expc6 = S * theta
    """
    expc6 = np.asarray(expc6, dtype=float).reshape(6)
    theta = np.linalg.norm(expc6[0:3])

    if nearZero(theta):
        theta = np.linalg.norm(expc6[3:6])

    if nearZero(theta):
        return expc6, 0.0

    return expc6 / theta, theta


def expTwist(S, theta: float):
    """
    Homogeneous transform exponential e^[S]theta.

    S = [omega, v]. theta is in radians for revolute screws and linear
    displacement for pure prismatic screws.
    """
    S = np.asarray(S, dtype=float).reshape(6)
    return MatrixExp6(VecTose3(S * theta))


def MatrixExp6(se3mat):
    """
    Matrix exponential from se(3) to SE(3).

    Input is [S] * theta.
    """
    se3mat = np.asarray(se3mat, dtype=float).reshape(4, 4)
    omgtheta = so3ToVec(se3mat[0:3, 0:3])

    if nearZero(np.linalg.norm(omgtheta)):
        T = np.eye(4)
        T[0:3, 3] = se3mat[0:3, 3]
        return T

    omega, theta = AxisAng3(omgtheta)
    omgmat = se3mat[0:3, 0:3] / theta
    vtheta = se3mat[0:3, 3]

    R = expRot(omega,theta)

    G_theta = (
        np.eye(3) * theta
        + (1.0 - cos(theta)) * omgmat
        + (theta - sin(theta)) * (omgmat @ omgmat)
    )
    p = G_theta @ (vtheta / theta)

    return RpToTrans(R, p)


def MatrixLog6(T):
    """
    Matrix logarithm from SE(3) to se(3).

    Returns [S] * theta.
    """
    R, p = TransToRp(T)
    omgmat = MatrixLog3(R)

    se3mat = np.zeros((4, 4))

    if np.allclose(omgmat, np.zeros((3, 3))):
        se3mat[0:3, 3] = p
        return se3mat

    theta = acos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    omgmat_unit = omgmat / theta

    G_inv = (
        np.eye(3) / theta
        - 0.5 * omgmat_unit
        + (1.0 / theta - 0.5 / np.tan(theta / 2.0))
        * (omgmat_unit @ omgmat_unit)
    )

    se3mat[0:3, 0:3] = omgmat
    se3mat[0:3, 3] = G_inv @ p * theta

    return se3mat


# ============================================================
# Optional numerical validity helpers from Modern Robotics Ch. 3
# ============================================================

def ProjectToSO3(mat):
    """Project a near-rotation matrix onto SO(3)."""
    mat = np.asarray(mat, dtype=float).reshape(3, 3)
    U, _, Vt = np.linalg.svd(mat)
    R = U @ Vt

    if np.linalg.det(R) < 0:
        R[:, 2] = -R[:, 2]

    return R


def ProjectToSE3(mat):
    """Project a near-transform matrix onto SE(3)."""
    mat = np.asarray(mat, dtype=float).reshape(4, 4)
    R = ProjectToSO3(mat[0:3, 0:3])
    p = mat[0:3, 3]

    return RpToTrans(R, p)


def DistanceToSO3(mat):
    """Return distance-like error from SO(3). Large value means invalid det."""
    mat = np.asarray(mat, dtype=float).reshape(3, 3)

    if np.linalg.det(mat) > 0:
        return np.linalg.norm(mat.T @ mat - np.eye(3))

    return 1e9


def DistanceToSE3(mat):
    """Return distance-like error from SE(3). Large value means invalid det."""
    mat = np.asarray(mat, dtype=float).reshape(4, 4)
    R = mat[0:3, 0:3]

    if np.linalg.det(R) > 0:
        T_check = np.r_[
            np.c_[R.T @ R, np.zeros((3, 1))],
            np.array([[0.0, 0.0, 0.0, 1.0]])
        ]
        return np.linalg.norm(T_check - np.eye(4))

    return 1e9


def TestIfSO3(mat, tol: float = 1e-3) -> bool:
    """Return True if mat is close to a valid rotation matrix."""
    return DistanceToSO3(mat) < tol


def TestIfSE3(mat, tol: float = 1e-3) -> bool:
    """Return True if mat is close to a valid homogeneous transform."""
    mat = np.asarray(mat, dtype=float).reshape(4, 4)
    bottom_row_ok = np.allclose(mat[3, :], np.array([0, 0, 0, 1]), atol=tol)

    return bottom_row_ok and DistanceToSE3(mat) < tol

