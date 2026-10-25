import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from models.analytical.franka_arm import FrankaArm
from src.robot_model import RobotModel
from src.system_identification_utility import (
    build_inertial_regressor, sample_state, extract_base_regressor,
)

if __name__=="__main__":
    robot = RobotModel(FrankaArm())
    rng = np.random.default_rng(1)
    regressor_size = 500

    W = np.vstack([
        build_inertial_regressor(robot, *sample_state(robot, max_vel=2.0, max_acc=4.0, rng = rng))
        for _ in range(regressor_size)
    ])

    U, s, Vt = np.linalg.svd(W, full_matrices=False)

    tol = max(W.shape) * np.finfo(s.dtype).eps * s[0]
    rank = np.count_nonzero(s > tol)

    print("W shape:", W.shape)
    print("Numerical rank:", rank)
    print("Unidentifiable directions:", W.shape[1] - rank)

    indices, parameter_map = extract_base_regressor(W, rank)
    W_base = W[:, indices]
    print("Base regressor shape:", W_base.shape)
    print("Independent column indices (zero-based):", indices)
    print("Parameter map shape:", parameter_map.shape)

    # Verify the dependencies on states that were not used to select columns.
    validation_rng = np.random.default_rng(2026)
    W_check = np.vstack([
        build_inertial_regressor(
            robot, *sample_state(robot, max_vel=1.0, max_acc=2.0,
                                 rng=validation_rng)
        )
        for _ in range(100)
    ])
    
    for name, matrix in [("Training", W), ("Independent validation", W_check)]:
        reconstructed = matrix[:, indices] @ parameter_map
        relative_error = np.linalg.norm(matrix - reconstructed) / np.linalg.norm(matrix)
        print(f"{name} reconstruction relative error: {relative_error:.3e}")
        np.testing.assert_allclose(reconstructed, matrix, atol=1e-9, rtol=1e-9)

    # For any full inertial vector theta:
    # theta_base = parameter_map @ theta
    # tau = Y[:, indices] @ theta_base

    plt.semilogy(np.arange(1, len(s) + 1), s, "o-")
    plt.axhline(tol, color="red", linestyle="--", label="Numerical tolerance")
    plt.xlabel("Singular value index")
    plt.ylabel("Singular value")
    plt.grid(True)
    plt.legend()
    plt.show()

