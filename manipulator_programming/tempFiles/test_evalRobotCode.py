import numpy as np
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tempFiles.test_robot import ThreeDOFManipulator

def main():
    robot = ThreeDOFManipulator()

    tests = [
        {
            "name": "A: forward/right/up",
            "p_d": np.array([1.1433, 0.7178, 0.4206]),
            "theta0_deg": [25, 25, -35],
        },
        {
            "name": "B: left/high",
            "p_d": np.array([0.6269, -0.5562, 1.2401]),
            "theta0_deg": [-35, 35, 20],
        },
        {
            "name": "C: side reach",
            "p_d": np.array([0.5872, 1.1171, 0.0910]),
            "theta0_deg": [50, -10, 55],
        },
        {
            "name": "D: not reachable, too far",
            "p_d": np.array([2.1, 0.05, 0.05]),
            "theta0_deg": [0, 0, 0],
        },
    ]

    for test in tests:
        p_d = test["p_d"]
        theta0 = np.deg2rad(test["theta0_deg"])

        theta_space, success_space, k_space = robot.iKinPosition(
            p_d,
            theta0,
            ep=0.01,
            maxIterations=300,
            damping=1e-2,
            stepScale=0.5,
        )

        theta_body, success_body, k_body = robot.iKinPositionBodyMR(
            p_d,
            theta0,
            ep=0.01,
            maxIterations=300,
            damping=1e-2,
            stepScale=0.5,
        )

        p_space = robot.fKinSpace(theta_space)[:3, 3]
        p_body = robot.fKinSpace(theta_body)[:3, 3]

        err_space = np.linalg.norm(p_d - p_space)
        err_body = np.linalg.norm(p_d - p_body)

        print("\n" + "=" * 60)
        print(test["name"])
        print("Target p_d:", np.round(p_d, 4))
        print("Initial guess deg:", test["theta0_deg"])

        print("\nSpace-frame position Jacobian IK")
        print("success:", success_space)
        print("iterations:", k_space)
        print("theta deg:", np.round(np.rad2deg(theta_space), 3))
        print("achieved p:", np.round(p_space, 4))
        print("position error:", round(err_space, 6))

        print("\nMR body-frame position IK")
        print("success:", success_body)
        print("iterations:", k_body)
        print("theta deg:", np.round(np.rad2deg(theta_body), 3))
        print("achieved p:", np.round(p_body, 4))
        print("position error:", round(err_body, 6))


if __name__ == "__main__":
    main()
