"""Parts 1 and 2: forward and inverse kinematics of the RB5 (your code goes here).

Part 1: forward_kinematics()                  test: uv run part1_fk.py
Part 2: jacobian(), inverse_kinematics()      test: uv run part2_ik.py

The robot controller calls forward_kinematics() and inverse_kinematics() for every move,
so the pipetting task (Part 3) runs on your kinematics. In student_mujoco.py you compute
the same FK and Jacobian with MuJoCo's functions, and the tests compare the two.

Rules: use NumPy only. Don't import mujoco or rb5_sim in this file. You may add your own
helper functions. Don't change the function signatures or the provided helpers.

Frames and units: meters and radians. Poses are 4x4 homogeneous transforms.
  base frame    the robot base (= the world frame of the simulation)
  flange frame  the RB5 tool flange: z out of the flange (site "tcp" in rb5_850e.xml)
  tool frame    a frame fixed to the flange, given by `tool` = its pose in the flange
                frame. The robot passes the hand's "grasp" site here.
"""

import numpy as np

# Joint limits [rad], the same for all 6 joints.
Q_LO = np.full(6, -6.2832)
Q_HI = np.full(6, 6.2832)


# ---- provided helpers ----------------------------------------------------------------

def rotation_vector(R: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle [rad]) of the rotation matrix R."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-9:
        return 0.5 * v
    if angle > np.pi - 1e-6:  # sin(angle) ~ 0: take the axis from R + I instead
        B = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(B)))
        axis = B[:, k] / np.sqrt(B[k, k])
        return angle * axis
    return angle / (2.0 * np.sin(angle)) * v


def pose_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """6D error from the current pose to the target pose (both 4x4), in the base frame.

    [0:3] position difference [m]; [3:6] rotation vector [rad] that turns the current
    orientation into the target orientation.
    """
    dp = target[:3, 3] - current[:3, 3]
    dw = rotation_vector(target[:3, :3] @ current[:3, :3].T)
    return np.concatenate([dp, dw])


# ---- Part 1: forward kinematics ------------------------------------------------------

def forward_kinematics(q: np.ndarray, tool: np.ndarray = np.eye(4)) -> np.ndarray:
    """Pose of the tool frame in the base frame.

    q:    (6,) joint angles [rad]: base, shoulder, elbow, wrist1, wrist2, wrist3
    tool: (4, 4) pose of the tool frame in the flange frame (identity = the flange itself)

    Returns the (4, 4) homogeneous transform T_base_tool.
    """
    # Each joint first translates in its parent frame, then rotates locally.
    offsets = ((0, 0, 0.1692), (0, 0, 0), (0, 0, 0.425),
               (0, 0, 0.392), (0, -0.1107, 0.1107), (0, 0, 0))
    axes = ("z", "y", "y", "y", "z", "y")
    T = np.eye(4)
    for angle, offset, axis in zip(q, offsets, axes):
        c, s = np.cos(angle), np.sin(angle)
        if axis == "z":
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        else:
            R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        joint = np.eye(4)
        joint[:3, :3] = R
        joint[:3, 3] = offset
        T = T @ joint

    # Fixed flange: translate along link6 -y, then rotate +90 deg about x.
    flange = np.array([[1, 0, 0, 0],
                       [0, 0, -1, -0.0967],
                       [0, 1, 0, 0],
                       [0, 0, 0, 1]], dtype=float)
    return T @ flange @ tool


# ---- Part 2: Jacobian and inverse kinematics -----------------------------------------

def jacobian(q: np.ndarray, tool: np.ndarray = np.eye(4)) -> np.ndarray:
    """6x6 geometric Jacobian of the tool origin, expressed in the base frame.

    For each revolute joint i:
        J[:3, i] = axis_i x (p_tool - p_joint_i)
        J[3:, i] = axis_i
    The geometry follows the exact transform order used in forward_kinematics().
    """
    q = np.asarray(q, dtype=float).reshape(6)
    tool = np.asarray(tool, dtype=float)

    def rot_y(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    def rot_z(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    # Before each revolute rotation, record the joint origin and axis in base.
    # The translations are in the PRECEDING joint's rotating frame.
    local_axes = (
        np.array([0.0, 0.0, 1.0]),  # base
        np.array([0.0, 1.0, 0.0]),  # shoulder
        np.array([0.0, 1.0, 0.0]),  # elbow
        np.array([0.0, 1.0, 0.0]),  # wrist1
        np.array([0.0, 0.0, 1.0]),  # wrist2
        np.array([0.0, 1.0, 0.0]),  # wrist3
    )
    offsets = (
        (0.0, 0.0, 0.1692),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.425),
        (0.0, 0.0, 0.392),
        (0.0, -0.1107, 0.1107),
        (0.0, 0.0, 0.0),
    )

    R = np.eye(3)
    p = np.zeros(3)
    origins = np.empty((6, 3))
    axes = np.empty((6, 3))

    for i in range(6):
        p = p + R @ np.asarray(offsets[i])
        origins[i] = p
        axes[i] = R @ local_axes[i]
        R = R @ (rot_z(q[i]) if i in (0, 4) else rot_y(q[i]))

    # Flange translation (0, -0.0967, 0) and its fixed +90 deg x rotation.
    # A point [tx, ty, tz] in the tool frame is rotated by Rx(pi/2),
    # giving [tx, -tz, ty] in the last wrist frame.
    tx, ty, tz = tool[:3, 3]
    p_tool = p + R @ np.array([tx, -0.0967 - tz, ty])

    J = np.empty((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(axes[i], p_tool - origins[i])
        J[3:, i] = axes[i]
    return J


def inverse_kinematics(target: np.ndarray, q_init: np.ndarray, tool: np.ndarray = np.eye(4),
                       max_iters: int = 200, tol: float = 1e-6) -> tuple[np.ndarray, float]:
    """Iterative 6-DoF IK using the analytic Jacobian and damped least squares.

    The solution is initialized at q_init (to retain the intended IK branch).
    Joint-step limits and a short backtracking search avoid unstable jumps.
    """
    q = np.clip(np.asarray(q_init, dtype=float).reshape(6).copy(), Q_LO, Q_HI)
    target = np.asarray(target, dtype=float)
    tool = np.asarray(tool, dtype=float)
    eye6 = np.eye(6)
    step_limit = 0.30          # rad / iteration, per joint
    lam = 1e-3                # DLS regularization, robust near singularities

    current = forward_kinematics(q, tool)
    e = pose_error(target, current)
    err = float(np.linalg.norm(e))

    for _ in range(max_iters):
        if err < tol:
            break

        J = jacobian(q, tool)
        # Use solve() rather than explicitly inverting the 6x6 matrix.
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * eye6, e)
        dq = np.clip(dq, -step_limit, step_limit)

        # Full update is fastest near the target. Backtrack only if the
        # nonlinear FK error would grow, such as near a singular pose.
        accepted = False
        scale = 1.0
        for _ in range(7):
            candidate = np.clip(q + scale * dq, Q_LO, Q_HI)
            next_error = pose_error(target, forward_kinematics(candidate, tool))
            next_norm = float(np.linalg.norm(next_error))
            if next_norm < err:
                q, e, err = candidate, next_error, next_norm
                accepted = True
                break
            scale *= 0.5

        if not accepted:
            # A rejected step could indicate proximity to a singular pose or
            # joint limits. Increase damping without leaving the current branch.
            lam = min(lam * 10.0, 1.0)
        else:
            lam = max(lam * 0.5, 1e-5)

    return q, err