"""Parts 1 and 2: the same kinematics with MuJoCo's functions (your code goes here).

In student_kinematics.py you derive the forward kinematics and the Jacobian yourself.
Here you get the same quantities from MuJoCo, the way you would in practice, and the
test scripts check both against each other:

Part 1: mujoco_fk()          test: uv run part1_fk.py
Part 2: mujoco_jacobian()    test: uv run part2_ik.py

Rules: use mujoco and NumPy. Don't import rb5_sim. Don't change the function signatures.

The scripts pass you the simulation model and a scratch MjData of your own, so changing
its qpos never moves the simulated robot.

MuJoCo functions and fields you will need (https://mujoco.readthedocs.io):
  model.joint(name).qposadr[0]   index of a joint's angle in data.qpos
  model.joint(name).dofadr[0]    index of a joint's velocity (= Jacobian column)
  model.site(name).id            index of a site
  mujoco.mj_kinematics(m, d)     positions and orientations of all bodies and sites
                                 from data.qpos
  data.site_xpos[i]              site position (3,) in the world (= base) frame
  data.site_xmat[i]              site rotation (9,), row by row
  mujoco.mj_comPos(m, d)         must run (after mj_kinematics) before mj_jacSite
  mujoco.mj_jacSite(m, d, jacp, jacr, i)   linear and angular Jacobians of site i,
                                 each 3 x model.nv
The model holds more than the arm (fingers, pipette, ...), so the arm joints are not the
first entries of qpos and the Jacobian has more than 6 columns.
"""

import mujoco
import numpy as np

# The arm joints in rb5_850e.xml, in the order of q.
ARM_JOINTS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")


# ---- Part 1 --------------------------------------------------------------------------

def mujoco_fk(model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray, site: str = "tcp") -> np.ndarray:
    """Pose of a site at joint angles q, from MuJoCo.

    q:    (6,) joint angles [rad], in the order of ARM_JOINTS
    site: "tcp" is the flange, "grasp" is the hand's grasp site

    Returns the (4, 4) homogeneous transform of the site in the base frame.
    """
    # Look up addresses: the model also contains joints outside the arm.
    for name, angle in zip(ARM_JOINTS, q):
        address = model.joint(name).qposadr[0]
        data.qpos[address] = angle
    mujoco.mj_kinematics(model, data)

    site_id = model.site(site).id
    T = np.eye(4)
    T[:3, :3] = data.site_xmat[site_id].reshape(3, 3)
    T[:3, 3] = data.site_xpos[site_id]
    return T


# ---- Part 2 --------------------------------------------------------------------------

def mujoco_jacobian(model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray, site: str = "grasp") -> np.ndarray:
    """Return the 6x6 [linear; angular] site Jacobian for the six RB5 arm joints."""
    q = np.asarray(q, dtype=float).reshape(6)

    # qpos addresses and velocity/DoF addresses are not interchangeable.
    for i, name in enumerate(ARM_JOINTS):
        data.qpos[model.joint(name).qposadr[0]] = q[i]

    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, model.site(site).id)

    columns = [model.joint(name).dofadr[0] for name in ARM_JOINTS]
    return np.vstack((jacp[:, columns], jacr[:, columns]))
