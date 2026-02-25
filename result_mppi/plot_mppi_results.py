#!/usr/bin/env python3

import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import os

CSV_FILE = "mppi_stats0.csv"  
URDF_PATH = "/home/chri/CHRI_2025/Time_optimal_baseline/ur5_urdf/full_UR5.urdf" 
END_EFFECTOR_LINK_INDEX = 8   
JOINT_INDICES_URDF = [1,2,3,4,5,6] 
DT = 0.05

# Continuous SSM limit 
def ssm_velocity_limit_from_distance(d_eff, Vh=2.0, ar_max=4.6*0.9119, t2=0.100):
    """
    d_eff = clearance distance available for braking
    MATLAB formula:
    Vr = -(t2 + Vh/ar) + sqrt((t2 + Vh/ar)^2 + (2/ar)*d_eff) * ar
    """
    if d_eff <= 0:
        return 0.0

    term = (t2 + Vh/ar_max)
    inside = (term * term) - 2.0/ar_max * (-d_eff)    # == (term)^2 + 2*d_eff/ar_max

    if inside < 0:
        return 0.0

    Vr = ( -term + np.sqrt(inside) ) * ar_max
    return max(0.0, Vr)


# Load CSV
data = np.loadtxt(CSV_FILE, delimiter=',')

secs = data[:,0]
nsecs = data[:,1]
timestamps = secs + nsecs*1e-9

dt_arr = np.ones_like(timestamps) * DT

goal_joints = data[:, 4:10]
meas_joints = data[:, 10:16]

# 56 human values begin at col 16
human_flat = data[:, 16:]
human_spheres = human_flat.reshape((-1, 14, 4))  # (N,14,4)

# joint velocities
joint_vel = np.vstack([np.zeros(6), np.diff(meas_joints, axis=0)]) / dt_arr.reshape(-1,1)


# PyBullet FK
physics_id = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

if not os.path.exists(URDF_PATH):
    raise FileNotFoundError(f"URDF not found at {URDF_PATH}.")

robot_uid = p.loadURDF(URDF_PATH, useFixedBase=True)

ee_positions = []
ee_speeds = []
dist_to_goal = []
min_dist_human = []  

for i in range(meas_joints.shape[0]):
    q = meas_joints[i].tolist()
    q_goal = goal_joints[i].tolist()

    # set measured robot joints
    for j_idx, qval in zip(JOINT_INDICES_URDF, q):
        p.resetJointState(robot_uid, j_idx, qval)

    link_state = p.getLinkState(robot_uid, END_EFFECTOR_LINK_INDEX, computeForwardKinematics=True)
    ee_pos = np.array(link_state[4])
    ee_positions.append(ee_pos)

    # Jacobian-based EE linear velocity
    jac = p.calculateJacobian(robot_uid, END_EFFECTOR_LINK_INDEX, [0,0,0],
                              q, [0]*6, [0]*6)
    Jv = np.array(jac[0])  # 3x<n_joints>
    Jv = Jv[:, :6]         # keep 6 joints

    qdot = joint_vel[i].reshape(6,1)
    ee_vel = (Jv @ qdot).reshape(3)
    ee_speeds.append(np.linalg.norm(ee_vel))

    # goal EE position
    for j_idx, qval in zip(JOINT_INDICES_URDF, q_goal):
        p.resetJointState(robot_uid, j_idx, qval)

    link_state_goal = p.getLinkState(robot_uid, END_EFFECTOR_LINK_INDEX, computeForwardKinematics=True)
    goal_pos = np.array(link_state_goal[4])
    dist_to_goal.append(np.linalg.norm(ee_pos - goal_pos))

    # Compute robot-human minimum distance
    # each sphere: [hx, hy, hz, r]
    # signed dist = ||center - ee|| - r
    spheres = human_spheres[i]
    centers = spheres[:, 0:3]
    radii   = spheres[:, 3]

    dists = np.linalg.norm(centers - ee_pos[None,:], axis=1) - radii
    min_dist_human.append(np.min(dists))


p.disconnect()

ee_positions = np.array(ee_positions)
ee_speeds = np.array(ee_speeds)
dist_to_goal = np.array(dist_to_goal)
min_dist_human = np.array(min_dist_human)

# Compute SSM curve using minimum robot-human distance
# Use robot radius rr to compute effective distance
rr = 0.15
d_eff = min_dist_human - rr       # remaining clearance
ssm_curve = np.array([ssm_velocity_limit_from_distance(d) for d in d_eff])


# PLOTS
t = timestamps - timestamps[0]

fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

# (1) EE speed + SSM limit
axs[0].plot(t, ee_speeds, label='End-effector speed (m/s)')
axs[0].plot(t, ssm_curve, '--', label='SSM velocity limit', linewidth=1.5)
axs[0].set_ylabel('speed (m/s)')
axs[0].legend()
axs[0].grid()

# (2) real robot–human minimum distance
axs[1].plot(t, min_dist_human, label='Min EE-to-human distance (m)')
axs[1].set_ylabel("dist (m)")
axs[1].legend()
axs[1].grid()

# (3) distance to goal
axs[2].plot(t, dist_to_goal, color='black', label='Distance to goal (m)')
axs[2].set_ylabel('distance (m)')
axs[2].legend()
axs[2].grid()

# (4) joint error
j_error = data[:,3]
axs[3].plot(t, j_error, label='joint error (rad)')
axs[3].set_xlabel('time (s)')
axs[3].legend()
axs[3].grid()

plt.tight_layout()
plt.show()
