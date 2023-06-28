#!/usr/bin/env python3
# -*- coding: utf-8 -*
# from time import time

use_real_robot = False

import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import sys
from utils import ur5_dh, sdf
from utils import mr_casadi as mc
if use_real_robot:
    from utils import arm_control_client
from utils.hmqr5_agl_map import hmqr5_agl_map
import rospy
from scipy.spatial.transform import Rotation
from trajectory_generator_ee import trajectory_generator_door
from geometry_msgs.msg import Pose

import os
from sensor_msgs.msg import JointState

o_path = '/root/catkin_ws/src/open_door_mpc/script'
o_path = o_path + '/utils'
sys.path.append(o_path)

from utils.omnirob_ur5_ros import OmniRobUr5Ros
rob = OmniRobUr5Ros()

# dt = 0.001  # 10ms

agl_set = np.array([])
# loop_count: every loop_count epoch(es) control arm
def ctrl_rob(cfg, u_base, gripper_config, loop_time, loop_count=1, gripper_state=1, real=False):
    desired_world_positions = np.zeros(3, dtype=np.float64)
    desired_arm_joint_angles = np.zeros(6, dtype=np.float64)

    desired_world_positions[0] = cfg[0][0]
    desired_world_positions[1] = cfg[0][1]
    desired_world_positions[2] = cfg[0][2]

    desired_arm_joint_angles[0] = cfg[0, 3]
    desired_arm_joint_angles[1] = cfg[0, 4]
    desired_arm_joint_angles[2] = cfg[0, 5]
    desired_arm_joint_angles[3] = cfg[0, 6]
    desired_arm_joint_angles[4] = cfg[0, 7]
    desired_arm_joint_angles[5] = cfg[0, 8]
    if not real:
        rob.world_frame_set(desired_world_positions)
        rob.arm_ctrl(desired_arm_joint_angles)
    else:
        rob.base_ctrl(x=u_base[0], y=0, theta=u_base[1])
        rob.gripper_ctrl(gripper_config)
        print("desired_arm_joint_angles: ", desired_arm_joint_angles)
        global agl_set
        agl_set = np.append(agl_set, desired_arm_joint_angles, axis=0)
        if(agl_set.size==6*loop_count):
            # rob.base_ctrl(x=0, y=0, theta=0)
            print("agl_set:", agl_set)
            result = arm_control_client.arm_control_client(agl_set, loop_count)
            print(result)
            agl_set = np.array([])


Q_x, Q_y, Q_theta = 0, 0, 0
# state weight matrix (Q_X, Q_Y, Q_THETA, q1, ..., q6)[9]
Q = ca.diagcat(Q_x, Q_y, Q_theta, 0, 0, 0, 0, 0, 0)
# controls weights matrix
# (v, theta, q1, ..., q6)[8]
Rv, Rw, Ra = 1, 1, 1
R = ca.diagcat(Rv, Rw, Ra, Ra, Ra, Ra, Ra, Ra)

step_horizon = 0.1  # time between steps in seconds
N = 30  # number of look ahead steps
rob_diam = 0.3  # diameter of the robot

sim_time = 1000  # simulation time

# specs target
# base
x_init, y_init, theta_init = 0, 0, 0
# car base
# x_target, y_target, theta_target = -3.2, 3.8, pi / 2
x_target, y_target, theta_target = 0.55, -0.65, pi / 4
# arm tip
arm_tip_target_x = -3
arm_tip_target_y = 3
arm_tip_target_z = 1.0

# q0 = rob.get_amr_pos()
# while abs(q0[0]) <= 1e-4:
#     print("q_init:", q0)
#     q0 = rob.get_amr_pos()
#     rospy.sleep(0.1)

# q0 = [1.5778058448580374, -1.996235163274898, -1.5911667359506643, 0, np.pi/2, 0]
# q0 = [1.577, -1.996, -1.591, -0.293, -0.194, 1.735]
q0 = [-1.591, -1.096, 1.577, -0.293, -0.194, 1.735]
print("q_init:", q0)
# assert 0
state_init = ca.DM([x_init, y_init, theta_init, q0[0], q0[1], q0[2], q0[3], q0[4], q0[5]])  # initial state
state_target = ca.DM([x_target, y_target, theta_target, 0, 0, 0, 0, 0, 0])  # target state


Qa_x, Qa_y, Qa_z = 200, 200, 200
Qa = ca.diag([Qa_x, Qa_y, Qa_z])
Qa_rot = ca.diag([10, 10, 10])

# obs
obs_x, obs_y, obs_z = 1.18, -0.5, 1.0 # hinge的右边
obs_diam = 0.1
Q_obs = ca.diag([0, 1, 0]) # 不管z轴
Q_obs_base = ca.diag([1, 1])
epi_obs = 0.1


#
v_max = 0.2
v_min = -0.2
v_arm_max = 0.1
v_arm_min = -0.1

o_path = os.getcwd()
o_path = o_path + '/utils'
print("o_path:", o_path)
sys.path.append(o_path)
def shift_timestep(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:, 0])
    next_state = ca.DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )
    return t0, next_state, u0

def DM2Arr(dm):
    return np.array(dm.full())

# state symbolic variables
q_mm = ca.SX.sym('q_mm', 9)  # (x,y,theta,q1-q6)
u_mm = ca.SX.sym('u_mm', 8)  # dot(v,w,q1-q6)

# states symbolic variables
n_states = q_mm.numel()
# control symbolic variables
n_controls = u_mm.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', n_controls, N)

# coloumn vector for storing initial state and target state
# P = ca.SX.sym('P', n_states + n_states)
# [base initial state and target state].T + [arm trajectory].T
# i th arm trajectory = [x_i, y_i, z_i, angle_axis](6X1)
P = ca.SX.sym('P', n_states + n_states + 2 + 2 + 7 * (N + 1))
# P2 = ca.SX.sym('P', 6, N+1)
P_gra = P[2 * n_states:2 * n_states + 2]
P_dw_base = P[2 * n_states + 2:2 * n_states + 3]
P_dw_arm = P[2 * n_states + 3:2 * n_states + 4]
P_door = P[2 * n_states:2 * n_states + 4]
P_arm = ca.reshape(P[4 + 2 * n_states:], 7, N + 1)
print("P_arm:", P_arm)
# # assert 0
theta = q_mm[2]
v = u_mm[0]
omega = u_mm[1]
RHS = ca.vertcat(v * ca.cos(theta),
                 v * ca.sin(theta),
                 omega,
                 u_mm[2],
                 u_mm[3],
                 u_mm[4],
                 u_mm[5],
                 u_mm[6],
                 u_mm[7])
f = ca.Function('f_mm', [q_mm, u_mm], [RHS])

# 构建arm tip的障碍物RBF
joint5_diam = 0.10
tip_pos = ca.SX.sym('tip_pos', 3)  # (x,y)
dh = tip_pos - ca.DM([obs_x, obs_y, obs_z])
h = ca.sqrt(dh.T @ Q_obs @ dh) - joint5_diam / 2
rbf_out = -epi_obs * ca.log(h)
f_rbf = ca.Function('rbf', [tip_pos], [rbf_out], ['tip_pos'], ['rbf'])

# 构建障碍物RBF1
base_pos = ca.SX.sym('base_pos', 2)  # (x,y)
dh = base_pos - ca.DM([obs_x, obs_y])
# dh = sdf.doorSDF()
h = ca.sqrt(dh.T @ Q_obs_base @ dh) - obs_diam / 2 - rob_diam / 2
# rbf_out = -epi_obs * ca.log(h)
# rbf_base_out = ca.if_else(h < 3, -epi_obs * ca.log(h), 0) # 这儿分段会导致求解速度变慢
rbf_base_out = -epi_obs * ca.log(h)
f_rbf_base = ca.Function('rbf_base', [base_pos], [rbf_base_out], ['base_pos'], ['rbf_base'])
print(f"f_rbf_base: {f_rbf_base}")

# 构建障碍物RBF2
base_pos = ca.SX.sym('base_pos', 2)  # (x,y)
dh = base_pos - ca.DM([obs_x, 1])
h = ca.sqrt(dh.T @ Q_obs_base @ dh) - obs_diam / 2 - rob_diam / 2
# rbf_out = -epi_obs * ca.log(h)
# rbf_base_out = ca.if_else(h < 3, -epi_obs * ca.log(h), 0) # 这儿分段会导致求解速度变慢
rbf_base_out2 = -epi_obs * ca.log(h)
f_rbf_base2 = ca.Function('rbf_base2', [base_pos], [rbf_base_out2], ['base_pos'], ['rbf_base'])
print(f"f_rbf_base: {f_rbf_base2}")

# fake esdf
base_pos = ca.SX.sym('base_pos', 2)  # (x,y)
a = P_door[:2]
b = P_door[2:]
esdf_func = sdf.myLineSdfCasadi(base_pos, a, b) - rob_diam / 2
epi_obs = 0.01
rbf_base_out = -epi_obs * ca.log(esdf_func)
f_rbf_esdf_base = ca.Function('f_rbf_esdf_base', [base_pos], [rbf_base_out], ['base_pos'], ['rbf_base'])
print(f"f_rbf_esdf_base: {f_rbf_esdf_base}")


# =============================================== cost function ========================================================
cost_fn = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation

# runge kutta
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    x_arm, R_arm = ur5_dh.RobFki(st)
    # R_err = R_arm.T @ R_target
    # from utils import mr_casadi as mc
    # r_err = mc.RotationErrorByQuaternion(R_arm, R_target)
    qua_arm = mc.RotationToQuaternion(R_arm)
    r_err = mc.QuaternionError(qua_arm, P_arm[3:, k])
    x_arm_t = np.array([arm_tip_target_x, arm_tip_target_y, arm_tip_target_z])
    # ==============
    # a = ca.DM([P_door[0], P_door[1]])
    # b = ca.DM([P_door[2], P_door[3]])
    # esdf_func = sdf.myLineSdfCasadi(st[:2], a, b) - rob_diam / 2
    # rbf_base_out = -epi_obs * ca.log(esdf_func)
    # ==============
    # base_door_error = f_rbf_two_pos_error(ca.DM([st[0], st[1],x_arm[0],x_arm[1]]))
    # print(base_door_error)
    # + (x_arm - x_arm_t).T @ Qa @ (x_arm - x_arm_t) \
    # + (x_arm - P[:3, k + 1]).T @ Qa @ (x_arm - P[:3, k + 1]) \
    x_arm3, _ = ur5_dh.RobFki(st, 2)
    x_arm4, _ = ur5_dh.RobFki(st, 3)
    rbf_joints = f_rbf(x_arm3)
    rbf_joints += f_rbf(x_arm4)
    cost_fn = cost_fn \
                + con.T @ R @ con \
                + (x_arm - P_arm[:3, k]).T @ Qa @ (x_arm - P_arm[:3, k]) \
                + r_err.T @ Qa_rot @ r_err 
                # + f_rbf_esdf_base(st[:2])
                # + P_dw_base * (st - P[n_statstate_inites:2 * n_states]).T @ Q @ (st - P[n_states:2 * n_states]) \
                # + rbf_joints\
    st_next = X[:, k + 1]
    k1 = f(st, con)
    k2 = f(st + step_horizon / 2 * k1, con)
    k3 = f(st + step_horizon / 2 * k2, con)
    k4 = f(st + step_horizon * k3, con)
    st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    g = ca.vertcat(g, st_next - st_next_RK4)
# Todo: Not Add Terminal Cost yet
# =============================================== cost function ========================================================
OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),  # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
)
nlp_prob = {
    'f': cost_fn,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))
ubx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))

lbx[0: n_states * (N + 1): n_states] = -ca.inf  # X lower bound
lbx[1: n_states * (N + 1): n_states] = -ca.inf  # Y lower bound
lbx[2: n_states * (N + 1): n_states] = -ca.inf  # theta lower bound

am = hmqr5_agl_map()
lbx[3: n_states * (N + 1): n_states] = am.limit[0][0]  # q1 lower bound
lbx[4: n_states * (N + 1): n_states] = am.limit[1][0]  # q2 lower bound
lbx[5: n_states * (N + 1): n_states] = am.limit[2][0]  # q3 lower bound
lbx[6: n_states * (N + 1): n_states] = am.limit[3][0]  # q4 lower bound
lbx[7: n_states * (N + 1): n_states] = am.limit[4][0]  # q5 lower bound
lbx[8: n_states * (N + 1): n_states] = am.limit[5][0]  # q6 lower bound

# print("limit 6:", am.limit[5][0], am.limit[5][1])
# assert 0
ubx[0: n_states * (N + 1): n_states] = ca.inf  # X upper bound
ubx[1: n_states * (N + 1): n_states] = ca.inf  # Y upper bound
ubx[2: n_states * (N + 1): n_states] = ca.inf  # theta upper bound

ubx[3: n_states * (N + 1): n_states] = am.limit[0][1]  # q1 upper bound
ubx[4: n_states * (N + 1): n_states] = am.limit[1][1]  # q2 upper bound
ubx[5: n_states * (N + 1): n_states] = am.limit[2][1]  # q3 upper bound
ubx[6: n_states * (N + 1): n_states] = am.limit[3][1]  # q4 upper bound
ubx[7: n_states * (N + 1): n_states] = am.limit[4][1]  # q5 upper bound
ubx[8: n_states * (N + 1): n_states] = am.limit[5][1]  # q6 upper bound

lbx[n_states * (N + 1):] = v_arm_min  # u lower bound for all U
ubx[n_states * (N + 1):] = v_arm_max  # u upper bound for all U
lbx[n_states * (N + 1): -1: n_controls] = v_min  # u upper bound for v
lbx[n_states * (N + 1) + 1: -1: n_controls] = v_min  # u upper bound for w
ubx[n_states * (N + 1): -1: n_controls] = v_max  # u upper bound for v
ubx[n_states * (N + 1) + 1: -1: n_controls] = v_max  # u upper bound for w

args = {
    'lbg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints lower bound
    'ubg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints upper bound
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0

# xx = DM(state_init)
t = ca.DM(t0)

u0 = ca.DM.zeros((n_controls, N))  # initial control
X0 = ca.repmat(state_init, 1, N + 1)  # initial state full

mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])

arg_p_arm_mat = ca.DM.zeros(7, (N + 1))  # end effector trajectory


esdf_gradient = ca.DM([0, 0])
ee_pos_record = []

from utils import visualization as vl
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path

ee_planning_path_pub = rospy.Publisher('ee_planning_trajectory', Path, queue_size=50)
base_predict_path_pub = rospy.Publisher('base_predict_trajectory', Path, queue_size=50)
ee_real_path_pub = rospy.Publisher('ee_real_trajectory', Path, queue_size=50)
rob_sphere_pub = rospy.Publisher("rob_sphere_pub", MarkerArray, queue_size=10)
p_all_record = []

arg_p_arm = ca.DM.zeros(7 * (N + 1))
pose0 = [0,0,1,0,0.707,0,0.707] # [x,y,z, ox,oy,oz,ow]
for k in range(N + 1):
        for j in range(7):
            arg_p_arm[k * 7 + j] = pose0[j]   

def go_target_pose(pose_msg):
    print(f"Target:\n{pose_msg}")
    global arg_p_arm, N
    p, ori = pose_msg.position, pose_msg.orientation
    args = [p.x, p.y, p.z, ori.x, ori.y, ori.z, ori.w]  
    for k in range(N + 1):
        for j in range(7):
            arg_p_arm[k * 7 + j] = args[j]   
    

##############################################################################
if __name__ == '__main__':
    rospy.Subscriber("/mobile_manipulator_mpc_target", Pose, go_target_pose)
    rate = rospy.Rate(50)
    while(not rospy.is_shutdown()):
        main_loop = time.time()  # return time in sec
        door_info = ca.DM([1.2, -0.65, 1.2, 0])
        mpc_iter = 0
        while (ca.norm_2(state_init - state_target) > 1e-1):
                t1 = time.time()        
                args['p'] = ca.vertcat(
                    state_init,  # current state
                    state_target,  # target state
                    door_info,
                    arg_p_arm
                )
                # optimization variable current state（sates and controls）
                args['x0'] = ca.vertcat(
                    ca.reshape(X0, n_states * (N + 1), 1),
                    ca.reshape(u0, n_controls * N, 1)
                )
                sol = solver(
                    x0=args['x0'],
                    lbx=args['lbx'],
                    ubx=args['ubx'],
                    lbg=args['lbg'],
                    ubg=args['ubg'],
                    p=args['p']
                )
                u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
                X0 = ca.reshape(sol['x'][: n_states * (N + 1)], n_states, N + 1)
                cat_states = np.dstack((
                    cat_states,
                    DM2Arr(X0)
                ))
                cat_controls = np.vstack((
                    cat_controls,
                    DM2Arr(u[:, 0])
                ))
                t = np.vstack((
                    t,
                    t0
                ))
                t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)
                # ======================================== CTROL =====================================================
                # ======================================== CTROL =====================================================
                ctrl_rob(state_init.T, u[:2,0], 0, mpc_iter, loop_count=1, real=use_real_robot)
                if use_real_robot:
                    real_pose = rob.get_world_frame()
                    state_init[0, 0] = real_pose[0]
                    state_init[1, 0] = real_pose[1]
                    state_init[2, 0] = real_pose[2]
                # arm_pos = rob.get_amr_pos()
                x_arm_now, R_arm_now = ur5_dh.RobFki(state_init)
                ee_pos_record.append(x_arm_now)
                # vl.DrawPath(ee_planning_path_pub, traj_ee, 'odom', 0)
                vl.DrawBasePredictPath(base_predict_path_pub, X0, 'odom')
                vl.PubEndEffectorRealTrajectory(ee_real_path_pub, x_arm_now)
                p_all = vl.DrawRobSphere(rob_sphere_pub, state_init[:, 0], scale=0.1)
                p_all_record.append(p_all)

                X0 = ca.horzcat(
                    X0[:, 1:],
                    ca.reshape(X0[:, -1], -1, 1)
                )
                # xx ...
                t2 = time.time()
                # print(mpc_iter)
                # print("Time cost:", t2 - t1)
                times = np.vstack((
                    times,
                    t2 - t1
                ))
                mpc_iter = mpc_iter + 1
            # main_loop_time = time.time()
            # ss_error = ca.norm_2(state_init - state_target)
            # print('\n\n')
            # print(f'Total time: {main_loop_time - main_loop} s.')
            # print('Avg iteration time: ', np.array(times).mean() * 1000, 'ms')
            # print('Final error: ', ss_error)
            # print('ATE: ', ATE/mpc_iter)
        rate.sleep()
