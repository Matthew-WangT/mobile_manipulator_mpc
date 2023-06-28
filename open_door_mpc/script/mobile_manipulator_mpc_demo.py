#!/usr/bin/env python3
# -*- coding: utf-8 -*
# from time import time

import time
import rospy
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import sys
from utils import ur5_dh, sdf
from utils import mr_casadi as mc
from utils.hmqr5_agl_map import hmqr5_agl_map
from geometry_msgs.msg import Pose
import os

o_path = os.environ['HOME'] + '/catkin_ws/src/open_door_mpc/script/utils'
sys.path.append(o_path)

# Visualize in RVIZ
from utils import visualization as vl
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path

from utils.omnirob_ur5_ros import OmniRobUr5Ros
rob = OmniRobUr5Ros()

class MPC:
    def __init__(self):
        # set the Parameters of MPC 
        Q_x, Q_y, Q_theta = 0, 0, 0 # 所以实际上没有指定底盘的目标点
        # state weight matrix (Q_X, Q_Y, Q_THETA, q1, ..., q6)[9]
        self.Q = ca.diagcat(Q_x, Q_y, Q_theta, 0, 0, 0, 0, 0, 0)
        # controls weights matrix
        Rv, Rw, Ra = 1, 1, 1
        # (v, theta, q1, ..., q6)[8]
        self.R = ca.diagcat(Rv, Rw, Ra, Ra, Ra, Ra, Ra, Ra)
        self.step_horizon = 0.1  # time between steps in seconds
        self.N = 30  # number of look ahead steps
        self.rob_diam = 0.3  # diameter of the robot

        # states symbolic variables
        self.n_states = 9
        # control symbolic variables
        self.n_controls = 8
        # initial state
        x_init, y_init, theta_init = 0, 0, 0
        q0 = [-1.591, -1.096, 1.577, -0.293, -0.194, 1.735]
        self.state_init = ca.DM([x_init, y_init, theta_init, q0[0], q0[1], q0[2], q0[3], q0[4], q0[5]])  # initial state
        # initial target
        # car base
        x_target, y_target, theta_target = 0.55, -0.65, pi / 4
        # arm tip
        self.state_target = ca.DM([x_target, y_target, theta_target, 0, 0, 0, 0, 0, 0])  # target state
        # set the para of arm
        Qa_x, Qa_y, Qa_z = 200, 200, 200
        self.Qa = ca.diag([Qa_x, Qa_y, Qa_z])
        self.Qa_rot = ca.diag([10, 10, 10])
        # control limit
        self.v_max = 0.2
        self.v_min = -0.2
        self.v_arm_max = 0.1
        self.v_arm_min = -0.1

        self.create_symbolic_variables()
        self.build_cost_function()
        self.set_limit()
        self.set_optimize_option()

    def create_symbolic_variables(self):
        # state symbolic variables
        q_mm = ca.SX.sym('q_mm', self.n_states)  # (x,y,theta,q1-q6)
        u_mm = ca.SX.sym('u_mm', self.n_controls)  # dot(v,w,q1-q6)

        # matrix containing all states over all time steps +1 (each column is a state vector)
        self.X = ca.SX.sym('X', self.n_states, self.N + 1)
        # matrix containing all control actions over all time steps (each column is an action vector)
        self.U = ca.SX.sym('U', self.n_controls, self.N)
        self.P = ca.SX.sym('P', 2*self.n_states + 7 * (self.N + 1))
        self.P_arm = ca.reshape(self.P[2 * self.n_states:], 7, self.N + 1)
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
        self.f = ca.Function('f_mm', [q_mm, u_mm], [RHS])

    def build_cost_function(self):
        self.cost_fn = 0  # cost function
        self.g = self.X[:, 0] - self.P[:self.n_states]  # constraints in the equation
        # runge kutta
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            x_arm, R_arm = ur5_dh.RobFki(st)
            qua_arm = mc.RotationToQuaternion(R_arm)
            r_err = mc.QuaternionError(qua_arm, self.P_arm[3:, k])
            self.cost_fn = self.cost_fn \
                        + st.T @ self.Q @ st \
                        + con.T @ self.R @ con \
                        + (x_arm - self.P_arm[:3, k]).T @ self.Qa @ (x_arm - self.P_arm[:3, k]) \
                        + r_err.T @ self.Qa_rot @ r_err 
            st_next = self.X[:, k + 1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon / 2 * k1, con)
            k3 = self.f(st + self.step_horizon / 2 * k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_RK4 = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.g = ca.vertcat(self.g, st_next - st_next_RK4)

    def set_optimize_option(self):
        OPT_variables = ca.vertcat(
            self.X.reshape((-1, 1)),  # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            self.U.reshape((-1, 1))
        )
        nlp_prob = {
            'f': self.cost_fn,
            'x': OPT_variables,
            'g': self.g,
            'p': self.P
        }
        opts = {
            'ipopt': {
                'max_iter': 1000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    def set_limit(self):
        n_states, n_controls = self.n_states, self.n_controls
        N = self.N
        lbx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))
        ubx = ca.DM.zeros((n_states * (N + 1) + n_controls * N, 1))

        lbx[0: n_states * (N + 1): n_states] = -ca.inf  # X lower bound
        lbx[1: n_states * (N + 1): n_states] = -ca.inf  # Y lower bound
        lbx[2: n_states * (N + 1): n_states] = -ca.inf  # theta lower bound

        am = hmqr5_agl_map() # 实际的机械臂关节角限制
        lbx[3: n_states * (N + 1): n_states] = am.limit[0][0]  # q1 lower bound
        lbx[4: n_states * (N + 1): n_states] = am.limit[1][0]  # q2 lower bound
        lbx[5: n_states * (N + 1): n_states] = am.limit[2][0]  # q3 lower bound
        lbx[6: n_states * (N + 1): n_states] = am.limit[3][0]  # q4 lower bound
        lbx[7: n_states * (N + 1): n_states] = am.limit[4][0]  # q5 lower bound
        lbx[8: n_states * (N + 1): n_states] = am.limit[5][0]  # q6 lower bound

        ubx[0: n_states * (N + 1): n_states] = ca.inf  # X upper bound
        ubx[1: n_states * (N + 1): n_states] = ca.inf  # Y upper bound
        ubx[2: n_states * (N + 1): n_states] = ca.inf  # theta upper bound

        ubx[3: n_states * (N + 1): n_states] = am.limit[0][1]  # q1 upper bound
        ubx[4: n_states * (N + 1): n_states] = am.limit[1][1]  # q2 upper bound
        ubx[5: n_states * (N + 1): n_states] = am.limit[2][1]  # q3 upper bound
        ubx[6: n_states * (N + 1): n_states] = am.limit[3][1]  # q4 upper bound
        ubx[7: n_states * (N + 1): n_states] = am.limit[4][1]  # q5 upper bound
        ubx[8: n_states * (N + 1): n_states] = am.limit[5][1]  # q6 upper bound

        lbx[n_states * (N + 1):] = self.v_arm_min  # u lower bound for all U
        ubx[n_states * (N + 1):] = self.v_arm_max  # u upper bound for all U
        lbx[n_states * (N + 1): -1: n_controls] = self.v_min  # u upper bound for v
        lbx[n_states * (N + 1) + 1: -1: n_controls] = self.v_min  # u upper bound for w
        ubx[n_states * (N + 1): -1: n_controls] = self.v_max  # u upper bound for v
        ubx[n_states * (N + 1) + 1: -1: n_controls] = self.v_max  # u upper bound for w

        self.args = {
            'lbg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints lower bound
            'ubg': ca.DM.zeros((n_states * (N + 1), 1)),  # constraints upper bound
            'lbx': lbx,
            'ubx': ubx
        }
    
    # 如果你愿意，可以传入一段path
    def go_target_pose(self, pose_msg):
        print(f"Target:\n{pose_msg}")
        p, ori = pose_msg.position, pose_msg.orientation
        args = [p.x, p.y, p.z, ori.x, ori.y, ori.z, ori.w]
        for k in range(self.N + 1):
            for j in range(7):
                arg_p_arm[k * 7 + j] = args[j]
        return arg_p_arm

    def set_reference(self, p):
        self.args['p'] = p
    
    def set_x0(self, X0, u0):
        self.args['x0'] = ca.vertcat(
                ca.reshape(X0, self.n_states * (self.N + 1), 1),
                ca.reshape(u0, self.n_controls * self.N, 1)
                )
    def get_states_and_control(self):
        sol = self.solver(
                x0=self.args['x0'],
                lbx=self.args['lbx'],
                ubx=self.args['ubx'],
                lbg=self.args['lbg'],
                ubg=self.args['ubg'],
                p=self.args['p']
            )
        u = ca.reshape(sol['x'][self.n_states * (self.N + 1):], self.n_controls, self.N)
        X0 = ca.reshape(sol['x'][: self.n_states * (self.N + 1)], self.n_states, self.N + 1)
        return X0, u 
    
    @staticmethod
    def DM2Arr(dm):
        return np.array(dm.full())

# Forward integration
def shift_timestep(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:, 0])
    next_state = ca.DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )
    return t0, next_state, u0

def ctrl_rob(cfg):
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
    rob.world_frame_set(desired_world_positions)
    rob.arm_ctrl(desired_arm_joint_angles)

if __name__=='__main__':
    mpc = MPC()
    rospy.Subscriber("/mobile_manipulator_mpc_target", Pose, mpc.go_target_pose)
    rate = rospy.Rate(50)
    # 
    t0 = 0
    t = ca.DM(t0)

    u0 = ca.DM.zeros((mpc.n_controls, mpc.N))  # initial control
    X0 = ca.repmat(mpc.state_init, 1, mpc.N + 1)  # initial state full

    mpc_iter = 0
    arg_p_arm_mat = ca.DM.zeros(7, (mpc.N + 1))  # end effector trajectory

    # Visualize in RVIZ
    ee_planning_path_pub = rospy.Publisher('ee_planning_trajectory', Path, queue_size=50)
    base_predict_path_pub = rospy.Publisher('base_predict_trajectory', Path, queue_size=50)
    ee_real_path_pub = rospy.Publisher('ee_real_trajectory', Path, queue_size=50)
    rob_sphere_pub = rospy.Publisher("rob_sphere_pub", MarkerArray, queue_size=10)

    arg_p_arm = ca.DM.zeros(7 * (mpc.N + 1))
    # 末端执行器的初始目标位姿
    pose_init = [0,0,1,0,0.707,0,0.707] # [x,y,z, ox,oy,oz,ow]
    for k in range(mpc.N + 1):
            for j in range(7):
                arg_p_arm[k * 7 + j] = pose_init[j]
    state_now = mpc.state_init
    while(not rospy.is_shutdown()):
            main_loop = time.time()  # return time in sec
            mpc_iter = 0
            door_info = ca.DM([1.2, -0.65, 1.2, 0])
            while (ca.norm_2(mpc.state_init - mpc.state_target) > 1e-1):
                    p = ca.vertcat(
                            state_now,  # current state
                            mpc.state_target,  # target state
                            arg_p_arm
                        )
                    mpc.set_reference(p)
                    mpc.set_x0(X0, u0)
                    X0, u0 = mpc.get_states_and_control()
                    # 在真实系统里，直接将u0[:,0]应用于机器人控制
                    # 并从实际机器人中获得机器人的状态
                    t0, state_now, u0 = shift_timestep(mpc.step_horizon, t0, state_now, u0, mpc.f)
                    ctrl_rob(state_now.T)
                    # Visualize in RVIZ
                    x_arm_now, R_arm_now = ur5_dh.RobFki(state_now)
                    vl.DrawBasePredictPath(base_predict_path_pub, X0, 'odom')
                    vl.PubEndEffectorRealTrajectory(ee_real_path_pub, x_arm_now)
                    p_all = vl.DrawRobSphere(rob_sphere_pub, state_now[:, 0], scale=0.1)
            rate.sleep()
            print("hh")
