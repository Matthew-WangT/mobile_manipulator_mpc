##!/home/matthew/anaconda3/envs/normal_env/bin/python3
##!/home/matthew/anaconda3/envs/normal_env/bin/python3
# -*- coding: utf-8 -*
import numpy as np
import casadi as ca
import pinocchio as pin
'''
# L = [0.104, 0.35, 0.2253, 0.07, 0.049, 0.1]
L = [0.104, 0.35, 0.2253, 0.07, 0.049, 0.1]
alpha1, alpha4, alpha5 = -np.pi / 2, np.pi / 2, -np.pi / 2
d1 = L[0]
a2, a3, a4, a5, a6 = -L[1], L[2], L[3], L[4], L[5]
# phi2, phi3, phi4 = -0.2257, 0.2257, -np.pi / 2
phi3, phi4, phi5 = -0.2257, 0.2257, -np.pi/2
phi3, phi4, phi5 = -0.2257, 0.2257, 0
phi_s = ca.SX.sym('phi', 6)
# alpha, a, d, phi
dh_list = np.array([[alpha1, 0, d1, 0],
                    [0, a2, 0, 0],
                    [0, a3, 0, phi3],
                    [alpha4, a4, 0, phi4],
                    [alpha5, a5, 0, phi5],
                    [0, a6, 0, 0]])
'''
L = [0.103, 0.35, 0.2253, 0.07, 0.2, 0.1]
# L = [0.104, 0.35, 0.2253, 0.07, 0.049, 0.1]
alpha1, alpha4, alpha5 = -np.pi / 2, np.pi / 2, -np.pi / 2
d1 = L[0]
a2, a3, a4, a5, a6 = -L[1], L[2], L[3], L[4], L[5]
phi3, phi4, phi5, phi6 = -0.2257, 0.2257, -np.pi / 2, np.pi / 2
phi_s = ca.SX.sym('phi', 6)
# alpha, a, d, phi
dh_list = np.array([[alpha1, 0, d1, 0],
                    [0, a2, 0, 0],
                    [0, a3, 0, phi3],
                    [alpha4, a4, 0, phi4],
                    [alpha5, 0, 0, phi5],
                    [0, 0, a5, phi6]])
# urdf_model_path = "/home/matthew/open_door_ws/src/simulation/ur5_description/urdf/ur5_robot.urdf"
# urdf_model_path = "/home/matthew/open_door_ws/src/simulation/ur5_description/urdf/ur5.urdf"
# ur5_model = pin.buildModelFromUrdf(urdf_model_path)
# print("model name:", ur5_model.name)
# print("ur5_model nq:", ur5_model.nq)
# # print("ur5_model:", ur5_model)
# ur5_data = ur5_model.createData()
#
#
# def Ur5Rena(q, v, a):
#     tau = pin.rnea(ur5_model, ur5_data, q, v, a)
#     return tau


def buildTij(dh, theta):
    alpha, a, d, phi_bias = dh
    phi = theta + phi_bias
    T_a = ca.DM.eye(4)
    T_a[0, 3] = a
    T_d = ca.DM.eye(4)
    T_d[2, 3] = d
    Rzt = ca.vertcat(
        ca.horzcat(ca.cos(phi), -ca.sin(phi), 0, 0),
        ca.horzcat(ca.sin(phi), ca.cos(phi), 0, 0),
        ca.horzcat(0, 0, 1, 0),
        ca.horzcat(0, 0, 0, 1))
    Rxa = ca.vertcat(
        ca.horzcat(1, 0, 0, 0),
        ca.horzcat(0, ca.cos(alpha), -ca.sin(alpha), 0),
        ca.horzcat(0, ca.sin(alpha), ca.cos(alpha), 0),
        ca.horzcat(0, 0, 0, 1))
    # 等价
    # T_ij = T_d @ Rzt @ T_a @ Rxa
    # T_ij = Rzt @ T_d @ T_a @ Rxa
    T_ij = Rzt @ T_d @ Rxa @ T_a

    return T_ij


def Z1_FK(idx):
    """
    :param idx: joint idx: 1-6
    :return:
    """
    assert 1 <= idx <= 6
    T = ca.DM.eye(4)
    for i in range(idx):
        Ti = buildTij(dh_list[i], phi_s[i])
        T = T @ Ti
    return T


def Z1Fki(q, idx):
    """
    :param idx: joint idx: 1-6
    :return:
    """
    assert 1 <= idx <= 6
    T = ca.DM.eye(4)
    for i in range(idx):
        Ti = buildTij(dh_list[i], q[i])
        T = T @ Ti
    return T


def RobZ1Fki(st, idx=6, z_b2a=0.56, v2=False):
    x_base_xy, phi = st[:2], st[2]
    x_base = ca.vertcat(x_base_xy[0], x_base_xy[1], z_b2a)
    q = st[3:]
    if v2:
        q = st[6:12]
    T_tip = Z1Fki(q, idx)
    p_tip = ca.vertcat(T_tip[0, 3], T_tip[1, 3], T_tip[2, 3])
    R_be = T_tip[:3, :3]
    R_wb = ca.vertcat(
        ca.horzcat(ca.cos(phi), -ca.sin(phi), 0),
        ca.horzcat(ca.sin(phi), ca.cos(phi), 0),
        ca.horzcat(0, 0, 1)
    )
    x_a = R_wb @ p_tip
    rhs_fk = x_base + x_a
    R_we = R_wb @ R_be
    return rhs_fk, R_we


def Z1_Fa(q, dq, ddq):
    J = buildJacobian()
    Jq = J(q)
    # TODO: un-finished
    # a_ee = Jq @ ddq + dJq @ dq
    a_ee = Jq @ ddq
    return a_ee


def buildJacobian():
    Ji = []
    Zi = []
    Oi = []
    Zi.append(ca.DM([0, 0, 1]))
    Oi.append(ca.DM([0, 0, 0]))
    for i in range(6):
        T0i = Z1_FK(i + 1)
        Zi.append(T0i[0:3, 2])
        Oi.append(T0i[0:3, 3])
    # print('Oi[-1]:')
    # print(Oi[-1])
    for i in range(6):
        Jv = ca.cross(Zi[i], Oi[-1] - Oi[i])
        # print(f'Oi[{i}]: {Oi[i]}')
        Ji.append(ca.vertcat(Jv, Zi[i]))
    Ja = ca.horzcat(Ji[0], Ji[1], Ji[2], Ji[3], Ji[4], Ji[5])
    f_ja = ca.Function('f_fk', [phi_s], [Ja])
    # return Ja
    return f_ja


def build_fk_func():
    T = ca.DM.eye(4)
    for i in range(6):
        Ti = buildTij(dh_list[i], phi_s[i])
        T = T @ Ti
    f_fk = ca.Function('f_fk', [phi_s], [T])
    # print(T[0:3,3])
    # print(T)
    return f_fk


if __name__ == "__main__":
    f = build_fk_func()
    print("f:")
    print(f)
    list = np.array([[0, np.pi / 2, 0, 0, -np.pi / 2, 0]])
    list = np.array([[0, 0, 0, 0, 0, 0]])
    print("my_fk", f(list[0])[:3, 3].T)
    # Ja = buildJacobian()
    # print(Ja)
    # q = ca.SX.sym('q', 6)
    # print(Ja(q))
    # print(Ja(q).shape)
    # d_ja = ca.jacobian(Ja(q), q)
    # print(d_ja.shape)
    # ca.diff(Ja(q), 2)
    # q = np.zeros(6)
    # dq = np.zeros(6)
    # ddq = np.ones(6)
    # ee_a = Z1_Fa(q, dq, ddq)
    # print(ee_a)
