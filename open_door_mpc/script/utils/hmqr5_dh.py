##!/home/matthew/anaconda3/envs/normal_env/bin/python3
##!/home/matthew/anaconda3/envs/normal_env/bin/python3
# -*- coding: utf-8 -*
import numpy as np
import casadi as ca

alpha1, alpha4, alpha5 = -np.pi / 2, -np.pi / 2, np.pi / 2
# d1, d4, d5, d6 = 0.089159, 0.10915, 0.09465, 0.0823  # ur5
# d1, d4, d5, d6 = 0.114, 0.1147, 0.1135, 0.2087  # hmqr5
d1, d4, d5, d6 = 0.141, 0.1381, 0.123, 0.094  # hmqr5
d6 += 0.147  # gripper
# a2, a3 = 0.425, 0.39225 # ur5
a2, a3 = 0.3743, 0.343
phi_s = ca.SX.sym('phi', 6)
# alpha, a, d
dh_list = np.array([[alpha1, 0, d1],
                    [0, a2, 0],
                    [0, a3, 0],
                    [alpha4, 0, d4],
                    [alpha5, 0, d5],
                    [0, 0, d6]])


def buildTij(dh, phi):
    alpha, a, d = dh
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


def Hmqr5_FK(idx):
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


def Hmqr5Fki(q, idx):
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


def RobFki(st, idx=6, z_b2a=0.44, v2=False):
    x_base_xy, phi = st[:2], st[2]
    x_base = ca.vertcat(x_base_xy[0], x_base_xy[1], z_b2a)
    q = st[3:]
    if v2:
        q = st[6:12]
    # print("q:",q)
    T_tip = Hmqr5Fki(q, idx)
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


def Hmqr5_Fa(q, dq, ddq):
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
        T0i = Hmqr5_FK(i + 1)
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
    Ja = buildJacobian()
    print(Ja)
    q = ca.SX.sym('q', 6)
    print(Ja(q))
    print(Ja(q).shape)
    d_ja = ca.jacobian(Ja(q), q)
    print(d_ja.shape)
    ca.diff(Ja(q), 2)
    q = np.zeros(6)
    dq = np.zeros(6)
    ddq = np.ones(6)
