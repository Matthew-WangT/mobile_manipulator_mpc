"""
实现基于casadi的机器人旋量理论（部分）
"""
import casadi as ca
import numpy as np


# : list 只是标注，不检查
def BuildSXMatrix(matrix_list: list):
    """
    直接用SX(list)无法生成带符号量的matrix/vector，可以使用逻辑判断表达式等
    Args:
        matrix_list: 包含sx符号量的二维list格式
    Returns: sx矩阵
    """
    assert isinstance(matrix_list, list)
    if not isinstance(matrix_list[0], list):  # 如果不是list就说明是向量
        rows = len(matrix_list)
        sx_matrix = ca.SX.zeros(rows, 1)
        for i in range(rows):
            sx_matrix[i, 0] = matrix_list[i]
        return sx_matrix
    rows, cols = len(matrix_list), len(matrix_list[0])
    sx_matrix = ca.SX.zeros(rows, cols)
    # print(sx_matrix)
    for i in range(rows):
        for j in range(cols):
            sx_matrix[i, j] = matrix_list[i][j]
    return sx_matrix


def TransToRp(T: ca.SX):
    # 确认T是SX
    assert isinstance(T, ca.SX)
    R, p = T[0:3, 0:3], T[0: 3, 3]
    return R, p


def VecToso3(omg):
    # 不用验证omg类型
    so3mat_list = [[0, -omg[2], omg[1]],
                   [omg[2], 0, -omg[0]],
                   [-omg[1], omg[0], 0]]
    so3mat = BuildSXMatrix(so3mat_list)
    return so3mat


def VecTose3(V):
    so3mat_tmp = VecToso3([V[0], V[1], V[2]])
    se3mat_tmp = ca.horzcat(so3mat_tmp, ca.vertcat(V[3], V[4], V[5]))
    se3mat_tmp = ca.vertcat(se3mat_tmp, ca.SX.zeros(1, 4))
    return se3mat_tmp


def so3ToVec(so3mat):
    return ca.vertcat(so3mat[2, 1], so3mat[0, 2], so3mat[1, 0])


def Normalize(V):
    # Normalizes a vector
    return V / ca.norm_2(V)


def AxisAng3(expc3):
    return Normalize(expc3), ca.norm_2(expc3)


def AxisAngToQuaternion(omega, theta):
    """
    将轴角转换为四元数
    :param omega: 旋转矢量
    :param theta: 旋转角度
    :return: 四元数（x,y,z,w）
    """
    q = ca.vertcat(omega * ca.sin(theta / 2), ca.cos(theta / 2))
    return q


def RotationToQuaternion(R):
    """
    旋转矩阵变换为四元数。
    :param R: 旋转矩阵
    :return: 四元数（x,y,z,w）
    """
    omega, theta = MatrixLog3(R)
    q = AxisAngToQuaternion(omega, theta)
    return q


def QuaternionError(q1, q2):
    """
    计算四元数之间的误差。
    :param q1: 四元数1
    :param q2: 四元数2
    :return: 误差
    """
    tmp = VecToso3(q2)
    return q1[3] * q2[:3] - q2[3] * q1[:3] + tmp @ q1[:3]


def RotationErrorByQuaternion(R1, R2):
    """
    通过四元数计算旋转矩阵之间的误差。
    :param R1: 旋转矩阵1
    :param R2: 旋转矩阵2
    :return: 误差
    """
    q1 = RotationToQuaternion(R1)
    q2 = RotationToQuaternion(R2)
    return QuaternionError(q1,q2)


def MatrixExp6(se3mat):
    """Computes the matrix exponential of an se3 representation of
    exponential coordinates

    :param se3mat: A matrix in se3
    :return: The matrix exponential of se3mat
    """
    pass


def MatrixExp3(w, theta):
    """ 旋转向量(角轴)映射到旋转矩阵。
        即罗德里格斯公式。
        :param w_hat: in R_3X1 (转轴)
        :param theta: in R_1, 转角
        :return: The matrix exponential of so3mat
        """
    w_hat = VecToso3(w)
    print(w_hat)
    R = ca.DM.eye(3) + ca.sin(theta) * w_hat + w_hat @ w_hat * (1 - ca.cos(theta))
    return R


def trace(T):
    """
    求矩阵的迹(方阵)
    :param T: 矩阵
    :return: 矩阵的迹
    """
    dim = T.shape[0]
    trace_R = 0
    for i in range(dim):
        trace_R += T[i, i]
    return trace_R


def MatrixLog3(R):
    """
    旋转矩阵映射到旋转向量(角轴)，即对数映射。
    :param R: 旋转矩阵
    :return:  角轴
    """
    # assert isinstance(R, ca.SX)
    theta = ca.arccos((trace(R) - 1) / 2)
    omega = ca.vertcat(R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]) / (2 * ca.sin(theta))
    return omega, theta


def Adjoint(T: ca.SX):
    R, p = TransToRp(T)
    Ad = ca.SX.zeros(6, 6)
    Ad[0:3, 0:3] = R
    Ad[3:6, 3:6] = R
    Ad[3:6, 0:3] = ca.mtimes(VecToso3(p), R)
    return Ad


def JacobianBody(Blist: np.ndarray, thetalist: ca.SX):
    assert isinstance(thetalist, ca.SX)
    Jb = ca.SX(Blist)
    T = ca.SX.eye(4)
    for i in range(thetalist.size()[0] - 2, -1, -1):  # 4->0
        T = ca.mtimes(T, MatrixExp6(VecTose3(np.array(Blist)[:, i + 1] * -thetalist[i + 1])))
        Jb[:, i] = ca.mtimes(Adjoint(T), np.array(Blist)[:, i])
    return Jb


def FKinBody(M: np.ndarray, Blist: np.ndarray, thetalist: ca.SX):
    T = ca.SX(M)
    for i in range(thetalist.size()[0]):
        T = ca.mtimes(T, MatrixExp6(VecTose3(np.array(Blist)[:, i] * thetalist[i])))
    return T


if __name__ == '__main__':
    # 伴随
    # w, theta = [1, 0, 0], 0.6981317
    # w, t = [0.9748485, 0.2059039, 0.0852882], 0.8035727  # x45, y30, z0
    # w, t = [0.3574067, -0.3574067, 0.8628562], 1.7177715  # x45, y0, z90
    # w, t = [1, 0, 0], 0.  # x 0 deg
    w, t = [1, 0, 0], 0.5235988  # x 30 deg
    # w, t = [1, 0, 0], 0.9238795  # x 45 deg
    T = MatrixExp3(w, t)
    print(T)
    omega, theta = MatrixLog3(T)
    print(omega, theta)
    print(omega * theta)
    Q = AxisAngToQuaternion(omega, theta)
    print("Q: ", Q)
    # print(RotationToQuaternion(T))
    Q2 = np.array([0.3826834, 0, 0, 0.9238795])  # x 45 deg
    print("Q2:", Q2)
    print("Error: ", QuaternionError(Q, Q2))
    print("Error: ", ca.norm_2(QuaternionError(Q, Q2)))
