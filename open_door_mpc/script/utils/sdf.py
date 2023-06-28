import numpy as np
import casadi as ca


def length(a):
    return np.sqrt(a[0] ** 2 + a[1] ** 2)

def lengthCasadi(a):
    return ca.sqrt(a[0] ** 2 + a[1] ** 2)


def myCircleSDF(p, obs, r):
    return np.sqrt((p[0] - obs[0]) ** 2 + (p[1] - obs[1]) ** 2) - r


def clamp(h, min, max):
    if h > max:
        return max
    elif h < min:
        return min
    else:
        return h


def clampCasadi(h, min, max):
    return ca.if_else(h > max, max, ca.if_else(h < min, min, h))


# def clamp_casadi(h, min, max):
#     return ca.if_else(h>max, max, ca.if_else(h<))


def myLineSDF(p, a, b):
    # pa表示a点指向p点的向量， ba表示a点指向b点的向量
    pa = p - a
    ba = b - a
    # h表示pa在ba上投影的长度占ba长度的比例，限定到[0,1]
    h = clamp(pa.dot(ba) / ba.dot(ba), 0.0, 1.0)
    # ba*h 是以a为起点，方向与ba相同，长度等于pa在ba方向上投影的长度的向量
    # pa视为斜边，ba*h是直角边，那么pa - ba*h则是另一条直角边，也就是从p点做垂线垂直于ab，显然该垂线的长度就是所求最短距离
    return length(pa - ba * h)


def myLineSdfCasadi(p, a, b):
    # pa表示a点指向p点的向量， ba表示a点指向b点的向量
    pa = p - a
    ba = b - a
    # h表示pa在ba上投影的长度占ba长度的比例，限定到[0,1]
    # h = clamp(pa.dot(ba) / ba.dot(ba), 0.0, 1.0)
    h = clampCasadi(ca.dot(pa, ba) / ca.dot(ba, ba), 0.0, 1.0)

    # ba*h 是以a为起点，方向与ba相同，长度等于pa在ba方向上投影的长度的向量
    # pa视为斜边，ba*h是直角边，那么pa - ba*h则是另一条直角边，也就是从p点做垂线垂直于ab，显然该垂线的长度就是所求最短距离
    return lengthCasadi(pa - ba * h)


def doorSDF(p):
    a = np.array([2, 10])
    b = np.array([2, 2])
    c = np.array([2, 0])
    d = np.array([2, -10])
    mid_door_y = ((b + c) / 2)[1]
    if p[1] > mid_door_y:
        return myLineSDF(p, a, b)
    else:
        return myLineSDF(p, c, d)


def doorSDFCasadi(p):
    a = np.array([2, 10])
    b = np.array([2, 1])
    c = np.array([2, 0])
    d = np.array([2, -10])
    return myLineSdfCasadi(p, a, b) + myLineSdfCasadi(p, c, d)
    # mid_door_y = ((b + c) / 2)[1]
    # return ca.if_else(p[1] > mid_door_y, myLineSdfCasadi(p, a, b), myLineSdfCasadi(p, c, d))

def doorRBFCasadi(p, rob_diam, epi_obs=1):
    a = np.array([2, 10])
    b = np.array([2, 1])
    c = np.array([2, 0])
    d = np.array([2, -10])
    h1 = myLineSdfCasadi(p, a, b) - rob_diam / 2
    h2 = myLineSdfCasadi(p, c, d) - rob_diam / 2
    h1 = ca.if_else(h1 > 0.001, h1, 0.001)
    h2 = ca.if_else(h2 > 0.001, h2, 0.001)
    # rbf_base_out = ca.if_else(h < 3, -epi_obs * ca.log(h), 0) # 这儿分段会导致求解速度变慢
    rbf1 = -epi_obs * ca.log(h1)
    rbf2 = -epi_obs * ca.log(h2)

    return rbf1 + rbf2

def doorGradientCasadi(p):
    px1 = ca.vertcat(p[0]-0.1, p[1])
    px2 = ca.vertcat(p[0]+0.1, p[1])
    grad_x = (doorSDFCasadi(px2) - doorSDFCasadi(px1))/0.2
    py1 = ca.vertcat(p[0], p[1]-0.1)
    py2 = ca.vertcat(p[0], p[1]+0.1)
    grad_y = (doorSDFCasadi(py2) - doorSDFCasadi(py1))/0.2
    # grad_x = 0
    grad_x *= 4
    grad_y *= 4
    return ca.vertcat(grad_x, grad_y)



def queryMap(p, ori, step):
    op = p - ori
    print("op: ", op)
    d_n = op / step
    # print("dn: ", d_n)
    # biLinearInterpolation(d_n, doorSDF)
    return d_n


def biLinearInterpolation(p, f):
    Q11 = np.floor(p).astype(dtype=np.int8)
    Q22 = np.ceil(p).astype(dtype=np.int8)
    x1, x2 = Q11[0], Q22[0]
    y1, y2 = Q11[1], Q22[1]
    Q21 = np.array([Q22[0], Q11[1]])
    Q12 = np.array([Q11[0], Q22[1]])
    # print(f"p1:{Q11}")
    # print(f"p2:{Q22}")
    # print(f"f(Q11):{f(Q11)}")
    # print(f"f(Q22):{f(Q22)}")
    f_R1 = (x2 - p[0]) / (x2 - x1) * f(Q11) + (p[0] - x1) / (x2 - x1) * f(Q21)
    f_R2 = (x2 - p[0]) / (x2 - x1) * f(Q12) + (p[0] - x1) / (x2 - x1) * f(Q22)
    # print("f_R1:", f_R1)
    # print("f_R2:", f_R2)
    f_P = (y2 - p[1]) / (y2 - y1) * f_R1 + (p[1] - y1) / (y2 - y1) * f_R2
    gradient = np.array([f(Q21) - f(Q11), (f(Q12) - f(Q11))])
    return f_P, gradient


def biLinearInterpolationCasadi(p, f):
    Q11 = np.floor(p).astype(dtype=np.int8)
    Q22 = np.ceil(p).astype(dtype=np.int8)
    x1, x2 = Q11[0], Q22[0]
    y1, y2 = Q11[1], Q22[1]
    Q21 = np.array([Q22[0], Q11[1]])
    Q12 = np.array([Q11[0], Q22[1]])
    # print(f"p1:{Q11}")
    # print(f"p2:{Q22}")
    # print(f"f(Q11):{f(Q11)}")
    # print(f"f(Q22):{f(Q22)}")
    f_R1 = (x2 - p[0]) / (x2 - x1) * f(Q11) + (p[0] - x1) / (x2 - x1) * f(Q21)
    f_R2 = (x2 - p[0]) / (x2 - x1) * f(Q12) + (p[0] - x1) / (x2 - x1) * f(Q22)
    # print("f_R1:", f_R1)
    # print("f_R2:", f_R2)
    f_P = (y2 - p[1]) / (y2 - y1) * f_R1 + (p[1] - y1) / (y2 - y1) * f_R2
    gradient = np.array([f(Q21) - f(Q11), (f(Q12) - f(Q11))])
    return f_P, gradient


def sdf_map_func(p):
    i, j = p[0], p[1]
    return sdf_map[i, j]


def draw_circle_sdf(fig, ax, min_scale, max_scale, obs_coord, obs_diam, N=50, use_pcolor=False):
    x = np.linspace(min_scale, max_scale, N)
    y = np.linspace(min_scale, max_scale, N)
    xx, yy = np.meshgrid(x, y)
    z = myCircleSDF([xx, yy], obs_coord, r=obs_diam / 2)
    if use_pcolor:
        cb1 = ax.pcolormesh(xx, yy, z)
        fig.colorbar(cb1, ax=ax)
    else:
        cb = ax.contourf(x, y, z, cmap='RdBu')
        cs = ax.contour(x, y, z, colors='k')
        ax.clabel(cs, inline=True)
        fig.colorbar(cb, ax=ax)
    ax.set_xlabel('x')
    ax.set_ylabel('y')


if __name__ == "__main__":
    p = ca.SX.sym('p', 2)
    print(doorSDFCasadi(p))

    a = np.array([1, 2])
    b = np.array([1, 1])

    ori = np.array([0, -1])
    dl = np.array([1, 1]) * 4
    step = 0.1
    x = np.linspace(ori[0], ori[0] + dl[0], int(dl[0] / step) + 1)
    y = np.linspace(ori[1], ori[1] + dl[1], int(dl[1] / step) + 1)
    print("x:", x)
    print("y:", y)
    sdf_map = np.zeros(shape=(x.shape[0], y.shape[0]))
    print(x.shape[0])
    print(y.shape[0])
    # p = np.array([0, 0])
    for i in range(x.shape[0]):
        for j in range(y.shape[0]):
            p = np.array([x[i], y[j]])
            sdf_map[i, j] = doorSDF(p)
            # sdf_map[i, j] = myLineSDF(p, a, b)
    query_p = np.array([0.11, 0.11])
    print("query_p: ", doorSDF(query_p))
    # print(sdf_map)
    # d_n = queryMap(query_p, ori, 0.1)
    dn = queryMap(query_p, ori, 0.1)
    dist, grad = biLinearInterpolation(dn, sdf_map_func)
    print("query map biLinearInterpolation: ", dist)
    print("grad:", grad)
    dn = np.floor(dn).astype(np.int8)
    print("query map: ", sdf_map[dn[0], dn[1]])

    import matplotlib.pyplot as plt

    # plt.quiver()

    # u, v = np.meshgrid(x, y)
    u, v = np.gradient(sdf_map)
    for i in range(0, len(x), 5):
        for j in range(0, len(y), 5):
            plt.quiver(x[i], y[j], u[i][j], v[i][j], alpha=0.4)
    # plt.Line2D(a, b)
    for i in range(len(x)):
        for j in range(len(y)):
            s = sdf_map[i, j] * 10
            rc = s / 25
            s *= 2
            if rc > 1.0:
                rc = 1.0
            # print(rc)
            plt.scatter(x[i], y[j], color=(rc, 1 - rc, 0), s=s)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('sdf')
    plt.show()
# print(doorSDF(p))
