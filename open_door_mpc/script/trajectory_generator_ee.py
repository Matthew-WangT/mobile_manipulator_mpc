import numpy as np
import modern_robotics as mr
from scipy.spatial.transform import Rotation


def rotate_handle(T_se_before, hinge_p, handle_length, agl, d_agl, time, order='anticlockwise', handle_type='right'):
    n = round(agl / d_agl)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
    print(f"Steps for rotate: {n}")
    print(f"旋转方向：", order)
    print(f"把手朝向：", handle_type)
    print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
    pos = np.zeros(shape=(n, 4), dtype=float)  # (x,y,z,theta)
    r = handle_length
    theta = 0
    for i in range(n):
        pos[i][0] = hinge_p[0]
        if handle_type == 'right':
            if order == 'clockwise':  # 顺时针
                pos[i][1] = hinge_p[1] - r * np.cos(theta)
                pos[i][2] = hinge_p[2] - r * np.sin(theta)
                pos[i][3] = theta
            elif order == 'anticlockwise':  # 逆时针
                # 在世界坐标系中表达
                pos[i][1] = hinge_p[1] - r * np.cos(theta)
                pos[i][2] = hinge_p[2] + r * np.sin(theta)
                pos[i][3] = theta
        elif handle_type == 'left':
            if order == 'clockwise':  # 顺时针
                pos[i][1] = hinge_p[1] + r * np.cos(theta)
                pos[i][2] = hinge_p[2] + r * np.sin(theta)
                pos[i][3] = theta
            elif order == 'anticlockwise':  # 逆时针
                # 在世界坐标系中表达
                pos[i][1] = hinge_p[1] + r * np.cos(theta)
                pos[i][2] = hinge_p[2] - r * np.sin(theta)
                pos[i][3] = theta
        theta += d_agl
    # return pos
    #pos
    rotate_door_seq = pos
    n = rotate_door_seq.shape[0]
    # T_se_segment_rotate = [T_se_before[-1].copy()]
    # T_se_segment_rotate = [T_se_before[-1].copy()]
    for i in range(n):
        T_se_end_tmp = T_se_before[-1]
        # 绕着世界坐标系x轴负方向转agl角度
        # t = rotate_door_seq[i][3]
        t = d_agl
        if order == 'anticlockwise':
            t *= -1
        # print(f"theta[{i}]: {t}")
        r = Rotation.from_euler('xyz', [t, 0, 0], degrees=False)
        rm = r.as_matrix()
        # print("rm:", rm)
        rm_t = np.array([[rm[0][0], rm[0][1], rm[0][2], 0],
                         [rm[1][0], rm[1][1], rm[1][2], 0],
                         [rm[2][0], rm[2][1], rm[2][2], 0],
                         [0, 0, 0, 1]])
        T_se_end_tmp2 = np.dot(rm_t, T_se_end_tmp)
        T_se_end_tmp2[0][3] = rotate_door_seq[i][0]
        T_se_end_tmp2[1][3] = rotate_door_seq[i][1]
        T_se_end_tmp2[2][3] = rotate_door_seq[i][2]
        T_se_segment_tmp = mr.CartesianTrajectory(T_se_before[-1], T_se_end_tmp2, time, time/0.01, 3)
        T_se_before = np.append(T_se_before, T_se_segment_tmp, axis=0)
        # T_se_segment_rotate = np.append(T_se_segment_rotate, T_se_segment_tmp, axis=0)
    return T_se_before


def manipulate_door(T_se_before, step_time, R_door, action="pull", agl=np.pi/3, da=1*np.pi/180, door_type='right'):
    beta = 0.0    
    # R_door = 0.63 # Todo: edit it
    door_type_sign = 1 if door_type == 'left' else -1
    D0 = np.array([T_se_before[-1][0][3], T_se_before[-1][1][3] - R_door*door_type_sign, T_se_before[-1][2][3]])
    # R_door = abs(T_se_before[-1][1, 3])  # y_w
    R_tmp = T_se_before[-1][0:3, 0:3]
    # print(f"T_se_before[-1]: {T_se_before[-1]}")
    # print(f"R_tmp: {R_tmp}")
    # print(f"R_tmp shape: {R_tmp.shape}")
    print(f"pz: {T_se_before[-1][2][3]}")
    # time_4 = 0.2
    for i in range(round(agl / da)):
        beta += da
        act_type_sign = 1 if action == 'pull' else -1
        # print(f"act_type_sign: {act_type_sign}")
        # print(f"door_type_sign: {door_type_sign}")
        # Dht_x = D0[0] - R_door * np.sin(beta)
        Dht_x = D0[0] - R_door * np.sin(beta) * act_type_sign
        Dht_y = D0[1] + R_door * np.cos(beta) * door_type_sign
        Dht_z = D0[2]
        p_tmp = np.array([Dht_x, Dht_y, Dht_z])
        # rotate
        r = Rotation.from_euler('xyz', [0, 0, da*act_type_sign*door_type_sign], degrees=False)
        rm = r.as_matrix()
        R_tmp = np.dot(rm, R_tmp)
        # print(f"R_tmp: {R_tmp}")
        T_pull_door_tmp = mr.RpToTrans(R_tmp, p_tmp)
        # print(f"T_pull_door_tmp: {T_pull_door_tmp}")
        # print(f"T_pull_door_tmp shape: {T_pull_door_tmp.shape}")
        # print(f"T_se_before[-1] shape: {T_se_before[-1].shape}")
        T_se_segment_5_tmp = mr.CartesianTrajectory(T_se_before[-1], T_pull_door_tmp, step_time, step_time / 0.01, 3)
        # print('T_se_segment_5_tmp:', len(T_se_segment_5_tmp)) => 2
        T_se_before = np.append(T_se_before, T_se_segment_5_tmp, axis=0)
    return T_se_before, D0


def trajectory_generator_door(T_se_initial, T_sh_initial, T_he_grasp, T_he_standoff, action="push", handle_length=0.1, handle_type='right', door_type='left', order = 'clockwise'):
    """
    Args:
        T_se_initial:   机器人初始配置
        T_sh_initial:   把手初始位置
        T_he_grasp:     把手抓取   *姿态*
        T_he_standoff:  把手预抓取 *姿态*
    Returns:            配置空间序列（轨迹）
    """
    # ---------------------------------
    # segment 1: A trajectory to move the gripper from its initial configuration to a "standoff" configuration
    # a few cm above the hinge
    # represent the frame of end-effector when standoff in space frame
    T_se_standoff_initial = T_sh_initial.dot(T_he_standoff)
    # generate trajectory when approaching the standoff position in segment 1
    # print("T_se_initial", T_se_initial)
    # time_1, time_2, time_3, time_4 = 5., 2., 0.1, 0.2
    time_1, time_2, time_3, time_4 = 1.5, 0.5, 0.02, 0.02
    time_5 = time_2
    time_used = [time_1, time_2, time_3, time_4]
    print("sum time: ", sum(time_used))
    mani_steps = 10

    T_se_segment_1 = mr.CartesianTrajectory(T_se_initial, T_se_standoff_initial, Tf=time_1, N=time_1 / 0.01, method=3)
  
    # segment 2: A trajectory to move the gripper down to the grasp position
    T_se_grasp = T_sh_initial.dot(T_he_grasp)
    # generate trajectory when approaching the grasping position in segment 2
    # time_2 = 2.
    T_se_seg2 = mr.CartesianTrajectory(T_se_segment_1[-1], T_se_grasp, time_2, time_2 / 0.01, 3)
    # append the trajectory of segment 2 after segment 1，由此形成了一条完整的轨迹
    T_se_before = np.append(T_se_segment_1, T_se_seg2, axis=0)

    # segment 3: Closing of the gripper
    # append the trajectory of segment 3 by 63 times
    close_idx = len(T_se_before)
    for _ in np.arange(mani_steps):
        T_se_before = np.append(T_se_before, np.array([T_se_before[-1]]), axis=0)

    # segment 4: 转动末端效应器到指定位置
    # =============================
    handle_agl = 30*np.pi / 180.0
    d_agl = 1 * np.pi / 180
    # Notice plus or minus handle_length 
    handle_type_sign = 1 if handle_type=='right' else -1
    hinge_position = np.array([T_se_before[-1][0, 3], T_se_before[-1][1, 3] + handle_length*handle_type_sign, T_se_before[-1][2, 3]])
    T_se_before = rotate_handle(T_se_before, hinge_position, handle_length, handle_agl, d_agl, time_3, order=order, handle_type=handle_type)

    idx_mani_door = len(T_se_before)
    R_door = 0.63
    door_agl = 30*np.pi/180.
    T_se_before, door_hinge = manipulate_door(T_se_before, step_time=time_4, R_door=R_door, action=action, agl=door_agl, da=1*np.pi/180, door_type=door_type)
    # ========
    # segment 6: Opening of the gripper
    # append the trajectory of segment 3 by 63 times
    for _ in np.arange(mani_steps):
        T_se_before = np.append(T_se_before, np.array([T_se_before[-1]]), axis=0)

    # Rotate back
    # print(T_se_rotate)
    # T_se_rotate_back = np.flipud(T_se_segment_rotate) # deep copy
    # print("back:\n",T_se_rotate_back)
    # n_rotate = len(T_se_rotate_back)
    # last_pose = T_se_before[-1][:3,:3].copy()
    # print("door hinge:", door_hinge)
    # for i in range(len(T_se_rotate_back)):
    #     x = T_se_rotate_back[i][0, 3]
    #     y = T_se_rotate_back[i][1, 3]
    #     x2 = door_hinge[0] - R_door*np.sin(door_agl)
    #     y2 = door_hinge[1] + R_door*np.cos(door_agl)
    #     T_se_rotate_back[i][0, 3] = x2
    #     T_se_rotate_back[i][1, 3] = y2
    #     T_se_rotate_back[i][:3, :3] = last_pose
    #     print(f'(x,y)=({x},{y})')
    #     print(f'(x2,y2)=({x2},{y2})')
    #     print(f'(xt,yt)=({T_se_rotate_back[i][0, 3]},{T_se_rotate_back[i][1, 3]})')
    #     # print(x2,y2)
    # T_se_before = np.append(T_se_before, T_se_rotate_back, axis=0)
    open_idx = len(T_se_before)
    d_th = 5
    for i in range(int(30/d_th)):    
        T_up = T_se_before[-1].copy()
        T_up[2,3] = T_up[2,3] + 0.8*handle_length*np.sin(d_th*np.pi/180.)
        T_se_before = np.append(T_se_before, [T_up], axis=0)
        
    # 继续开一会
    dt = 0.02
    for i in range(int(0.1/dt)):    
        T_move = T_se_before[-1].copy()
        T_move[1,3] -= dt
        T_se_before = np.append(T_se_before, [T_move], axis=0)
    # assert 0

    # segment 7: A trajectory to move the gripper up to the grasp position
    # T_se_seg7 = mr.CartesianTrajectory(T_se_before[-1], T_se_before[-150], time_5, time_5 / 0.01, 3)
    # append the trajectory of segment 2 after segment 1，由此形成了一条完整的轨迹
    # T_se_before = np.append(T_se_before, T_se_seg7, axis=0)
    
    # config gripper
    whole_process_configs = np.ones(shape=(len(T_se_before)))  # 1 -> open, 0 -> close
    # whole_process_configs[close_idx:open_idx] = 0
    print("whole_process_configs shape:", whole_process_configs.shape)
    # steps_before_close = int((time_1 + time_2) / 0.01)
    # steps_for_open = 100
    for it in range(close_idx, open_idx): # close period
        whole_process_configs[it] = 0
    # for it in range(steps_before_close, len(T_se_before)-mani_steps-int(time_5 / 0.01)-steps_for_open):
    #     whole_process_configs[it] = 0
    print("whole_process_configs: ", whole_process_configs)

    return T_se_before, whole_process_configs, idx_mani_door
