import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import casadi as ca
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from geometry_msgs.msg import WrenchStamped

import mr_casadi as mc
# from hmqr5_dh import RobFki
from ur5_dh import RobFki
from z1_dh import Z1Fki, RobZ1Fki


def DrawPath(path_pub, path, frame='odom', path_type=1, jump_step=3):
    """
    数据更新函数
    """
    path_record = Path()
    # 时间戳
    current_time = rospy.Time.now()
    if path_type:
        path_mat = ca.reshape(path, 7, -1)
        # 四元素转换
        for i in range(path_mat.shape[1]):
            # 配置姿态
            pose = PoseStamped()
            pose.header.stamp = current_time
            pose.header.frame_id = frame
            pose.pose.position.x = path_mat[0, i]
            pose.pose.position.y = path_mat[1, i]
            pose.pose.position.z = path_mat[2, i]
            pose.pose.orientation.x = path_mat[3, i]
            pose.pose.orientation.y = path_mat[4, i]
            pose.pose.orientation.z = path_mat[5, i]
            pose.pose.orientation.w = path_mat[6, i]

            # 配置路径
            path_record.header.stamp = current_time
            path_record.header.frame_id = frame
            path_record.poses.append(pose)

            # 路径数量限制
            if len(path_record.poses) > 1000:
                path_record.poses.pop(0)
    else:
        for i in range(path.shape[1]):
            item = jump_step * i
            if item >= path.shape[1]:
                break
            # 配置姿态
            pose = PoseStamped()
            pose.header.stamp = current_time
            pose.header.frame_id = frame
            pose.pose.position.x = path[0, item]
            pose.pose.position.y = path[1, item]
            pose.pose.position.z = path[2, item]
            pose.pose.orientation.x = path[3, item]
            pose.pose.orientation.y = path[4, item]
            pose.pose.orientation.z = path[5, item]
            pose.pose.orientation.w = path[6, item]

            # 配置路径
            path_record.header.stamp = current_time
            path_record.header.frame_id = frame
            path_record.poses.append(pose)

            # 路径数量限制
            if len(path_record.poses) > 1000:
                path_record.poses.pop(0)

    # 发布路径
    path_pub.publish(path_record)


global path_ee_real
global path_base_real
path_ee_real = Path()
path_base_real = Path()


def PubEndEffectorRealTrajectory(path_pub, pos, frame='odom', max_record=5e3):
    global path_ee_real
    # 时间戳
    current_time = rospy.Time.now()
    path_ee_real.header.stamp = current_time
    path_ee_real.header.frame_id = frame
    # 配置姿态
    pose = PoseStamped()
    current_time = rospy.Time.now()
    pose.header.stamp = current_time
    pose.header.frame_id = frame
    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    pose.pose.position.z = pos[2]
    # pose.pose.orientation.x = path[item][3]
    # pose.pose.orientation.y = path[ item][4]
    # pose.pose.orientation.z = path[ item][5]
    pose.pose.orientation.w = 1

    # 配置路径
    path_ee_real.poses.append(pose)

    # 路径数量限制
    if len(path_ee_real.poses) > max_record:
        path_ee_real.poses.pop(0)
    # 发布路径
    path_pub.publish(path_ee_real)


def PubBaseRealTrajectory(path_pub, pos, frame='odom', max_record=5e3):
    global path_base_real
    # 时间戳
    current_time = rospy.Time.now()
    path_base_real.header.stamp = current_time
    path_base_real.header.frame_id = frame
    # 配置姿态
    pose = PoseStamped()
    current_time = rospy.Time.now()
    pose.header.stamp = current_time
    pose.header.frame_id = frame
    pose.pose.position.x = pos[0]
    pose.pose.position.y = pos[1]
    # pose.pose.position.z = pos[2]
    pose.pose.orientation.w = 1

    # 配置路径
    path_base_real.poses.append(pose)

    # 路径数量限制
    if len(path_base_real.poses) > max_record:
        path_base_real.poses.pop(0)
    # 发布路径
    path_pub.publish(path_base_real)


def PubEndEffectorVelocity(vel_pub, pos, vel, frame='odom'):
    # 时间戳
    line = Marker()
    line.id = 0
    line.header.frame_id = 'odom'
    line.header.stamp = rospy.Time.now()
    # line.ns = 'rob'
    # line.pose.orientation.w = 1.0
    # line.pose.position.x = pos[0]
    # line.pose.position.y = pos[1]
    # line.pose.position.z = pos[2]

    # line.action = Marker.ADD
    line.type = Marker.LINE_STRIP

    line.color.a = 0.7
    line.scale.x = 0.1  # line width
    line.color.b = 0.8
    scale = 10
    p1 = Point()
    p2 = Point()
    p1.x = pos[0]
    p1.y = pos[1]
    p1.z = pos[2]
    line.points.append(p1)
    p2.x = pos[0] + vel[0] * scale
    p2.y = pos[1] + vel[1] * scale
    p2.z = pos[2] + vel[2] * scale
    line.points.append(p2)
    vel_pub.publish(line)


def DrawBasePredictPath(path_pub, predict_states, frame='odom'):
    # print("predict_states shape:", predict_states.shape)
    # print("predict_states:", predict_states)
    predict_states_base = predict_states[:3, :]
    N = predict_states_base.shape[1]
    path = np.zeros(shape=(7, N))
    for i in range(N):
        path[0, i] = predict_states_base[0, i]
        path[1, i] = predict_states_base[1, i]
        path[2, i] = 0.06
        path[6, i] = 1.0
    # print("path shape: ", path.shape)
    # print("path: ", path)
    DrawPath(path_pub, path, frame=frame, path_type=0, jump_step=1)


def DrawEePredictPath(path_pub, predict_states, is_z1=False, frame='odom'):
    N = predict_states.shape[1]
    path = np.zeros(shape=(7, N))
    for i in range(N):
        if is_z1:
            pre_pos, pre_R = RobZ1Fki(predict_states[:, i], 6)
        else:
            pre_pos, pre_R = RobFki(predict_states[:, i], 6)
        r_quat = mc.RotationToQuaternion(pre_R)
        path[0, i] = pre_pos[0]
        path[1, i] = pre_pos[1]
        path[2, i] = pre_pos[2]
        path[3, i] = r_quat[0]
        path[4, i] = r_quat[1]
        path[5, i] = r_quat[2]
        path[6, i] = r_quat[3]
    # print("path shape: ", path.shape)
    # print("path: ", path)
    DrawPath(path_pub, path, frame=frame, path_type=0, jump_step=1)


def DrawForce(pub, values):
    msg = WrenchStamped()

    msg.header.frame_id = "FT_link"
    msg.header.stamp = rospy.Time.now()
    msg.wrench.force.x = float(values[0])
    msg.wrench.force.y = float(values[1])
    msg.wrench.force.z = float(values[2])

    msg.wrench.torque.x = float(values[3])
    msg.wrench.torque.y = float(values[4])
    msg.wrench.torque.z = float(values[5])


"""
# marker type:
uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
"""


def BuildMarker(pos, id, scale=0.3, rgba=[0.8, 0, 0, 0.4]):
    m = Marker()
    m.id = id
    m.header.frame_id = 'odom'
    m.header.stamp = rospy.Time.now()
    # m.ns = 'rob'
    m.pose.orientation.w = 1.0
    m.pose.position.x = pos[0]
    m.pose.position.y = pos[1]
    m.pose.position.z = pos[2]
    # m.pose.
    # m.action = Marker.ADD
    m.type = Marker.SPHERE

    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale

    m.color.r = rgba[0]
    m.color.g = rgba[1]
    m.color.b = rgba[2]
    m.color.a = rgba[3]
    return m


def BuildCylinderMarker(pos, id, scale=0.3, a=0.4):
    m = Marker()
    m.id = id
    m.header.frame_id = 'odom'
    m.header.stamp = rospy.Time.now()
    # m.ns = 'rob'
    m.pose.orientation.w = 1.0
    m.pose.position.x = pos[0]
    m.pose.position.y = pos[1]
    m.pose.position.z = pos[2]
    # m.action = Marker.ADD
    m.type = Marker.CYLINDER

    m.color.a = a
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = 0.3

    m.color.r = 0.8
    return m


def DrawRobSphere(pub, state, scale=0.3, is_z1=False, v2=False):
    if v2:
        q = state[6:12]
    # print("q:", q.T)
    maker_array = MarkerArray()
    p_base = np.array([state[0], state[1], 0.23])
    p_all = p_base.copy()
    marker_base = BuildCylinderMarker(p_base, id=0, scale=0.6)
    maker_array.markers.append(marker_base)
    for i in range(6):
        if is_z1:
            p_ew = RobZ1Fki(state, i + 1)[0]
        else:
            # p_ew = RobFki(state, i + 1, z_b2a=0.41)[0]  # Todo z_b2a
            # p_ew = RobFki(state, i + 1, z_b2a=0.44)[0]
            p_ew = RobFki(state, i + 1, z_b2a=0.55)[0]
        p_all = np.append(p_all, p_ew)
        rgba = [i/5.0, 0, 1-i/5.0, 0.5]
        tmpM = BuildMarker(p_ew, id=i + 1, scale=scale, rgba=rgba)
        maker_array.markers.append(tmpM)
    pub.publish(maker_array)
    return p_all


def DrawRobFrame(frame_pub, state, is_z1=False, frame='odom', v2=False):
    rob_frames = Path()
    maker_array = MarkerArray()
    p_base = np.array([state[0], state[1], 0.23])
    marker_base = BuildCylinderMarker(p_base, id=0, scale=0.6)
    maker_array.markers.append(marker_base)
    # 时间戳
    current_time = rospy.Time.now()
    rob_frames.header.stamp = current_time
    rob_frames.header.frame_id = frame
    for i in range(6):
        if is_z1:
            p_ew, R_ew = RobZ1Fki(state, i + 1)
        else:
            p_ew, R_ew = RobFki(state, i + 1)
        # 配置姿态
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame
        pose.pose.position.x = p_ew[0]
        pose.pose.position.y = p_ew[1]
        pose.pose.position.z = p_ew[2]
        tmp_qua = mc.RotationToQuaternion(R_ew)
        print("tmp_qua: ", tmp_qua)
        pose.pose.orientation.x = tmp_qua[0]
        pose.pose.orientation.y = tmp_qua[1]
        pose.pose.orientation.z = tmp_qua[2]
        pose.pose.orientation.w = tmp_qua[3]
        rob_frames.poses.append(pose)

    frame_pub.publish(rob_frames)


def DrawSingleSphere(pub, pos, scale=0.2, rgba=[0, 0.8, 0, 0.6]):
    ms = BuildMarker(pos, 0, scale=scale, rgba=rgba)
    pub.publish(ms)


if __name__ == '__main__':
    rospy.init_node("points_and_lines", anonymous=True)
    types = [Marker.POINTS, Marker.LINE_STRIP, Marker.LINE_LIST]
    # markers = [Marker() for _ in ['points', 'line_strip', 'line_list']]
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rob_sphere_pub = rospy.Publisher("rob_sphere_pub", MarkerArray, queue_size=10)
    rob_frame_pub = rospy.Publisher("rob_frame_pub", Path, queue_size=10)
    rate = rospy.Rate(10)
    i=0
    while not rospy.is_shutdown():
        # draw(pub)
        state = np.ones(15) * 0
        # state[6] = i*0.05
        # state[7] = i*0.01
        state[5] = -i*0.0001
        state[6] = -i*0.0001
        state[7] = -i*0.01
        state[8] = -i*0.01
        # DrawRobSphere(rob_sphere_pub, state, scale=0.05, is_z1=True)
        rate.sleep()
        DrawRobFrame(rob_frame_pub, state, True)
        # DrawRobFrame(rob_frame_pub, state, False)
        i += 1
