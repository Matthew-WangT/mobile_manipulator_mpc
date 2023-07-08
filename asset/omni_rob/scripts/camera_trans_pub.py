#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

ts_pub = rospy.Publisher('ground_truth_ts_pub', TransformStamped, queue_size=1)
if __name__ == '__main__':
    rospy.init_node('camera_trans_pub_node')
    pub_frame_name = rospy.get_param('pub_frame_name')  # 获取参数
    # 创建tf的监听器
    listener = tf.TransformListener()
    ts = TransformStamped()
    while not rospy.is_shutdown():
        # 获取turtle1与turtle2坐标系之间的tf数据
        try:
            (trans, rot) = listener.lookupTransform('/odom', pub_frame_name, rospy.Time.now())  # this is right
            # print("trans:", trans)
            # print("rot:", rot)
            ts.header.stamp = rospy.Time.now()
            # ts.header.frame_id = 'odom'
            ts.header.frame_id = 'world'
            # ts.child_frame_id = 'd435_link'
            ts.child_frame_id = 'd435_depth_optical_frame'
            ts.transform.translation.x = trans[0]
            ts.transform.translation.y = trans[1]
            ts.transform.translation.z = trans[2]
            #
            # r = Rotation.from_euler('zxy', [-10, 10, -10], degrees=True)
            r = Rotation.from_euler('zxy', [-0, 0, -0], degrees=True)
            rm = r.as_matrix()
            # print(rot)
            # print(rm)
            r_ori = Rotation.from_quat(rot)
            r_ori = r_ori.as_matrix()
            print(r_ori)
            r_modify = rm @ r_ori
            r_modify = Rotation.from_matrix(r_modify)
            quat = r_modify.as_quat()
            ts.transform.rotation.x = quat[0]
            ts.transform.rotation.y = quat[1]
            ts.transform.rotation.z = quat[2]
            ts.transform.rotation.w = quat[3]
            ts_pub.publish(ts)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue