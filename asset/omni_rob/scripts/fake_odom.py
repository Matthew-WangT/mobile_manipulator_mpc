#!/usr/bin/env python3
# -*- coding: utf-8 -*
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
# import turtlesim.msg

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import tf


def handle_turtle_pose(rep, name):
    # 创建tf的广播器
    br = tf.TransformBroadcaster()

    # 广播world与海龟坐标系之间的tf数据
    x, y, z, w = rep.pose.orientation.x, rep.pose.orientation.y, rep.pose.orientation.z, rep.pose.orientation.w
    # br.sendTransform((rep.pose.position.x, rep.pose.position.y, 0),
    br.sendTransform((rep.pose.position.x, rep.pose.position.y, rep.pose.position.z),
                     # tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     (x, y, z, w),
                     rospy.Time.now(),
                     name,
                     "odom")


def base_pos_get():
    """
    :return: response.pose
    """
    # service
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        person_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # 请求服务调用，输入请求数据
        pos_msg = ModelState()
        pos_msg.model_name = "robot"
        pos_msg.reference_frame = "world"
        response = person_client("robot", "world")
        return response

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    rospy.init_node('fake_odom_node')
    odom_pub = rospy.Publisher('odom', Odometry)
    rate = rospy.Rate(100)
    # odom_trans = TransformStamped()
    # odom_trans.header.stamp = rospy.Time.now()
    # odom_trans.header.frame_id = 'odom'
    # odom_trans.child_frame_id = 'agv_base_link'
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'dummy'
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        response = base_pos_get()
        # print(response)
        odom.pose.pose = response.pose
        odom.twist.twist = response.twist
        # print("odom:============:")
        # print(odom)
        odom_pub.publish(odom)
        rate.sleep()
        handle_turtle_pose(response, 'dummy')
        # rospy.spin()
