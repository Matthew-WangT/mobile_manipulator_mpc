#!/usr/bin/env python3
# -*- coding: utf-8 -*
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
import math
import tf.transformations


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
        # pos_msg.twist.linear.x = cmd_vel[0]
        # pos_msg.twist.linear.y = cmd_vel[1]
        # pos_msg.twist.angular.z = cmd_vel[1]
        pos_msg.model_name = "robot"
        pos_msg.reference_frame = "world"
        response = person_client("robot", "world")
        # print("response: ")
        # print(response.header)
        # print("response.pose")
        # print(response.pose)
        # print(response.success)
        return response.pose

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def base_cmd_set(cmd_vel):
    """
    :param cmd_vel: dx, d_theta
    :return: response.success
    """
    # service
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        person_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # 请求服务调用，输入请求数据
        state_base = ModelState()
        state_base.pose = base_pos_get()
        v_xb, v_yb = cmd_vel.linear.x, cmd_vel.linear.y
        qua = [state_base.pose.orientation.x, state_base.pose.orientation.y, state_base.pose.orientation.z,
               state_base.pose.orientation.w]
        _, _, omega = tf.transformations.euler_from_quaternion(qua)
        # print(omega)
        print("v_yb:", v_yb)
        state_base.twist.linear.x = v_xb * math.cos(omega) - v_yb * math.sin(omega)
        state_base.twist.linear.y = v_xb * math.sin(omega) + v_yb * math.cos(omega)
        state_base.twist.angular.z = cmd_vel.angular.z
        # state_base.twist.angular.z *= 0.3
        # state_base.twist.linear.x *= 0.3
        # state_base.twist.linear.y *= 0.3
        state_base.model_name = "robot"
        state_base.reference_frame = "world"
        response = person_client(state_base)
        return response.success

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def callback(cmd_vel):
    base_cmd_set(cmd_vel)


def listener():
    rospy.init_node('base_controller', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(50)
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
