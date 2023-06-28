#!/usr/bin/env python3
import numpy as np
import modern_robotics as mr
from omnirob_ur5 import OmniRobUr5
import rospy
from geometry_msgs.msg import Twist, WrenchStamped, Wrench
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation


class OmniRobUr5Ros(OmniRobUr5):
    def __init__(self, base_topic='/cmd_vel'):
        rospy.init_node('OmniRobUr5Ros', anonymous=True)
        self._base_vel_pub = rospy.Publisher(base_topic, Twist, queue_size=1)
        self._gripper_left_pub = rospy.Publisher("/left_finger_controller/command", Float64, queue_size=1)
        self._gripper_right_pub = rospy.Publisher("/right_finger_controller/command", Float64, queue_size=1)
        self._arm_pos_pub = []
        self._arm_effort_pub = []
        for i_pub in range(6):
            tmp_pub = rospy.Publisher("/joint" + str(i_pub + 1) + "_position_controller/command", Float64, queue_size=1)
            tmp_effort_pub = rospy.Publisher("/joint" + str(i_pub + 1) + "_effort_controller/command", Float64,
                                             queue_size=1)
            self._arm_pos_pub.append(tmp_pub)
            self._arm_effort_pub.append(tmp_effort_pub)
        # subscriber
        # rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.joint_q = np.zeros(6)
        self.joint_dq = np.zeros(6)
        self.joint_ddq = np.zeros(6)
        # force and torque
        rospy.Subscriber("/ft_sensor_topic_ee_fixed_joint", WrenchStamped, self.wrench_callbck)
        self.ee_wrench = np.zeros(3)
        self.ee_wrench_ee_frame = np.zeros(3)
        self.ee_wrench_list = []
        # # tf
        self._tf_listener = tf.TransformListener()
        #
        # rate = rospy.Rate(100)
        # rate.sleep()
        # rospy.spin()
        # pose of rob (x, y, theta)
        self.pose = [0, 0, 0]

    def get_world_frame(self):
        return self.pose

    def joint_state_callback(self, js):
        # order is chaos, remap it
        idx_remap = [4, 3, 0, 5, 6, 7]
        for i in range(6):
            idx = idx_remap[i]
            self.joint_q[i] = js.position[idx]
            self.joint_dq[i] = js.velocity[idx]
            self.joint_ddq[i] = js.effort[idx]

    def wrench_callbck(self, ws):
        self.ee_wrench_list.append(ws.wrench)
        if len(self.ee_wrench_list) >= 3:
            self.ee_wrench_list.pop(0)
        wrench_sum = np.array([0.0, 0.0, 0.0])
        N = len(self.ee_wrench_list)
        for i in range(N):
            wf = self.ee_wrench_list[i].force
            tmp_wrench = np.array([wf.x, wf.y, wf.z])
            wrench_sum += tmp_wrench
        ee_wrench = wrench_sum / N
        self.ee_wrench_ee_frame = ee_wrench
        rot_quat = self.get_tip_pos()[1]
        rot_Rota = Rotation.from_quat(rot_quat)
        rot_R = rot_Rota.as_matrix()
        # print("rot_R:", rot_R)
        self.ee_wrench = rot_R.T @ ee_wrench
        # self.ee_wrench.force.x = wrench_sum[0]/N
        # self.ee_wrench.force.y = wrench_sum[1]/N
        # self.ee_wrench.force.z = wrench_sum[2]/N

    def wheel_vel_ctrl(self, desired_wheel_vel):
        pass

    def get_amr_pos(self):
        pass

    def get_tip_pos(self):
        for _ in range(1):
            try:
                latest_t = self._tf_listener.getLatestCommonTime("ee_link", "odom")
                # print("latest_t:")
                # print(latest_t)
                P_we, rot_tmp = self._tf_listener.lookupTransform("ee_link", "odom", latest_t)
                # print("P_we:")
                # print(P_we)
                # print("rot_tmp:", rot_tmp)
                return P_we, rot_tmp
            except (tf.LookupException, tf.ConnectivityException):
                continue
        # return trans_tmp

    def get_tip_pos2(self, cfg):
        # rospy.Subscriber("/cmd_vel", Twist, callback)
        # trans_tmp, rot_tmp = -1, -1
        # trans_tmp = -1
        P_wb_w = np.array([cfg[0, 1], cfg[0, 2], 0.4]).reshape(3, 1)
        for _ in range(1):
            try:
                # latest_t = self._tf_listener.getLatestCommonTime("ee_link", "dummy")
                # print("latest_t:")
                # print(latest_t)
                # P_be_b, rot_tmp = self._tf_listener.lookupTransform("ee_link", "dummy", latest_t)
                P_be_b = self.ur5_fk(cfg[0, 3:9])[0:3, 3]
                print("P_be_b:")
                print(P_be_b)
                P_be_b = np.array(P_be_b).reshape(3, 1)
                print("P_be_b:")
                print(P_be_b)
                R_wb = Rotation.from_euler('zxy', [self.config[0, 0], 0, 0], degrees=False).as_matrix()
                # TR_wb = mr.RpToTrans(R_wb, P_wb)
                P_we_w = P_wb_w + R_wb.dot(P_be_b)
                print("P_we_w:", P_we_w)
                return P_we_w[:, 0]
                # return trans_tmp  # , rot_tmp
            except (tf.LookupException, tf.ConnectivityException):
                continue
        # return trans_tmp

    # Get amrRob arm joint angles
    def get_arm_agl(self):
        amrRob_arm_q = [0, 0, 0, 0, 0, 0]

    #     for j in range(0, 6):
    #         amrRob_arm_q[j] =
    #     return amrRob_arm_q

    def arm_ctrl(self, desired_arm_joint_angles):
        for i in range(6):
            msg = Float64()
            msg.data = desired_arm_joint_angles[i]
            self._arm_pos_pub[i].publish(msg)

    def arm_ctrl_effort(self, desired_arm_joint_torque):
        for i in range(6):
            msg = Float64()
            msg.data = desired_arm_joint_torque[i]
            self._arm_effort_pub[i].publish(msg)

    def base_ctrl(self, x, y, theta):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = theta
        self._base_vel_pub.publish(msg)

    # 
    def odom_callback(self, odom):
        posi = odom.pose.pose.position
        ori = odom.pose.pose.orientation
        (r, p, y) = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        # print("rpy:", r,p,y)
        self.pose = [posi.x, posi.y, y]

    @staticmethod
    def world_frame_set(desired_world_positions):
        """
        :param desired_world_positions: x, y, theta
        :return: response.success
        """
        # service
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            person_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            # 请求服务调用，输入请求数据
            pos_msg = ModelState()
            pos_msg.pose.position.x = desired_world_positions[0]
            pos_msg.pose.position.y = desired_world_positions[1]
            pos_msg.pose.position.z = 0.06
            theta_quat_tmp = Rotation.from_euler('zxy', [desired_world_positions[2], 0, 0], degrees=False).as_quat()
            # print("theta_quat_tmp:")
            # print(theta_quat_tmp)
            pos_msg.pose.orientation.z = theta_quat_tmp[2]
            pos_msg.pose.orientation.w = theta_quat_tmp[3]
            pos_msg.model_name = "robot"
            pos_msg.reference_frame = "world"
            response = person_client(pos_msg)
            # print("response: ")
            # print(response)
            # print(response.success)
            return response.success

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    @staticmethod
    def base_cmd_set(cmd_vel):
        """
        :param cmd_vel: dx, dy, d_theta
        :return: response.success
        """
        # service
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            person_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            # 请求服务调用，输入请求数据
            pos_msg = ModelState()
            pos_msg.twist.linear.x = cmd_vel[0]
            # pos_msg.twist.linear.y = cmd_vel[1]
            pos_msg.twist.angular.z = cmd_vel[1]
            pos_msg.model_name = "robot"
            pos_msg.reference_frame = "world"
            response = person_client(pos_msg)
            # print("response: ")
            # print(response)
            # print(response.success)
            return response.success

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def gripper_ctrl(self, state, value=1.0):
        gripper_msg = Float64()
        if state == 1:  # open
            gripper_msg.data = value
            self._gripper_left_pub.publish(gripper_msg)
            gripper_msg.data = -value
            self._gripper_right_pub.publish(gripper_msg)
        elif state == 0:  # close
            gripper_msg.data = -value
            self._gripper_left_pub.publish(gripper_msg)
            gripper_msg.data = value
            self._gripper_right_pub.publish(gripper_msg)
        else:
            exit(1)

    def wheel_pos_ctrl(self, desired_wheel_positions):
        pass


#  ================================================== test =============================================================
"""
test omni_rob 
"""
if __name__ == "__main__":
    rob = OmniRobUr5Ros('/cmd_vel')
    import numpy as np

    arm_theta_list = np.zeros(6)
    # print(arm_theta_list)
    # print(rob.ur5_fk(arm_theta_list))
    n = 20000
    x, y, t = 0, 0, 0
    for i in range(n):
        # arm_theta_list[0] = np.sin(i * 0.01) * 1.5708
        # arm_theta_list[1] = (np.sin(i * 0.01) - 1) * 1.5708
        rob.arm_ctrl(arm_theta_list)
        import time

        # x, y, theta = 0.0, 0.0, 0
        x, y, theta = np.cos(i * 0.01), np.sin(i * 0.01), (i * 0.01)
        while theta > np.pi * 2:
            theta -= 2 * np.pi
        time.sleep(0.02)
        # rob.base_ctrl(x, y, theta)
        time.sleep(0.02)
        # theta, x, y = -1.57, 10, 1
        dwp = np.array([x, y, theta])
        # rob.world_frame_set(dwp)
        trans = rob.get_tip_pos()
        # print(f"trans:{trans}")
        rob.gripper_ctrl(1)
        time.sleep(0.1)
        rob.gripper_ctrl(0)
        time.sleep(0.1)
        # print("tip pose:", rob.get_tip_pos())
        print("ee wrench:", rob.ee_wrench)
