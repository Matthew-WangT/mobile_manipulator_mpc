#!/usr/bin/env python3
import numpy as np
import modern_robotics as mr
from omnirob_hmqr5 import OmniRobHmqr5
import rospy
from geometry_msgs.msg import Twist, WrenchStamped, Wrench
from std_msgs.msg import Float64,Float64MultiArray, String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
from sensor_msgs.msg import JointState

from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry


# real robot
class OmniRobHmqr5Ros(OmniRobHmqr5):
    def __init__(self, base_topic='/cmd_vel'):
        rospy.init_node('OmniRobUr5Ros', anonymous=True)
        self._base_vel_pub = rospy.Publisher(base_topic, Twist, queue_size=1)
        self._gripper_pub = rospy.Publisher("/gripper", String, queue_size=1)
        # self._real_arm_pub = rospy.Publisher("/joint_position_controller/command", Float64MultiArray, queue_size=1)
        self._real_arm_pub = rospy.Publisher("/joint_position_controller/command", Float64MultiArray, queue_size=1)
        self._arm_pos_pub = []
        self._arm_effort_pub = []
        for i_pub in range(6):
            tmp_pub = rospy.Publisher("/joint" + str(i_pub + 1) + "_position_controller/command", Float64, queue_size=1)
            self._arm_pos_pub.append(tmp_pub)
        # subscriber
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.joint_q = np.zeros(6)
        self.joint_dq = np.zeros(6)
        self.joint_ddq = np.zeros(6)
        # tf
        self._tf_listener = tf.TransformListener()
        self.pose = [0, 0, 0]

    def get_world_frame(self):
        return self.pose

    def joint_state_callback(self, js):
        # order is chaos, remap it
        idx_remap = [4, 3, 0, 5, 6, 7]
        for i in range(6):
            idx = idx_remap[i]
            self.joint_q[i] = list(js.position)[idx]
            # print(self.joint_q[i])
            # self.joint_dq[i] = list(js.velocity)[idx]
            # self.joint_ddq[i] = list(js.effort)[idx]

    def wheel_vel_ctrl(self, desired_wheel_vel):
        pass

    def get_amr_pos(self):
        return self.joint_q

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

    def gripper_ctrl(self, state):
        msg = String()
        if state == 1:
            msg.data = 'open'
        else:
            msg.data = 'close'        
        self._gripper_pub.publish(msg)


    def arm_ctrl(self, desired_arm_joint_angles, real=False):
        # gazebo
        if not real:
            for i in range(6):
                msg = Float64()
                msg.data = desired_arm_joint_angles[i]
                self._arm_pos_pub[i].publish(msg)
        else: # real(topic control, abandoned)
            real_msg = Float64MultiArray(data=desired_arm_joint_angles)
            print("pos_cmd(rad): ", real_msg.data)
            self._real_arm_pub.publish(real_msg)

    def base_ctrl(self, x, y, theta):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = theta
        self._base_vel_pub.publish(msg)

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
        gripper_msg = String()
        if state == 1:  # open
            gripper_msg.data = 'open'
            self._gripper_pub.publish(gripper_msg)
        elif state == 0:  # close
            gripper_msg.data = 'close'
            self._gripper_pub.publish(gripper_msg)
        else:
            exit(1)

    def wheel_pos_ctrl(self, desired_wheel_positions):
        pass


#  ================================================== test =============================================================
"""
test omni_rob 
"""
if __name__ == "__main__":
    rob = OmniRobHmqr5Ros('/cmd_vel')
    import numpy as np

    arm_theta_list = np.zeros(6)
    while rob.joint_q[0] == 0.0:
        rospy.sleep(0.1)
    arm_theta_list = rob.joint_q
    print("rec: ", arm_theta_list)
    # print(rob.ur5_fk(arm_theta_list))
    n = 3
    x, y, t = 0, 0, 0
    for i in range(n):
        print("ctrl")
        arm_theta_list[5] = arm_theta_list[5] + 0.2
        print("pub: ", arm_theta_list)
        rob.arm_ctrl(arm_theta_list)
        rospy.sleep(1)
