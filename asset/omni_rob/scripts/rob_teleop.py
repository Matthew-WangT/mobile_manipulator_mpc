#!/usr/bin/env python3
# -*- coding: utf-8 -*

import os
import sys
import tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist)


def keyboardLoop():
    # 初始化
    rospy.init_node('rob_teleop')
    rate = rospy.Rate(rospy.get_param('~hz', 3))

    # 速度变量
    walk_vel_ = rospy.get_param('walk_vel', 0.2)
    run_vel_ = rospy.get_param('run_vel', 0.5)
    yaw_rate_ = rospy.get_param('yaw_rate', 0.2)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.0)

    max_tv = walk_vel_
    max_rv = yaw_rate_

    class MaxSpdCls:
        def __init__(self):
            self.count = 0
            # self.max_v_l = [0.4, 0.8, 1.]
            self.max_v_l = [0.8, 1.6, 3.2]

        def choose_spd(self):
            self.count += 1
            self.count %= len(self.max_v_l)
            return self.max_v_l[self.count]

    # 显示提示信息
    print("Reading from keyboard")
    print("Use WASD keys to control the robot to move")
    print("Press F to choose spd level")
    print("Press J to turn left")
    print("Press K to turn right")
    print("Press Q to quit")

    spd_er = MaxSpdCls()
    # 读取按键循环
    while not rospy.is_shutdown():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        # 不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch == 'w':
            max_tv = walk_vel_
            speed = 1
            speed_y = 0
            turn = 0
        elif ch == 's':
            max_tv = walk_vel_
            speed = -1
            speed_y = 0
            turn = 0
        elif ch == 'j':
            max_rv = yaw_rate_
            speed = 0
            speed_y = 0
            turn = 1
        elif ch == 'a':
            max_rv = yaw_rate_
            speed = 0
            speed_y = 1
            turn = 0
        elif ch == 'd':
            max_rv = yaw_rate_
            speed = 0
            speed_y = -1
            turn = 0
        elif ch == 'l':
            max_rv = yaw_rate_
            speed = 0
            speed_y = 0
            turn = -1
        elif ch == 'f':
            max_tv = spd_er.choose_spd()
            max_rv = max_tv
        elif ch == 'q':
            exit()
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            speed_y = 0
            turn = 0

        # 发送消息
        cmd.linear.x = speed * max_tv
        cmd.linear.y = speed_y * max_tv
        cmd.angular.z = turn * max_rv
        # print(cmd)
        pub.publish(cmd)
        # base_cmd_set(cmd)
        rate.sleep()
        # 停止机器人
        stop_robot()


def stop_robot():
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)


if __name__ == '__main__':
    # base_pos_get()
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass
