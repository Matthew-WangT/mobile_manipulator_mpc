#!/usr/bin/env python3
# -*- coding: utf-8 -*

# from geometry_msgs.msg import WrenchStamped
# import rospy
#
# def DrawForce(pub, values):
#     msg = WrenchStamped()
#
#     msg.header.frame_id = "FT_link"
#     msg.header.stamp = rospy.Time.now()
#     msg.wrench.force.x = float(values[0])
#     msg.wrench.force.y = float(values[1])
#     msg.wrench.force.z = float(values[2])
#
#     msg.wrench.torque.x = float(values[3])
#     msg.wrench.torque.y = float(values[4])
#     msg.wrench.torque.z = float(values[5])
#
# def listener():
#     rospy.init_node('base_controller', anonymous=True)
#     rospy.Subscriber("/cmd_vel", WrenchStamped, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rate = rospy.Rate(50)
#     rate.sleep()
#     rospy.spin()
# if __name__ == '__main__':
#     try:
#         listener()
#     except rospy.ROSInterruptException:
#         pass