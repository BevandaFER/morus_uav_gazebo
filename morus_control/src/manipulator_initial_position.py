#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import Float32, Float64
import math

def initial():
	pub_joint0_left = rospy.Publisher('/morus/joint0_left_controller/command', Float64, queue_size=1)
	pub_joint1_left = rospy.Publisher('/morus/joint1_left_controller/command', Float64, queue_size=1)
	pub_joint0_right = rospy.Publisher('/morus/joint0_right_controller/command', Float64, queue_size=1)
	pub_joint1_right = rospy.Publisher('/morus/joint1_right_controller/command', Float64, queue_size=1)

	rospy.init_node('initial', anonymous=True)

	init_joint0_msg = Float64()
	init_joint0_msg.data = 1.0298
	init_joint1_msg = Float64()
	init_joint1_msg.data = -2.0596

	rospy.sleep(5)

	pub_joint0_left.publish(init_joint0_msg)
	pub_joint1_right.publish(init_joint1_msg)
	pub_joint1_left.publish(init_joint1_msg)
	pub_joint0_right.publish(init_joint0_msg)

	rospy.spin()

	rospy.sleep(5)
if __name__ == '__main__':
	try:
		initial()
	except rospy.ROSInterruptException:
		pass