#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import Float32, Float64
import math
from geometry_msgs.msg import Vector3

def startCirlce():
	pub_euler = rospy.Publisher('/morus/euler_ref', Vector3, queue_size=1)
	rospy.init_node('startCirlce', anonymous=True)

	while (True):
		sleep_time = 4
		speed = 0.03
		pub_euler.publish(getVectorMsg(speed, 0, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(speed/2, speed*2, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(0, speed, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(-speed*2, speed/2, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(-speed, 0, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(-speed/2, -speed*2, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(0, -speed, 0))
		rospy.sleep(sleep_time)
		pub_euler.publish(getVectorMsg(speed*2, -speed/2, 0))
		rospy.sleep(sleep_time)

def getVectorMsg(x, y, z):

	newMsg = Vector3()
	newMsg.x = x
	newMsg.y = y
	newMsg.z = z
	return newMsg
	d
if __name__ == '__main__':
	try:
		startCirlce()
	except rospy.ROSInterruptException:
		pass	