#!/usr/bin/env python

# Always make a division by integer return a float
# this is a feature in python 3 but not in python 2
from __future__ import division

import rospy
from geometry_msgs.msg import Twist

class RobotFourMecanumWheels():
	"""Class to implement direct kinematic for a 4 Mecanum
	   wheel omnidireccional robot and publish the velocity
	   in every motor"""
	def __init__(self, node_name="kinematic4Mecanum"):
		rospy.init_node(node_name)
		self.node_name = rospy.get_name()
		rospy.loginfo("started node " + self.node_name)
		self.rate = rospy.get_param("rate", 50)

		# subscriber for the message of desire robot velocity
		self.totalRobotVelocitySubs = rospy.Subscriber("cmd_vel", Twist, self.callback)

		# publisher of robot wheels desired speeds
		self.

	def spin():
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()
		
if __name__ == '__main__':
	robot = RobotFourMecanumWheels()
	robot.spin()