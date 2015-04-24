#!/usr/bin/env python

# Always make a division by integer return a float
# this is a feature in python 3 but not in python 2
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
# custom message for the four mecanum wheels robot.
from atwork_msgs.msg import FourWheels

class RobotFourMecanumWheels():
	"""Class to implement direct kinematic for a 4 Mecanum
	   wheel omnidireccional robot and publish the velocity
	   in every motor"""
	def __init__(self, node_name="kinematic4Mecanum"):
		rospy.init_node(node_name)
		self.node_name = rospy.get_name()
		rospy.loginfo("started node " + self.node_name)
		self.rate = rospy.get_param("rate", 10)

		# subscriber for the message of desire robot velocity
		self.totalRobotVelocitySubs = rospy.Subscriber("cmd_vel", Twist, self.callback)

		# publisher of robot wheels desired speeds
		self.robotWheelSpeedPub = rospy.Publisher("wheelSpeeds", FourWheels, queue_size=1)
		self.robotWheelSpeedMsg = FourWheels()

		# variables for physical properties in the robot
		self.lengthAxisX = rospy.get_param("lengthAxisX", 0.05) # [m]
		self.lengthAxisY = rospy.get_param("lengthAxisY", 0.1) # [m]
		self.wheelRadius = rospy.get_param("wheelRadius", 0.1012) # [m]

		self.wheelOneSpeed = 0.0
		self.wheelTwoSpeed = 0.0
		self.wheelThreeSpeed = 0.0
		self.wheelFourSpeed = 0.0

	def callback(self, data):
		self.wheelOneSpeed = (1 / self.wheelRadius) \
			* ( 1 * data.linear.x - (1) * data.linear.y \
				- (self.lengthAxisX + self.lengthAxisY) * data.angular.z)

		self.wheelTwoSpeed = (1 / self.wheelRadius) \
			* ( 1 * data.linear.x + (1) * data.linear.y \
				+ (self.lengthAxisX + self.lengthAxisY) * data.angular.z)

		self.wheelThreeSpeed = (1 / self.wheelRadius) \
			* ( 1 * data.linear.x + (1) * data.linear.y \
				- (self.lengthAxisX + self.lengthAxisY) * data.angular.z)

		self.wheelFourSpeed = (1 / self.wheelRadius) \
			* ( 1 * data.linear.x - (1) * data.linear.y \
				+ (self.lengthAxisX + self.lengthAxisY) * data.angular.z)

	def update(self):
		# assign the calculated speed for each wheel
		self.robotWheelSpeedMsg.wheel1 = self.wheelOneSpeed
		self.robotWheelSpeedMsg.wheel2 = self.wheelTwoSpeed
		self.robotWheelSpeedMsg.wheel3 = self.wheelThreeSpeed
		self.robotWheelSpeedMsg.wheel4 = self.wheelFourSpeed

		self.robotWheelSpeedPub.publish(self.robotWheelSpeedMsg)

	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()
		
if __name__ == '__main__':
	robot = RobotFourMecanumWheels()
	robot.spin()