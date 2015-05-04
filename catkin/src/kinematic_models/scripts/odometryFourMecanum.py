#!/usr/bin/env python

from __future__ import division

import rospy

# to broadcast transformation between odom -> base_link
import tf
from geometry_msgs.msg import TransformStamped

# to subscribe for encoder info
from atwork_msgs.msg import FourWheels

# to publish odometry message
from nav_msgs.msg import Odometry

import math

from tf.transformations import quaternion_from_euler

class OdometryFourMecanum():
	"""Class to calculate odometry message from a omnidireccional
	   robot with four mecanum wheels"""
	def __init__(self, node_name = 'odometryNodeMecanum'):
		rospy.init_node(node_name)
		self.node_name = rospy.get_name()
		rospy.loginfo("started node " + self.node_name)
		self.rate = rospy.get_param("rate", 5)

		self.encoderSub = rospy.Subscriber("encTicks", FourWheels, self.cbEncTicks)

		# odometry publisher and message
		self.odomPub = rospy.Publisher("odom", Odometry, queue_size=1)
		self.odomMsg = Odometry()

		# odometry transform and message
		self.odomBroadcaster = tf.TransformBroadcaster()
		self.odomTransform = TransformStamped()

		# initialize all the variables in class

		# robot physical properties
		self.lengthAxisX = rospy.get_param("lengthAxisX", 0.05) # [m]
		self.lengthAxisY = rospy.get_param("lengthAxisY", 0.1) # [m]
		self.wheelRadius = rospy.get_param("wheelRadius", 0.1012) # [m]
		self.encTotalTicks = rospy.get_param("encTotalTicks", 3592)

		# wheel speeds
		self.wheelOneSpeed = 0.0
		self.wheelTwoSpeed = 0.0
		self.wheelThreeSpeed = 0.0
		self.wheelFourSpeed = 0.0

		# the initial position and orientation
		self.positionX = 0.0
		self.positionY = 0.0
		self.angleTheta = 0.0

		# robot global velocities
		self.speedBodyX = 0.0
		self.speedBodyY = 0.0
		self.speedBodyTheta = 0.0

		# time variables
		self.currentTime = rospy.get_time()
		self.lastTime = rospy.get_time()


	def cbEncTicks(self, data):
		self.currentTime = rospy.get_time()

		rospy.loginfo("calculating odometry")

		self.dt = self.currentTime - self.lastTime
		self.calculateSpeeds(data)
		self.calculateVelocity()
		self.computeNewPosition()

		# save the current time
		self.lastTime = self.currentTime


	def calculateSpeeds(self, encTicks):
		wheelOneAngularSpeed = (encTicks.wheel1 * \
			(2 * math.pi / self.encTotalTicks )) / self.dt
		self.wheelOneSpeed = wheelOneAngularSpeed * self.wheelRadius

		wheelTwoAngularSpeed = (encTicks.wheel2 * \
			(2 * math.pi / self.encTotalTicks )) / self.dt
		self.wheelTwoSpeed = wheelTwoAngularSpeed * self.wheelRadius

		wheelThreeAngularSpeed = (encTicks.wheel3 * \
			(2 * math.pi / self.encTotalTicks )) / self.dt
		self.wheelThreeSpeed = wheelThreeAngularSpeed * self.wheelRadius

		wheelFourAngularSpeed = (encTicks.wheel4 * \
			(2 * math.pi / self.encTotalTicks )) / self.dt
		self.wheelFourSpeed = wheelFourAngularSpeed * self.wheelRadius

	def calculateVelocity(self):
		# operations are split for a better reading
		self.speedBodyX = self.wheelOneSpeed + self.wheelTwoSpeed \
			+ self.wheelThreeSpeed + self.wheelFourSpeed
		self.speedBodyX = self.speedBodyX * (1 / 4) * self.wheelRadius

		self.speedBodyY = - self.wheelOneSpeed + self.wheelTwoSpeed \
			+ self.wheelThreeSpeed - self.wheelFourSpeed
		self.speedBodyY = self.speedBodyY * (1 / 4) * self.wheelRadius

		self.speedBodyTheta = - self.wheelOneSpeed + self.wheelTwoSpeed \
			- self.wheelThreeSpeed + self.wheelFourSpeed
		self.speedBodyTheta = self.speedBodyTheta * (1/4) * self.wheelRadius \
			* (1 / (self.lengthAxisX + self.lengthAxisY))

	def computeNewPosition(self):
		self.positionX += (self.speedBodyX * math.cos(self.angleTheta) \
			- self.speedBodyY * math.sin(self.angleTheta)) * self.dt

		self.positionY += (self.speedBodyX * math.sin(self.angleTheta) \
			+ self.speedBodyY * math.cos(self.angleTheta)) * self.dt

		self.angleTheta += self.speedBodyTheta * self.dt

		# bound the value of angleTheta from
		# - 2 * pi < theta < 2 * pi
		while (self.angleTheta > 2 * math.pi):
			self.angleTheta -= 2 * math.pi
		while (self.angleTheta < -2 * math.pi):
			self.angleTheta += 2 * math.pi

	def update(self):
		# http://answers.ros.org/question/69754/quaternion-transformations-in-python/
		quaternion = quaternion_from_euler(0,0,self.angleTheta)
		parent_frame = "odom"
		child_frame = "base_link"

		# In tf.broadcaster.sendTransform() function for python, every element
		# need to be declare in the function, can't use a Msg
		# https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/broadcaster.py
		# data for transformation
		self.odomTransform.header.frame_id = parent_frame
		self.odomTransform.header.stamp = rospy.Time.from_sec(self.currentTime)
		self.odomTransform.child_frame_id = child_frame

		self.odomTransform.transform.translation.x = self.positionX
		self.odomTransform.transform.translation.y = self.positionY
		self.odomTransform.transform.translation.z = 0
		self.odomTransform.transform.rotation.x = quaternion[0]
		self.odomTransform.transform.rotation.y = quaternion[1]
		self.odomTransform.transform.rotation.z = quaternion[2]
		self.odomTransform.transform.rotation.w = quaternion[3]

		# broadcast the transformation
		self.odomBroadcaster.sendTransformMessage(self.odomTransform)

	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()

if __name__ == '__main__':
	robotOdometry = OdometryFourMecanum()
	robotOdometry.spin()