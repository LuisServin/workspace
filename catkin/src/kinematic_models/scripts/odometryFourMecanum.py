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
		rospy.logifo("started node " + self.node_name)
		self.rate = rospy.get_param("rate", 50)

		self.encoderSub = rospy.Subscriber("encTicks", FourWheels, self.cbEncTicks)

		# odometry publisher and message
		self.odomPub = rospy.Publisher("odom", Odometry)
		self.odomMsg = Odometry()

		# odometry transform and message
		self.odomBroadcaster = tf.TransformBroadcaster()
		self.odomTransform = TransformStamped()

		# initialize all the variables in class

	def cbEncTicks(self, data):
		self.currentTime = rospy.get_time()

	def update():
		rospy.logifo("hola")

	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()

if __name__ == '__main__':
	robotOdometry = OdometryFourMecanum()
	robotOdometry.spin()