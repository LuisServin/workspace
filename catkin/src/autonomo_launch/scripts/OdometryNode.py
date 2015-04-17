#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class Odometry:
	"""This class is inteded to calculate odometry message from a 
	    differential robot"""
	def __init__(self, node_name = 'odometryNode'):
		rospy.init_node(node_name)
		self.node_name = rospy.get_name()
		rospy.loginfo('started node %s', self.node_name)
		rospy.Subscriber('encTicks', Vector3, self.callback)

		# initialize all the variables in class

		# robot properties
		self.wheelRadius = 0 # [m]
		self.wheelBase = 0 # [m]

		self.velocityRigthWheel = 0
		self.velocityLeftWheel = 0

		self.positionX = 0
		self.positionY = 0
		self.angleTheta = 0

		self.linearVelocityX = 0
		self.linearVelocityY = 0
		self.angularVelocityTheta = 0

		self.ICCx = 0
		self.ICCy = 0

		self.dt = 0
		self.dx = 0
		self.dy = 0
		self.dtheta = 0

		self.currentTime = rospy.get_time()
		self.lastTime = rospy.get_time()
		
	def callback(self, data):
		rospy.loginfo("Encoder ticks are " + str(data.x) + " and " + str(data.y))
		calculateVelocities(data)

	def calculateVelocities(self, encoderTicks):
		self.currentTime = rospy.get_time()
		self.dt = currentTime - lastTime		

		# calculate velocity 
		self.velocityLeftWheel = data.x

	# ICC -> Instantaneous Center of Curvature
	# def computeICC():

if __name__ == '__main__':
	odometry = Odometry()
	rospy.spin()