#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from math import *

class Odometry:
	"""This class is inteded to calculate odometry message from a 
	    differential robot"""
	def __init__(self, node_name = 'odometryNode'):
		rospy.init_node(node_name)
		self.node_name = rospy.get_name()
		rospy.Subscriber('encTicks', Vector3, self.callback)
		self.rate = rospy.get_param("rate", 5)
		rospy.loginfo("Rate for this node is " + str(self.rate))
		rospy.loginfo("started node " + self.node_name)
		rospy.loginfo("Pi value is " + str(pi))

		# initialize all the variables in class

		# robot properties
		self.wheelRadius = 0.1012 # [m]
		self.wheelBase = 0.207 # [m]
		self.encCountsPerRev = 3592

		# wheel velocities
		self.velocityRigthWheel = 0
		self.velocityLeftWheel = 0

		# the beginning position and orientation
		self.positionX = 0
		self.positionY = 0
		self.angleTheta = 0

		# General Radius of curvature, linear and angular velocity
		self.radiusOfCurvature = 0
		self.linearVelocity = 0
		self.angularVelocity = 0

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
		self.calculateVelocities(data)
		rospy.loginfo("Radius of curvature" + str(self.radiusOfCurvature))
		
		# 



	def calculateVelocities(self, encoderTicks):
		self.currentTime = rospy.get_time()
		self.dt = self.currentTime - self.lastTime	

		# calculate velocity 
		rightWheelAngularVelocity = (encoderTicks.x * ( 2 * pi / 3592 ) ) / self.dt
		self.velocityRigthWheel = rightWheelAngularVelocity * self.wheelRadius

		leftWheelAngularVelocity = (encoderTicks.y * ( 2 * pi / 3592 ) ) / self.dt
		self.velocityLeftWheel = leftWheelAngularVelocity * self.wheelRadius

		rospy.loginfo("velocity left wheel " + str(self.velocityLeftWheel))
		rospy.loginfo("deltaTime " + str(self.dt))

	# compute the general Radius of curvature
	# angular and linear velocities
	def computeGeneralRWV(self):
		self.radiusOfCurvature = (self.wheelBase / 2) *  \
			((self.velocityLeftWheel + self.velocityRigthWheel) / \
			(self.velocityRigthWheel - self.velocityLeftWheel))


	# ICC -> Instantaneous Center of Curvature
	#def computeICC(self, ):


	# def spin(self):


if __name__ == '__main__':
	odometry = Odometry()
	rospy.spin()