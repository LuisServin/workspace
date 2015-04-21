#!/usr/bin/env python

# Always make a division by integer return a float
# this a feature in python 3 but not in python 2
from __future__ import division

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from math import *

class ComputeOdometry:
	"""This class is inteded to calculate odometry message from a 
	    differential robot"""
	def __init__(self, node_name = 'odometryNode'):
		rospy.init_node(node_name)
		self.node_name = rospy.get_name()
		rospy.loginfo("started node " + self.node_name)
		self.rate = rospy.get_param("rate", 5)

		self.encoderSub = rospy.Subscriber('encTicks', Vector3, self.callback)

		# odometry message
		self.odomPub = rospy.Publisher("odom", Odometry)

		# initialize all the variables in class

		# robot properties
		self.wheelRadius = 0.1012 # [m]
		self.wheelBase = 0.207 # [m]
		self.encCountsPerRev = 3592

		# wheel velocities
		self.velocityRigthWheel = 0
		self.velocityLeftWheel = 0

		# the initial position and orientation
		self.positionX = 0
		self.positionY = 0
		self.angleTheta = 0

		# General Radius of curvature, linear and angular velocity
		self.radiusOfCurvature = 0
		self.linearBodyVelocity = 0
		self.angularBodyVelocity = 0

		self.velocityBodyX = 0
		self.velocityBodyY = 0
		self.velocityBodyTheta = 0

		self.ICCx = 0
		self.ICCy = 0

		self.dt = 0
		self.dx = 0
		self.dy = 0
		self.dtheta = 0

		self.currentTime = rospy.get_time()
		self.lastTime = rospy.get_time()

	def callback(self, data):
		self.calculateVelocities(data)
		if self.velocityLeftWheel == self.velocityRigthWheel:		
			rospy.loginfo("Same velocity in both wheel")

			self.linearBodyVelocity = (1/2) * (self.velocityLeftWheel + self.velocityRigthWheel)
			self.angularBodyVelocity = 0

			self.computeVelocitiesXY()

			# calculate the position
			self.computeNewPositionSameVelocities()
		else:
			rospy.loginfo("Different wheels' velocities")
			self.computeGeneralRWV()
			self.computeVelocitiesXY()
			self.computeICC()			
			self.computeNewPositionDifferentVelocites()

		rospy.loginfo("x = " + str(self.positionX) + " y = " + str(self.positionY) \
			+ " theta = " + str(self.angleTheta * 180 / pi))
		self.lastTime = self.currentTime
		
	# compute the speed in every wheel based on encoder ticks
	def calculateVelocities(self, encoderTicks):
		self.currentTime = rospy.get_time()
		self.dt = self.currentTime - self.lastTime	

		# calculate velocity 
		rightWheelAngularVelocity = (encoderTicks.x * ( 2 * pi / 3592 ) ) / self.dt
		self.velocityRigthWheel = rightWheelAngularVelocity * self.wheelRadius

		leftWheelAngularVelocity = (encoderTicks.y * ( 2 * pi / 3592 ) ) / self.dt
		self.velocityLeftWheel = leftWheelAngularVelocity * self.wheelRadius

	# compute the general Radius of curvature
	# angular and linear velocities
	def computeGeneralRWV(self):
		self.radiusOfCurvature = (self.wheelBase / 2) * \
			((self.velocityLeftWheel + self.velocityRigthWheel) / \
			(self.velocityRigthWheel - self.velocityLeftWheel))

		self.angularBodyVelocity = (self.velocityRigthWheel - self.velocityLeftWheel) / \
			self.wheelBase

		self.linearBodyVelocity = (self.velocityRigthWheel	+ self.velocityLeftWheel) / 2

	# ICC -> Instantaneous Center of Curvature
	def computeICC(self):
		self.ICCx = self.positionX - self.radiusOfCurvature * sin(self.angleTheta)
		self.ICCy = self.positionY + self.radiusOfCurvature * cos(self.angleTheta)


	# descompose the linear body velocity into the x and y component
	# in odom frame
	def computeVelocitiesXY(self):
		self.velocityBodyX = self.linearBodyVelocity * cos(self.angleTheta)
		self.velocityBodyY = self.linearBodyVelocity * sin(self.angleTheta)

	def computeNewPositionDifferentVelocites(self):
		self.positionX = cos(self.angularBodyVelocity * self.dt) * (self.positionX - self.ICCx) \
			- sin(self.angularBodyVelocity * self.dt) * (self.positionY - self.ICCy) \
			+ self.ICCx

		self.positionY = sin(self.angularBodyVelocity * self.dt) * (self.positionX - self.ICCx) \
			+ cos(self.angularBodyVelocity * self.dt) * (self.positionY - self.ICCy) \
			+ self.ICCy

		self.angleTheta = self.angleTheta + self.angularBodyVelocity * self.dt

		# bound the value of angleTheta from 
		# - 2*pi < theta < 2*pi
		while (self.angleTheta > 2 * pi):
			self.angleTheta -= 2 * pi
		while (self.angleTheta < -2 * pi):
			self.angleTheta += 2 * pi

	def computeNewPositionSameVelocities(self):
		self.dx = self.velocityBodyX * cos(self.angleTheta) \
			- self.velocityBodyY * sin(self.angleTheta)
		self.dx *= self.dt

		self.dy = self.velocityBodyX * sin(self.angleTheta) \
			+ self.velocityBodyY * cos(self.angleTheta)
		self.dy *= self.dt

		# robot is straighforward
		self.dtheta = 0

		self.positionX += self.dx
		self.positionY += self.dy
		self.angleTheta += self.dtheta

	def update(self):
		self.pubc = 0

	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.update()
			r.sleep()


if __name__ == '__main__':
	odometry = ComputeOdometry()
	odometry.spin()