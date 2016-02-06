#!/usr/bin/env python
# -*- coding: utf8 -*-
""" 
Luis Alfredo Servín Garduño
National and Autonomous University of Mexico
luis.alfredo.sega@gmail.com 
"""

##############################################################
import rospy
##############################################################
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
##############################################################


class inverse_kinematic:
	"""docstring for inverse_kinematic
		This is a library to read cmd_vel message
		in ROS enviroment and publish wheels transformations
		to robot model"""
	def __init__(self):
		# init node
		rospy.init_node('inverse_kinematic_node')

		# variable declaration

		# create an instance of JoinState message and fill with
		# neccesary initial information
		self.state_str = JointState()
		self.state_str.header = Header()
		self.state_str.header.stamp = rospy.Time.now()
	
		# wheels start in home position
		self.state_str.name = ['base_footprint_to_base_wheel', 'base_footprint_to_base_wheel_front_right', 'base_footprint_to_base_wheel_front_left', 'base_footprint_to_base_wheel_back_right']
		self.state_str.position = [0.0, 0.5, 0.0, 0.0]
		self.state_str.velocity = []
		self.state_str.effort = []

		 # create a topic to publish joint states
		wheel_vel_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

		# subscribe to cmd_vel topic
		rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)

		# create a publisher for join states
		self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)

	def cmd_vel_cb(self, cmd_msg):
		rospy.loginfo("I'm hearing %s", str(cmd_msg.linear.x))
		# state_str.header.stamp = rospy.Time.now()
		# pub.publish(state_str)
	
	# function to start pushlising actual message
	def spin(self):
		# rate publication
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.state_str)
			r.sleep()

		