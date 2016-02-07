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

		self.nodename = rospy.get_name()
		rospy.loginfo("Starting node with name: %s", self.nodename)

		# variable declaration

		# speed used in robot
		self.v_x = 0.5
		self.v_y = 0.0
		self.w_z = 0.0

		# physical properties
		self.l_x = 0.5 # [m]
		self.l_y = 0.5 # [m]
		self.r = 0.1 # [m]

		# wheels speed
		self.w_1 = 0.0 # front left
		self.w_2 = 0.0 # front right
		self.w_3 = 0.0 # back left
		self.w_4 = 0.0 # back right

		# variables to store wheel positions
		self.w_1_p = 0.0
		self.w_2_p = 0.0
		self.w_3_p = 0.0
		self.w_4_p = 0.0

		# create an instance of JoinState message and fill with
		# neccesary initial information
		self.state_str = JointState()
		self.state_str.header = Header()
		self.state_str.header.stamp = rospy.Time.now()
	
		# wheels start in home position
		self.state_str.name = ['base_footprint_to_base_wheel', 'base_footprint_to_base_wheel_front_right', 'base_footprint_to_base_wheel_front_left', 'base_footprint_to_base_wheel_back_right']
		self.state_str.position = [0.0, 0.0, 0.0, 0.0]
		self.state_str.velocity = []
		self.state_str.effort = []

		# create a publisher for join states
		self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)

		# subscribe to cmd_vel topic
		rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)

	def cmd_vel_cb(self, cmd_msg):
		# update calculation values.
		self.v_x = cmd_msg.linear.x
		self.v_y = cmd_msg.linear.z
		self.w_z = cmd_msg.angular.z
	
	def update(self):
		# Writting equations for wheel speed rotation calculation
		self.w_1 = 1 / self.r * (self.v_x - self.v_y - \
			(self.l_x + self.l_y) * self.w_z)
		
		self.w_2 = 1 / self.r * (self.v_x + self.v_y + \
			(self.l_x + self.l_y) * self.w_z)

		self.w_1 = 1 / self.r * (self.v_x + self.v_y - \
			(self.l_x + self.l_y) * self.w_z)

		self.w_1 = 1 / self.r * (self.v_x - self.v_y + \
			(self.l_x + self.l_y) * self.w_z)

		# calculate new wheel position
		self.w_1_p = self.w_1_p + self.w_1 * self.r

		self.state_str.header.stamp = rospy.Time.now()
		self.state_str.position = [self.w_1_p, 0.0, 0.0, 0.0]

	# function to start pushlising actual message
	def spin(self):
		# rate publication
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.update()
			self.pub.publish(self.state_str)
			r.sleep()

if __name__ == '__main__':
	ian = inverse_kinematic()
	ian.spin()