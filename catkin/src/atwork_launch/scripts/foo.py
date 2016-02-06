#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from inverse_kinematic import *

# def cmd_vel_cb(cmd_msg):
# 	rospy.loginfo("I'm hearing %s", str(cmd_msg.linear.x))
# 	state_str.header.stamp = rospy.Time.now()
# 	pub.publish(state_str)

def foo():
	# init node
	# rospy.init_node('inverse_kinematic_node')

	# create an instance of JoinState message and fill with
	# neccesary initial information
	# state_str = JointState()
	# state_str.header = Header()
	# state_str.header.stamp = rospy.Time.now()
	
	# state_str.name = ['base_footprint_to_base_wheel', 'base_footprint_to_base_wheel_front_right', 'base_footprint_to_base_wheel_front_left', 'base_footprint_to_base_wheel_back_right']
	# state_str.position = [1, 0.5, -0.5, 2.1]
	# state_str.velocity = []
	# state_str.effort = []

	# create a topic to publish joint states
	# pub = rospy.Publisher('joint_states', JointState, queue_size=10)

	# create a subscriber to read cmd_vel messages
	# rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)

	# rospy.spin()

	# this case is not needed a rate because it is going to be 
	# the same rate of received messages
	# rate = rospy.Rate(10)

	# while not rospy.is_shutdown():
	#	state_str.position = [1 * rospy.get_time(), 2 * rospy.get_time(), 3 * rospy.get_time(), 4 * rospy.get_time()] 
	robot_vel = inverse_kinematic()
	robot_vel.spin()


if __name__ == '__main__':
	try:
		foo()
	except rospy.ROSInterruptException:
		pass