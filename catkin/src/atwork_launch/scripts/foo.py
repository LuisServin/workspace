#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
	rospy.init_node('joint_state_publisher')

	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(10)

	state_str = JointState()
	state_str.header = Header()
	state_str.header.stamp = rospy.Time.now()
	
	state_str.name = ['base_footprint_to_base_wheel', 'base_footprint_to_base_wheel_front_right', 'base_footprint_to_base_wheel_front_left', 'base_footprint_to_base_wheel_back_right']
	state_str.position = [1, 0.5, -0.5, 2.1]
	state_str.velocity = []
	state_str.effort = []

	while not rospy.is_shutdown():
		state_str.position = [1 * rospy.get_time(), 2 * rospy.get_time(), 3 * rospy.get_time(), 4 * rospy.get_time()] 
		state_str.header.stamp = rospy.Time.now()
		pub.publish(state_str)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass