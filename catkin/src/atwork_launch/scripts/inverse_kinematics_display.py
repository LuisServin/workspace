#!/usr/bin/env python

# National and Autonomous University of Mexico

import rospy
import tf
import geometry_msgs.msg

def broadcast_wheel_transform(msg):
	br = tf.TransformBroadcaster();
	br.sendTransform((-0.8, 1.0, 0),
					 tf.transformations.quaternion_from_euler(0, 0.765, 0),
					 rospy.Time.now(),
					 "base_wheel_back_left",
					 "base_footprint")

if __name__ == '__main__':
	rospy.init_node('wheels_rotation_broadcaster')

	# no parameters specified
	
	# subscribe to /cmd_topic, main model velocity
	rospy.Subscriber('/cmd_vel', 
					 geometry_msgs.msg.Twist,
					 broadcast_wheel_transform)

	rospy.spin()