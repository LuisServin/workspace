#!/usr/bin/env python

# National and Autonomous University of Mexico

import rospy
import tf
import geometry_msgs.msg

def broadcast_wheel_transform():
	br = tf.TransformBroadcaster();
	br.sendTr0ansform((0, 0, 0),
					 (0,0,0,0.001*rospy.Time.now()),
					 rospy.Time.now(),
					 "base_footprint",
					 "base_wheel_back_left")

if __name__ == '__main__':
	rospy.init_node('wheels_rotation_broadcaster')

	# no parameters specified
	
	# subscribe to /cmd_topic, main model velocity
	rospy.Subscriber('/cmd_vel', 
					 geometry_msgs.msg.Twist,
					 broadcast_wheel_transform)