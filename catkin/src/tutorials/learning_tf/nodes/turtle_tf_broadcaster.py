#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
	br = tf.TransformBroadcaster();
	br.sendTransform((msg.x, msg.y, 0),
					  tf.transformations.quaternion_from_euler(0, 0, msg.theta),
					  rospy.Time.now(),
					  "base_footprint",
					  "world")

if __name__ == '__main__':
	rospy.init_node('turtle_tf_broadcaster')

	# takes a parameter to specify turtle name
	turtlename = rospy.get_param('~turtle')

	# function subscribe to the topic '/turtleX/pose' and runs
	# the function handle_turtle_pose
	rospy.Subscriber('/%s/pose' % turtlename,
					 turtlesim.msg.Pose,
					 handle_turtle_pose,
					 turtlename)
	rospy.spin()