#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')

import rospy
import math
# tf package provide the implementation of ros tf
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
	rospy.init_node('tf_turtle')
	# object to receive transforms easier it buffer them for
	# up to 10 seconds
	listener = tf.TransformListener()

	rospy.wait_for_service('spawn')
	spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
	spawner(4, 2, 0, 'turtle2')

	turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

	# line code added later to show a new feature
	# last argument is the maximun waiting time
	listener.waitForTransform("/turtle2", "/carrot1", rospy.Time(), rospy.Duration(4.0))

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			now = rospy.Time.now() - rospy.Duration(5.0)
			listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(6.0))

			# assign transformation listener object to a specific transformation
			# from frame /turtle2 to frame /turtle1 and the time we want the transform
			# rospy.Time(0) will return the last transform.
			(trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', now)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) :
			continue

		angular = 4 * math.atan2(trans[1], trans[0])
		linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
		cmd = geometry_msgs.msg.Twist()
		cmd.linear.x = linear
		cmd.angular.z = angular
		turtle_vel.publish(cmd)

		rate.sleep()