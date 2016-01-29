#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')

import rospy
# package to use tf transform library
import tf
import math

if __name__ == '__main__':
	rospy.init_node('my_tf_broadcaster')
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		# this line was added later to show functioning of a moving frame
		t = rospy.Time.now().to_sec() * math.pi
		# here were creating a new fixed transform with "turtle1"
		# as parent and "carrot1" as child
		br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),
						 (0.0, 0.0, 0.0, 1.0),
						 rospy.Time.now(),
						 "carrot1",
						 "turtle1")
		rate.sleep()