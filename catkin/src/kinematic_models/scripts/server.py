#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from kinematic_models.cfg import pid_velConfig

def callback(config, devel):
	rospy.loginfo(""" Reconfigure request """.format(**config))
	return config

if __name__ == '__main__':
	rospy.init_node("tutorial_dynamicPID", anonymous = False)
	srv = Server(pid_velConfig, callback)
	rospy.spin()