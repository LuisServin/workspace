#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
import learning_actionlib.msg

def fibonacci_client():
	# creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor
	client = actionlib.SimpleActionClient('fibonacci',
		learning_actionlib.msg.FibonacciAction)

	# waits until the action server has started up and started
	# listening for goals
	client.wait_for_server()

	# creates a goal to send to the action server
	goal = learning_actionlib.msg.FibonacciGoal(order=20)

	# Sends the goal to the action server.
	client.send_goal(goal)

	# waits for the server to finish performing the action
	client.wait_for_result()

	# prints out the result for executing the action
	return client.get_result() # A FibonacciResult

if __name__ == '__main__':
	try:
		# initialize a rospy node so that the SimpleActionClient
		# can publish and subscribe over ROS
		rospy.init_node('fibonacci_client_py')
		result = fibonacci_client()
		print "Result:", ', '.join([str(n) for n in result.sequence])
	except rospy.ROSInterruptException:
		print "program interrupted before completition"
