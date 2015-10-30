#!/usr/bin/env python
import rospy

from actionlib import SimpleActionClient, GoalStatus

from move_base_msgs.msg import *
from hector_nav_msgs.srv import GetRobotTrajectory

class ExplorationController():
	def __init__(self):
		self._plan_service = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
		self._move_base = SimpleActionClient('move_base', MoveBaseAction)

	def run(self):
		r = rospy.Rate(1 / 7.0)
		while not rospy.is_shutdown():
			self.run_once()
			r.sleep()

	def run_once(self):
		path = self._plan_service().trajectory
		poses = path.poses
		if not path.poses:
			rospy.loginfo('No frontiers left.')
			return
		rospy.loginfo('Moving to frontier....')
		self.move_to_pose(poses[-1])

	def move_to_pose(self, pose_stamped, timeout=20.0):
		goal = MoveBaseGoal()
		goal.target_pose = pose_stamped
		self._move_base.send_goal(goal)
		self._move_base.wait_for_result(rospy.Duration(timeout))
		return self._move_base.get_state() == GoalStatus.SUCCEEDED

if __name__ == '__main__':
	rospy.init_node('exploration_base_controller')
	controller = ExplorationController()
	controller.run()