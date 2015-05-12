#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtle_actionlib/ShapeAction.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_shape");

	// create the action client
	// true causes the client to spin it's own thread
	actionlib::SimpleActionClient<
		turtle_actionlib::ShapeAction
		> ac("turtle_shape", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer();

	ROS_INFO("Action server started, sending goal.");
	// send goal to the action
	turtle_actionlib::ShapeGoal goal;
	goal.edges = 6;
	goal.radius = 2.3;
	ac.sendGoal(goal);

	// wait for the action to return
	bool finished_before_timeout = 
		ac.waitForResult(ros::Duration(40.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();		
	} else {
		ROS_INFO("Action did not finish before the time out");
	}

	return 0;
}