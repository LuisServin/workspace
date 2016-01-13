#include <ros/ros.h>
// provides an implementation of a transform broadcaster
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
	// transform broadcaster object for sending information
	static tf::TransformBroadcaster br;
	// transform object and copy information from turtle 2D to 3D
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transform.setRotation(q);
	// send a transform object. argumetns:
	// transform object, time for the timestamp, name of the parent frame, name of the child frame
	br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_tf_broadcaster");
	if(argc != 2) {
		ROS_ERROR("need turtle name as argument");
		return -1;
	}
	turtle_name = argv[1];

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name + "/pose", 10, &poseCallback);

	ros::spin();
	return 0;
}