#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Led.h>

namespace kobuki 
{
	/**
	 * @brief A simple bump-blink-controller
	 * 
	 * A simple nodelet-based controller for kobuki, which makes one
	 * of kobuki's LED's blink, when a bumper is pressed
	 */
class BumpBlinkController : public yocs::Controller
{
public:
	BumpBlinkController(ros::NodeHandle& nh, std::string& name) :
		Controller(),
		nh_(nh),
		name_(name) {};

	~BumpBlinkController() {};

	/**
	 * Set-up neccesary publisher/subscriber
	 * @return true, if successful
	 */

	 bool init() {
	 	enable_controller_subscriber_ = nh_.subscribe(
	 		"enable", 10, &BumpBlinkController::enableCB, this);
	 	disable_controller_subscriber_ = nh_.subscribe(
	 		"disable", 10, &BumpBlinkController::disableCB, this);
	 	bumper_event_subscriber_ = nh_.subscribe(
	 		"events/bumper", 10, 
	 		&BumpBlinkController::bumperEventCB, this);

	 	return true;
	 }

private:
	ros::NodeHandle nh_;
	std::string name_;
	ros::Subscriber enable_controller_subscriber_,
					disable_controller_subscriber_,
					bumper_event_subscriber_;
	ros::Publisher blink_publisher_;

	/**
	 * @brief ROS logging output for enabling the controller
	 * @param msg incoming topic message
	 */
	void enableCB(const std_msgs::EmptyConstPtr msg);

	/**
	 * @brief ROS logging output for disabling the controller
	 * @param msg incoming topic message
	 */
	 void disableCB(const std_msgs::EmptyConstPtr msg);

	 /**
	 * @brief Turn on/off a LED, when a bumper is pressed/released
	 * @param msg incoming topic message
	 */
	 void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);
};

void BumpBlinkController::enableCB(
	const std_msgs::EmptyConstPtr msg) {

	if (this->enable())
		ROS_INFO_STREAM("Controller has been enabled. [" <<
			name_ << "]");
	else
		ROS_INFO_STREAM("Controller was already enabled. " <<
			name_ << "]");
}

void BumpBlinkController::disableCB(
	const std_msgs::EmptyConstPtr msg) {

	if (this->disable()) 
		ROS_INFO_STREAM("Controller has been disabled. [" <<
			name_ << "]");
	else
		ROS_INFO_STREAM("Controller was already disabled. [" <<
			name_ << "]");
}

void BumpBlinkController::bumperEventCB(
	const kobuki_msgs::BumperEventConstPtr msg) {

	// check if the controller is active
	if (this->getState()) {

		// Preparing LED message
		kobuki_msgs::LedPtr led_msg_prt;
		led_msg_prt.reset(new kobuki_msgs::Led());

		if(msg->state == kobuki_msgs::BumperEvent::PRESSED) {
			ROS_INFO_STREAM("Bumper pressed. Turning LED on. [" <<
				name_ << "]");	
			led_msg_prt->value = kobuki_msgs::Led::GREEN;
			blink_publisher_.publish(led_msg_prt);
		} else {
			ROS_INFO_STREAM("Bumper released. Turning LED off [" <<
				name_ << "]");
			led_msg_prt->value = kobuki_msgs::Led::BLACK;
			blink_publisher_.publish(led_msg_prt);
		}
	}
}

} // namespace kobuki