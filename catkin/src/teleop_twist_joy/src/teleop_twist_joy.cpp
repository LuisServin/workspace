/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  //* modification to enable omnidireccional movement
  int enable_omnidireccional;
  int enable_omnidireccional_button;

  //* 
  int enable_turbo;
  int enable_button;
  int enable_turbo_button;
  int axis_linear_x;

  //* enable an axis_linear_y
  int axis_linear_y;

  int axis_angular;
  double scale_linear;
  double scale_linear_turbo;
  double scale_angular;
  //* Added to have a turn turbo mode
  double scale_angular_turbo;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);

  //* param for omnidireccional movement
  //* value -1 is the default value, if 1 enable onmidireccional
  nh_param->param<int>("enable_omnidireccional", pimpl_->enable_omnidireccional, -1);
  nh_param->param<int>("enable_omnidireccional_button", pimpl_->enable_omnidireccional_button, 5);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo", pimpl_->enable_turbo, -1);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  nh_param->param<int>("axis_linear_x", pimpl_->axis_linear_x, 1);
  //* get the param for the axis linear y
  nh_param->param<int>("axis_linear_y", pimpl_->axis_linear_y, 0);
  nh_param->param<double>("scale_linear", pimpl_->scale_linear, 0.5);
  nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_turbo, 1.0);

  nh_param->param<int>("axis_angular", pimpl_->axis_angular, 0);
  nh_param->param<double>("scale_angular", pimpl_->scale_angular, 1.0);
  nh_param->param<double>("scale_angular_turbo", pimpl_->scale_angular_turbo, 2.0);

  ROS_INFO_NAMED("TeleopTwistJoy", "Using axis %i for linear and axis %i for angular.",
      pimpl_->axis_linear_x, pimpl_->axis_angular);
  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop on button %i at scale %f linear, scale %f angular.",
      pimpl_->enable_button, pimpl_->scale_linear, pimpl_->scale_angular);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i at scale %f linear.", pimpl_->enable_turbo_button, pimpl_->scale_linear_turbo);

  pimpl_->sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  if (enable_omnidireccional >= 0 && joy_msg->buttons[enable_omnidireccional_button]) 
  {
    cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_x] * scale_linear_turbo;
    cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_y] * scale_linear_turbo;
    cmd_vel_msg.angular.z = joy_msg->axes[axis_angular] * scale_angular_turbo;
    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else if (enable_turbo >= 0 && joy_msg->buttons[enable_turbo_button])
  {
    cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_x] * scale_linear_turbo;
    cmd_vel_msg.angular.z = joy_msg->axes[axis_angular] * scale_angular_turbo ;
    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else if (joy_msg->buttons[enable_button])
  {
    cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_x] * scale_linear;
    cmd_vel_msg.angular.z = joy_msg->axes[axis_angular] * scale_angular;
    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
