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

#include <map>
#include <string>
#include <algorithm>


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
  void readParams();

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  int reread_parameters_button;
  bool reread_parameters_button_pressed;

  int enable_button;
  int enable_turbo_button;

  static void scaleUp(std::pair<const std::string, double> &entry);
  static void scaleDown(std::pair<const std::string, double> &entry);
  template <class ScaleFunction>
  static void handleScaleButton(
      int button,
      const sensor_msgs::Joy::ConstPtr& joy_msg,
      bool &button_pressed,
      std::map<std::string, double> &scale_map,
      ScaleFunction f);

  int scale_linear_up_button;
  int scale_linear_down_button;
  int scale_angular_up_button;
  int scale_angular_down_button;
  bool scale_linear_up_button_pressed;
  bool scale_linear_down_button_pressed;
  bool scale_angular_up_button_pressed;
  bool scale_angular_down_button_pressed;

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, double> scale_linear_map;
  std::map<std::string, double> scale_linear_turbo_map;

  std::map<std::string, int> axis_angular_map;
  std::map<std::string, double> scale_angular_map;
  std::map<std::string, double> scale_angular_turbo_map;

  bool sent_disable_msg;

  double inactivity_timeout;
  void restart_inactivity_timer(void);
  void stop_inactivity_timer(void);
  ros::Timer timer;
  void inactivityTimerCallback(const ros::TimerEvent& e);

  ros::NodeHandle *nh;
  ros::NodeHandle *nh_param;
};

void TeleopTwistJoy::Impl::readParams()
{
  nh_param->param<int>("reread_parameters_button", reread_parameters_button, -1);

  nh_param->param<int>("enable_button", enable_button, 0);
  nh_param->param<int>("enable_turbo_button", enable_turbo_button, -1);

  nh_param->param<int>("scale_linear_up_button", scale_linear_up_button, -1);
  nh_param->param<int>("scale_linear_down_button", scale_linear_down_button, -1);
  nh_param->param<int>("scale_angular_up_button", scale_angular_up_button, -1);
  nh_param->param<int>("scale_angular_down_button", scale_angular_down_button, -1);

  axis_linear_map.clear();
  scale_linear_map.clear();
  scale_linear_turbo_map.clear();
  if (nh_param->getParam("axis_linear", axis_linear_map))
  {
    nh_param->getParam("axis_linear", axis_linear_map);
    nh_param->getParam("scale_linear", scale_linear_map);
    nh_param->getParam("scale_linear_turbo", scale_linear_turbo_map);
  }
  else
  {
    nh_param->param<int>("axis_linear", axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", scale_linear_map["x"], 0.5);
    nh_param->param<double>("scale_linear_turbo", scale_linear_turbo_map["x"], 1.0);
  }

  axis_angular_map.clear();
  scale_angular_map.clear();
  scale_angular_turbo_map.clear();
  if (nh_param->getParam("axis_angular", axis_angular_map))
  {
    nh_param->getParam("axis_angular", axis_angular_map);
    nh_param->getParam("scale_angular", scale_angular_map);
    nh_param->getParam("scale_angular_turbo", scale_angular_turbo_map);
  }
  else
  {
    nh_param->param<int>("axis_angular", axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", scale_angular_map["yaw"], 0.5);
    nh_param->param<double>("scale_angular_turbo",
        scale_angular_turbo_map["yaw"], scale_angular_map["yaw"]);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", enable_button);

  // if the inactivity_timeout is <= 0, it is disabled
  nh_param->param<double>("inactivity_timeout", inactivity_timeout, -1.0);
  // if the inactivity_timout is enabled, create a one-shot timer that is stopped
  if (inactivity_timeout > 0.0)
  {
    timer = nh->createTimer(ros::Duration(inactivity_timeout), &TeleopTwistJoy::Impl::inactivityTimerCallback, this, true, false);
  }
  else
  {
    // if there was a previous timer, ensure it is stopped
    timer.stop();
  }
}

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->nh = nh;
  pimpl_->nh_param = nh_param;

  pimpl_->reread_parameters_button_pressed=false;

  pimpl_->scale_linear_up_button_pressed=false;
  pimpl_->scale_linear_down_button_pressed=false;
  pimpl_->scale_angular_up_button_pressed=false;
  pimpl_->scale_angular_down_button_pressed=false;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
  pimpl_->readParams();

  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map[it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_turbo_map[it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map[it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_turbo_map[it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::scaleUp(std::pair<const std::string, double> &entry)
{
  entry.second *= 1.10;
}

void TeleopTwistJoy::Impl::scaleDown(std::pair<const std::string, double> &entry)
{
  entry.second *= (1/1.10);
}

template <class ScaleFunction>
void TeleopTwistJoy::Impl::handleScaleButton(
    int button,
    const sensor_msgs::Joy::ConstPtr& joy_msg,
    bool &button_pressed,
    std::map<std::string, double> &scale_map,
    ScaleFunction f)
{
  if (button >= 0)
  {
    if (joy_msg->buttons[button])
    {
      if (!button_pressed)
      {
        std::for_each(scale_map.begin(), scale_map.end(), f);
      }
      button_pressed=true;
    }
    else
    {
      button_pressed=false;
    }
  }
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  // If the re-initialize button is pressed, re-read all parameters
  if (reread_parameters_button >= 0)
  {
    if (joy_msg->buttons[reread_parameters_button])
    {
      if (!reread_parameters_button_pressed)
      {
        readParams();
      }
      reread_parameters_button_pressed=true;
    }
    else
    {
      reread_parameters_button_pressed=false;
    }
  }

  if (enable_turbo_button >= 0 && joy_msg->buttons[enable_turbo_button])
  {
    if (axis_linear_map.find("x") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_turbo_map["x"];
    }
    if (axis_linear_map.find("y") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_turbo_map["y"];
    }
    if  (axis_linear_map.find("z") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_turbo_map["z"];
    }
    if  (axis_angular_map.find("yaw") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_turbo_map["yaw"];
    }
    if  (axis_angular_map.find("pitch") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_turbo_map["pitch"];
    }
    if  (axis_angular_map.find("roll") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_turbo_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
    restart_inactivity_timer();
  }
  else if (joy_msg->buttons[enable_button])
  {
    if  (axis_linear_map.find("x") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_map["x"];
    }
    if  (axis_linear_map.find("y") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_map["y"];
    }
    if  (axis_linear_map.find("z") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_map["z"];
    }
    if  (axis_angular_map.find("yaw") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_map["yaw"];
    }
    if  (axis_angular_map.find("pitch") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_map["pitch"];
    }
    if  (axis_angular_map.find("roll") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
    restart_inactivity_timer();
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
      stop_inactivity_timer();
    }
  }

  // Handle scale up/down buttons
  handleScaleButton(scale_linear_up_button, joy_msg, scale_linear_up_button_pressed, scale_linear_map, scaleUp);
  handleScaleButton(scale_linear_down_button, joy_msg, scale_linear_down_button_pressed, scale_linear_map, scaleDown);
  handleScaleButton(scale_angular_up_button, joy_msg, scale_angular_up_button_pressed, scale_angular_map, scaleUp);
  handleScaleButton(scale_angular_down_button, joy_msg, scale_angular_down_button_pressed, scale_angular_map, scaleDown);
}

void TeleopTwistJoy::Impl::restart_inactivity_timer(void)
{
  if (inactivity_timeout > 0.0)
  {
    timer.stop();
    timer.setPeriod(ros::Duration(inactivity_timeout));
    timer.start();
  }
}

void TeleopTwistJoy::Impl::stop_inactivity_timer(void)
{
  if (inactivity_timeout > 0.0)
  {
    timer.stop();
  }
}

void TeleopTwistJoy::Impl::inactivityTimerCallback(const ros::TimerEvent& e)
{
  if (!sent_disable_msg)
  {
    geometry_msgs::Twist cmd_vel_msg;
    ROS_INFO_NAMED("TeleopTwistJoy", "Joystick timed out, stopping motion");
    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = true;
  }
}

}  // namespace teleop_twist_joy
