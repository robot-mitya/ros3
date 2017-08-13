/*
 * joystick_node.cpp
 * Copyright (c) 2017, Robot Mitya.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Mitya nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Apr 22, 2017
 *      Author: Dmitry Dzakhov
 */

#include "ros/ros.h"
#include "mitya_teleop/Drive.h"
#include "mitya_teleop/HeadPosition.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "consts.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

class JoystickNode
{
public:
  JoystickNode();
private:
  static const float RAD_TO_DEG = 180.0f / M_PI;

  int driveAxisX_;
  int driveAxisY_;
  int headAxisX_;
  int headAxisY_;

  int driveMaxValue;
  bool driveInvertX;
  bool driveInvertY;
  float driveSignX;
  float driveSignY;

  float headHorizontalMinDegree;
  float headHorizontalCenterDegree;
  float headHorizontalMaxDegree;
  float headVerticalMinDegree;
  float headVerticalCenterDegree;
  float headVerticalMaxDegree;
  bool headInvertHorizontal;
  bool headInvertVertical;
  float headHorizontalAmplitude;
  float headVerticalAmplitude;

  ros::Subscriber joystickSubscriber_;
  ros::Publisher drivePublisher_;
  ros::Publisher headPositionPublisher_;
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
  int8_t getSpeedValue(float joystickValue);
  void publishDriveMessage(float x, float y);
  void publishHeadPositionMessage(float x, float y);
};

JoystickNode::JoystickNode()
{
  ros::NodeHandle joystickNodeHandle;
  joystickSubscriber_ = joystickNodeHandle.subscribe<sensor_msgs::Joy>(RM_JOY_TOPIC_NAME, 10, &JoystickNode::joystickCallback, this);

  ros::NodeHandle driveNodeHandle(RM_NAMESPACE);
  drivePublisher_ = driveNodeHandle.advertise<mitya_teleop::Drive>(RM_DRIVE_TOPIC_NAME, 1000);

  ros::NodeHandle headPositionNodeHandle(RM_NAMESPACE);
  headPositionPublisher_ = headPositionNodeHandle.advertise<mitya_teleop::HeadPosition>(RM_HEAD_POSITION_TOPIC_NAME, 1000);

  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("drive_axis_x", driveAxisX_, 0);
  privateNodeHandle.param("drive_axis_y", driveAxisY_, 1);
  privateNodeHandle.param("drive_max_value", driveMaxValue, 100);
  privateNodeHandle.param("drive_invert_x", driveInvertX, true);
  privateNodeHandle.param("drive_invert_y", driveInvertY, false);
  driveSignX = driveInvertX ? -1.0f : 1.0f;
  driveSignY = driveInvertY ? -1.0f : 1.0f;

  privateNodeHandle.param("head_axis_x", headAxisX_, 3);
  privateNodeHandle.param("head_axis_y", headAxisY_, 4);
  privateNodeHandle.param("head_invert_horizontal", headInvertHorizontal, true);
  privateNodeHandle.param("head_invert_vertical", headInvertVertical, true);

  privateNodeHandle.param("head_horizontal_min_degree", headHorizontalMinDegree, -120.0f);
  privateNodeHandle.param("head_horizontal_center_degree", headHorizontalCenterDegree, 0.0f);
  privateNodeHandle.param("head_horizontal_max_degree", headHorizontalMaxDegree, 120.0f);
  privateNodeHandle.param("head_vertical_min_degree", headVerticalMinDegree, -120.0f);
  privateNodeHandle.param("head_vertical_center_degree", headVerticalCenterDegree, -15.0f);
  privateNodeHandle.param("head_vertical_max_degree", headVerticalMaxDegree, 10.0f);

  headHorizontalAmplitude = MAX(
      abs(headHorizontalMinDegree - headHorizontalCenterDegree),
      abs(headHorizontalMaxDegree - headHorizontalCenterDegree));
  headVerticalAmplitude = MAX(
      abs(headVerticalMinDegree - headVerticalCenterDegree),
      abs(headVerticalMaxDegree - headVerticalCenterDegree));

  if (headInvertHorizontal)
    headHorizontalAmplitude *= -1.0f;
  if (headInvertVertical)
    headVerticalAmplitude *= -1.0f;

  std::string testValue;
  std::string defaultValue = "Default value";
  privateNodeHandle.param("test", testValue, defaultValue);
  ROS_INFO("test=%s", testValue.c_str());
}

void JoystickNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  publishDriveMessage(joy->axes[driveAxisX_], joy->axes[driveAxisY_]);
  publishHeadPositionMessage(joy->axes[headAxisX_], joy->axes[headAxisY_]);
}

int8_t JoystickNode::getSpeedValue(float joystickValue)
{
  int8_t result = (int8_t) round(joystickValue * driveMaxValue);
  if (result < -driveMaxValue) return -driveMaxValue;
  else if (result > driveMaxValue) return driveMaxValue;
  return result;
}

void JoystickNode::publishDriveMessage(float x, float y)
{
  x *= driveSignX;
  y *= driveSignY;

  float alpha = atan2(y, x) * RAD_TO_DEG;
  if (alpha < 0) alpha += 360;
  if (alpha < 0) alpha += 360;
  else if (alpha >= 360) alpha -= 360;

  float radius = sqrt(x * x + y * y);
  if (radius < 0) radius = 0.0f;
  else if (radius > 1) radius = 1.0f;

  float left = 0;
  float right = 0;
  if (alpha >= 0 && alpha < 90)
  {
    left = 1.0f;
    right = 2.0f / 90.0f * alpha - 1.0f;
  }
  else if (alpha >= 90 && alpha < 180)
  {
    left = -2.0f / 90.0f * alpha + 3.0f;
    right = 1.0f;
  }
  else if (alpha >= 180 && alpha < 270)
  {
    left = -1.0f;
    right = - 2.0f / 90.0f * alpha + 5.0f;
  }
  else
  {
    left = 2.0f / 90.0f * alpha - 7.0f;
    right = -1.0f;
  }
  left *= radius;
  right *= radius;

  //ROS_DEBUG("x=%+5.3f y=%+5.3f R=%+5.3f A=%+8.3f    Left=%+6.3f Right=%+6.3f", x, y, radius, alpha, left, right);

  mitya_teleop::Drive msg;
  msg.left = getSpeedValue(left);
  msg.right = getSpeedValue(right);
  drivePublisher_.publish(msg);
}

void JoystickNode::publishHeadPositionMessage(float x, float y)
{
  mitya_teleop::HeadPosition msg;

  x *= headHorizontalAmplitude;
  x += headHorizontalCenterDegree;
  if (x < headHorizontalMinDegree) x = headHorizontalMinDegree;
  else if (x > headHorizontalMaxDegree) x = headHorizontalMaxDegree;
  msg.horizontal = x;

  y *= headVerticalAmplitude;
  y += headVerticalCenterDegree;
  if (y < headVerticalMinDegree) y = headVerticalMinDegree;
  else if (y > headVerticalMaxDegree) y = headVerticalMaxDegree;
  msg.vertical = y;

  headPositionPublisher_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_JOYSTICK_NODE_NAME);
  JoystickNode joystickNode;
  ros::spin();

  return 0;
}
