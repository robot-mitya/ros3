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
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "consts.h"

class JoystickNode
{
public:
  JoystickNode();
private:
  const int axisX_;
  const int axisY_;
  static const float RAD_TO_DEG = 180.0f / M_PI;
  ros::Subscriber joystickSubscriber_;
  ros::Publisher drivePublisher_;
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
  int8_t getSpeedValue(float joystickValue);
};

JoystickNode::JoystickNode():
    axisX_(0),
    axisY_(1)
{
  ros::NodeHandle joystickNodeHandle;
  joystickSubscriber_ = joystickNodeHandle.subscribe<sensor_msgs::Joy>(RM_JOY_TOPIC_NAME, 10, &JoystickNode::joystickCallback, this);

  ros::NodeHandle driveNodeHandle(RM_NAMESPACE);
  drivePublisher_ = driveNodeHandle.advertise<mitya_teleop::Drive>(RM_DRIVE_TOPIC_NAME, 1000);
}

void JoystickNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float x = -joy->axes[axisX_];
  float y = joy->axes[axisY_];

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

  //ROS_INFO("x=%+5.3f y=%+5.3f R=%+5.3f A=%+8.3f    Left=%+6.3f Right=%+6.3f", x, y, radius, alpha, left, right);

  mitya_teleop::Drive msg;
  msg.left = getSpeedValue(left);
  msg.right = getSpeedValue(right);

  drivePublisher_.publish(msg);
}

int8_t JoystickNode::getSpeedValue(float joystickValue)
{
  float result = joystickValue * 100.0f;
  if (result < -100.0f) result = -100.0f;
  else if (result > 100.0f) result = 100.0f;
  return (int8_t) round(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_JOYSTICK_NODE_NAME);
  JoystickNode joystickNode;
  ros::spin();

  return 0;
}
