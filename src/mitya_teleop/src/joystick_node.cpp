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

class JoystickNode
{
public:
  JoystickNode();
private:
  const int axisX_;
  const int axisY_;
  ros::Subscriber joystickSubscriber_;
  ros::Publisher drivePublisher_;
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

JoystickNode::JoystickNode():
    axisX_(1),
    axisY_(2)
{
  ros::NodeHandle joystickNodeHandle;
  joystickSubscriber_ = joystickNodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickNode::joystickCallback, this);

  ros::NodeHandle driveNodeHandle("mitya");
  drivePublisher_ = driveNodeHandle.advertise<mitya_teleop::Drive>("drive", 1000);
}

void JoystickNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  mitya_teleop::Drive msg;
  msg.left = 19;
  msg.right = 74;

  ROS_INFO("%d %d", msg.left, msg.right);

  drivePublisher_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_node");
  JoystickNode joystickNode;
  ros::spin();

  return 0;
}
