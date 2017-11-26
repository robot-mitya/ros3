/*
 * test_herkulex_node.cpp
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
 *  Created on: Nov 24, 2017
 *      Author: Dmitry Dzakhov
 */

#include "ros/ros.h"
#include "consts.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include "herkulex.h"

#define SERVO_ID 0x01
#define CORRECTION_DURATION 0
#define SPEED 85

class TestHerkulexNode
{
public:
  TestHerkulexNode();
  void logPosition();
private:
  HerkulexClass herkulex_;

  std::string serialPortName_;
  int serialBaudRate_;

  int headAxisX_;
  float headHorizontalAmplitude_;
  float headHorizontalCenterDegree_;
  float headHorizontalMinDegree_;
  float headHorizontalMaxDegree_;

  float prevAngle_;
  float prevReadAngle_;
  bool prevBackButtonState_;
  bool prevStartButtonState_;
  bool prevUpButtonState_;
  bool prevDownButtonState_;
  bool prevLeftButtonState_;
  bool prevRightButtonState_;
  bool prevPowerButtonState_;

  ros::Subscriber joystickSubscriber_;
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);

  int calculateDurationInMillis(float deltaAngle, float degreesPerSecond);
};

TestHerkulexNode::TestHerkulexNode()
{
  headAxisX_ = 0;
  headHorizontalCenterDegree_ = 0;
  headHorizontalMinDegree_ = -120;
  headHorizontalMaxDegree_ = 120;
  headHorizontalAmplitude_ = 120;
  prevAngle_ = 0;
  prevReadAngle_ = 0;
  prevBackButtonState_ = false;
  prevStartButtonState_ = false;
  prevUpButtonState_ = false;
  prevDownButtonState_ = false;
  prevLeftButtonState_ = false;
  prevRightButtonState_ = false;
  prevPowerButtonState_ = false;

  ros::NodeHandle joystickNodeHandle;
  joystickSubscriber_ = joystickNodeHandle.subscribe<sensor_msgs::Joy>(RM_JOY_TOPIC_NAME, 10, &TestHerkulexNode::joystickCallback, this);

  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("serial_port", serialPortName_, (std::string) "/dev/ttyUSB0");
  privateNodeHandle.param("baud_rate", serialBaudRate_, 115200);

  bool portOpened = herkulex_.begin(serialPortName_.c_str(), serialBaudRate_);
  if (portOpened)
    ROS_INFO("Serial port \'%s\' is opened", serialPortName_.c_str());
  else
    ROS_ERROR("Error %d opening %s: %s", errno, serialPortName_.c_str(), strerror(errno));
  herkulex_.initialize();
}

void TestHerkulexNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float angle = joy->axes[headAxisX_];
  angle *= headHorizontalAmplitude_;
  angle += headHorizontalCenterDegree_;
  if (angle < headHorizontalMinDegree_) angle = headHorizontalMinDegree_;
  else if (angle > headHorizontalMaxDegree_) angle = headHorizontalMaxDegree_;

  if (angle != prevAngle_)
  {
    ROS_INFO("x: %.3f", angle);
    //herkulex_.moveOneAngle(SERVO_ID, angle, 0, 0);
    prevAngle_ = angle;
  }

  //-------------------
  bool backButtonState = joy->buttons[6] == 1;
  if (backButtonState && !prevBackButtonState_)
  {
    ROS_INFO("Back was pressed");
    herkulex_.torqueOFF(SERVO_ID);
  }
  prevBackButtonState_ = backButtonState;

  bool startButtonState = joy->buttons[7] == 1;
  if (startButtonState && !prevStartButtonState_)
  {
    ROS_INFO("Start was pressed");
    herkulex_.torqueON(SERVO_ID);
  }
  prevStartButtonState_ = startButtonState;

  //-------------------
  bool upButtonState = joy->axes[7] > 0;
  if (upButtonState && !prevUpButtonState_)
  {
    ROS_INFO("Up was pressed");
    herkulex_.moveOneAngle(SERVO_ID, headHorizontalCenterDegree_, 0, 0);
  }
  prevUpButtonState_ = upButtonState;

  bool downButtonState = joy->axes[7] < 0;
  if (downButtonState && !prevUpButtonState_)
  {
    ROS_INFO("Down was pressed");
    herkulex_.moveOneAngle(SERVO_ID, 45, 0, 0);
  }
  prevUpButtonState_ = downButtonState;

  //-------------------
  bool leftButtonState = joy->axes[6] > 0;
  if (leftButtonState && !prevLeftButtonState_)
  {
    ROS_INFO("Left was pressed");
    float currentAngle = herkulex_.getAngle(SERVO_ID);
    float targetAngle = headHorizontalMaxDegree_;
    int duration = calculateDurationInMillis(targetAngle - currentAngle, SPEED);
    herkulex_.moveOneAngle(SERVO_ID, targetAngle, duration, 0);
  }
  else if (!leftButtonState && prevLeftButtonState_)
  {
//    float currentAngle = herkulex_.getAngle(SERVO_ID);
//    ROS_INFO("++ Left was released (currentAngle=%.3f)", currentAngle);
//    herkulex_.moveOneAngle(SERVO_ID, currentAngle, 0, 0);
    herkulex_.moveOneAngle(SERVO_ID, herkulex_.getAngle(SERVO_ID), CORRECTION_DURATION, 0);
  }
  prevLeftButtonState_ = leftButtonState;

  bool rightButtonState = joy->axes[6] < 0;
  if (rightButtonState && !prevRightButtonState_)
  {
    ROS_INFO("Right was pressed");
    float currentAngle = herkulex_.getAngle(SERVO_ID);
    float targetAngle = headHorizontalMinDegree_;
    int duration = calculateDurationInMillis(targetAngle - currentAngle, SPEED);
    herkulex_.moveOneAngle(SERVO_ID, targetAngle, duration, 0);
  }
  else if (!rightButtonState && prevRightButtonState_)
  {
//    float currentAngle = herkulex_.getAngle(SERVO_ID);
//    ROS_INFO("++ Right was released (currentAngle=%.3f)", currentAngle);
//    herkulex_.moveOneAngle(SERVO_ID, currentAngle, 0, 0);
    herkulex_.moveOneAngle(SERVO_ID, herkulex_.getAngle(SERVO_ID), CORRECTION_DURATION, 0);
  }
  prevRightButtonState_ = rightButtonState;

  //-------------------
  bool powerButtonState = joy->buttons[8] != 0;
  if (powerButtonState && !prevPowerButtonState_)
  {
    ROS_INFO("Power was pressed");
    //herkulex_.reboot(SERVO_ID);
    float currentAngle = herkulex_.getAngle(SERVO_ID);
    ROS_INFO("++ currentAngle=%.3f", currentAngle);

  }
  prevPowerButtonState_ = powerButtonState;
}

void TestHerkulexNode::logPosition()
{
  float angle = herkulex_.getAngle(SERVO_ID);
  if (angle != prevReadAngle_)
  {
    ROS_INFO("Read angle: %.3f", angle);
    prevReadAngle_ = angle;
  }
}

int TestHerkulexNode::calculateDurationInMillis(float deltaAngle, float degreesPerSecond)
{
  //todo calculate 85
  // 85 is the max speed. Keep in mind that the longest single movement takes 2856 mS.
  // The longest movement in our case is 240°: abs(headHorizontalMaxDegree_ - headHorizontalMinDegree_).
  if (degreesPerSecond < 85)
    degreesPerSecond = 85;

  int result = (int)(deltaAngle * 1000.0f / degreesPerSecond);
  return result >= 0 ? result : -result;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_herkulex_node");
  TestHerkulexNode testHerkulexNode;
  ros::Rate loop_rate(100); // 100 Hz
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
    testHerkulexNode.logPosition();
  }

  return 0;
}
