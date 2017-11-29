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
#include "mitya_teleop/HeadMove.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "consts.h"
#include "robo_com.h"
#include "button_event.h"
#include "yaml-cpp/yaml.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

enum HeadControlMode { HEAD_MOVE, HEAD_POSITION };

class JoystickNode
{
public:
  JoystickNode();
private:
  static const float RAD_TO_DEG = 180.0f / M_PI;

  HeadControlMode headControlMode_;

  int rebootButtonIndex_;

  int driveAxisX_;
  int driveAxisY_;
  int driveBoost_;
  bool driveBoostWasChanged_; // (To fix a bug in joy node - bad initial values in triggers' axes)
  float driveBoostMinFactor_;
  float driveBoostMaxFactor_;

  int headModeButtonIndex_;
  int headAxisX_;
  int headAxisY_;
  int headMoveHorizontalAxis_;
  int headMoveVerticalAxis_;
  int headMoveCenterButtonIndex_;

  int led1ButtonIndex_;
  int led2ButtonIndex_;
  int tailButtonIndex_;

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

  ButtonEvent *rebootButton_;
  void rebootButtonHandler(bool state);

  ButtonEvent *led1Button_;
  void led1ButtonHandler(bool state);
  ButtonEvent *led2Button_;
  void led2ButtonHandler(bool state);
  ButtonEvent *tailButton_;
  void tailButtonHandler(bool state);

  ButtonEvent *headModeButton_;
  void headModeButtonHandler(bool state);

  ButtonEvent *headMoveLeftButton_;
  void headMoveLeftButtonHandler(bool state);
  ButtonEvent *headMoveRightButton_;
  void headMoveRightButtonHandler(bool state);
  ButtonEvent *headMoveUpButton_;
  void headMoveUpButtonHandler(bool state);
  ButtonEvent *headMoveDownButton_;
  void headMoveDownButtonHandler(bool state);
  ButtonEvent *headMoveCenterButton_;
  void headMoveCenterButtonHandler(bool state);

  ros::Subscriber joystickSubscriber_;
  ros::Publisher drivePublisher_;
  ros::Publisher headPositionPublisher_;
  ros::Publisher headMovePublisher_;
  ros::Publisher arduinoInputPublisher_;
  ros::Publisher herkulexInputPublisher_;
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
  int8_t getSpeedValue(float joystickValue);
  void publishDriveMessage(float x, float y, float boost);
  void publishHeadPositionMessage(float x, float y);
  void publishSwitchLed1Message();
  void publishSwitchLed2Message();
  void publishSwingTailMessage();
  void publishCenterHerkulex(uint8_t address);
  void publishRebootHerkulex();

  mitya_teleop::Drive driveMessage_;
  mitya_teleop::HeadMove headMoveMessage_;
  mitya_teleop::HeadPosition headPositionMessage_;
};

JoystickNode::JoystickNode()
{
  headControlMode_ = HEAD_MOVE;

  ros::NodeHandle joystickNodeHandle;
  joystickSubscriber_ = joystickNodeHandle.subscribe<sensor_msgs::Joy>(RM_JOY_TOPIC_NAME, 10, &JoystickNode::joystickCallback, this);

  ros::NodeHandle driveNodeHandle(RM_NAMESPACE);
  drivePublisher_ = driveNodeHandle.advertise<mitya_teleop::Drive>(RM_DRIVE_TOPIC_NAME, 1000);

  ros::NodeHandle headPositionNodeHandle(RM_NAMESPACE);
  headPositionPublisher_ = headPositionNodeHandle.advertise<mitya_teleop::HeadPosition>(RM_HEAD_POSITION_TOPIC_NAME, 1000);

  ros::NodeHandle headMoveNodeHandle(RM_NAMESPACE);
  headMovePublisher_ = headMoveNodeHandle.advertise<mitya_teleop::HeadMove>(RM_HEAD_MOVE_TOPIC_NAME, 1000);

  ros::NodeHandle arduinoInputNodeHandle(RM_NAMESPACE);
  arduinoInputPublisher_ = arduinoInputNodeHandle.advertise<std_msgs::String>(RM_ARDUINO_INPUT_TOPIC_NAME, 1000);

  ros::NodeHandle herkulexInputNodeHandle(RM_NAMESPACE);
  herkulexInputPublisher_ = herkulexInputNodeHandle.advertise<std_msgs::String>(RM_HERKULEX_INPUT_TOPIC_NAME, 1000);

  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("reboot_button", rebootButtonIndex_, 8);

  privateNodeHandle.param("drive_axis_x", driveAxisX_, 3);
  privateNodeHandle.param("drive_axis_y", driveAxisY_, 4);
  privateNodeHandle.param("drive_boost", driveBoost_, 2);
  privateNodeHandle.param("drive_boost_min_factor", driveBoostMinFactor_, 0.25f);
  privateNodeHandle.param("drive_boost_max_factor", driveBoostMaxFactor_, 1.0f);
  driveBoostWasChanged_ = false;
  privateNodeHandle.param("drive_max_value", driveMaxValue, 100);
  privateNodeHandle.param("drive_invert_x", driveInvertX, true);
  privateNodeHandle.param("drive_invert_y", driveInvertY, false);
  driveSignX = driveInvertX ? -1.0f : 1.0f;
  driveSignY = driveInvertY ? -1.0f : 1.0f;

  privateNodeHandle.param("head_mode_button", headModeButtonIndex_, 9);
  privateNodeHandle.param("head_axis_x", headAxisX_, 0);
  privateNodeHandle.param("head_axis_y", headAxisY_, 1);
  privateNodeHandle.param("head_move_horizontal_axis", headMoveHorizontalAxis_, 6);
  privateNodeHandle.param("head_move_vertical_axis", headMoveVerticalAxis_, 7);
  privateNodeHandle.param("head_invert_horizontal", headInvertHorizontal, true);
  privateNodeHandle.param("head_invert_vertical", headInvertVertical, true);
  privateNodeHandle.param("head_move_center_button", headMoveCenterButtonIndex_, 4);

  privateNodeHandle.param("led1_button", led1ButtonIndex_, 3);
  privateNodeHandle.param("led2_button", led2ButtonIndex_, 1);
  privateNodeHandle.param("tail_button", tailButtonIndex_, 2);

  ros::NodeHandle commonNodeHandle("");
  commonNodeHandle.param("head_horizontal_min_degree", headHorizontalMinDegree, -120.0f);
  commonNodeHandle.param("head_horizontal_center_degree", headHorizontalCenterDegree, 0.0f);
  commonNodeHandle.param("head_horizontal_max_degree", headHorizontalMaxDegree, 120.0f);
  commonNodeHandle.param("head_vertical_min_degree", headVerticalMinDegree, -120.0f);
  commonNodeHandle.param("head_vertical_center_degree", headVerticalCenterDegree, -15.0f);
  commonNodeHandle.param("head_vertical_max_degree", headVerticalMaxDegree, 10.0f);

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

  rebootButton_ = new ButtonEvent(this, &JoystickNode::rebootButtonHandler);

  led1Button_ = new ButtonEvent(this, &JoystickNode::led1ButtonHandler);
  led2Button_ = new ButtonEvent(this, &JoystickNode::led2ButtonHandler);
  tailButton_ = new ButtonEvent(this, &JoystickNode::tailButtonHandler);

  headModeButton_ = new ButtonEvent(this, &JoystickNode::headModeButtonHandler);
  headMoveLeftButton_ = new ButtonEvent(this, &JoystickNode::headMoveLeftButtonHandler);
  headMoveRightButton_ = new ButtonEvent(this, &JoystickNode::headMoveRightButtonHandler);
  headMoveUpButton_ = new ButtonEvent(this, &JoystickNode::headMoveUpButtonHandler);
  headMoveDownButton_ = new ButtonEvent(this, &JoystickNode::headMoveDownButtonHandler);
  headMoveCenterButton_ = new ButtonEvent(this, &JoystickNode::headMoveCenterButtonHandler);

  headMoveMessage_.horizontal = 0;
  headMoveMessage_.vertical = 0;
}

void JoystickNode::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float driveBoost =  joy->axes[driveBoost_];
  // driveBoost values should be in interval [+1..-1].
  // +1 - the trigger is released, -1 - the trigger is fully pressed.
  // By some reason the initial value of the trigger is wrong - it is set to 0.
  // To fix the bug in joy node I have to use a flag driveBoostWasChanged_:
  if (!driveBoostWasChanged_)
  {
    if (driveBoost < -0.01 || driveBoost > 0.01)
      driveBoostWasChanged_ = true;
    else
      driveBoost = 1.0;
  }

  publishDriveMessage(joy->axes[driveAxisX_], joy->axes[driveAxisY_], driveBoost);

  if (headControlMode_ == HEAD_POSITION)
  {
    publishHeadPositionMessage(joy->axes[headAxisX_], joy->axes[headAxisY_]);
  }

  rebootButton_->update(joy->buttons[rebootButtonIndex_] == 1);

  led1Button_->update(joy->buttons[led1ButtonIndex_] == 1);
  led2Button_->update(joy->buttons[led2ButtonIndex_] == 1);
  tailButton_->update(joy->buttons[tailButtonIndex_] == 1);

  headModeButton_->update(joy->buttons[headModeButtonIndex_] == 1);
  if (headControlMode_ == HEAD_MOVE)
  {
    headMoveLeftButton_->update(joy->axes[headMoveHorizontalAxis_] > 0);
    headMoveRightButton_->update(joy->axes[headMoveHorizontalAxis_] < 0);
    headMoveUpButton_->update(joy->axes[headMoveVerticalAxis_] > 0);
    headMoveDownButton_->update(joy->axes[headMoveVerticalAxis_] < 0);
    headMoveCenterButton_->update(joy->buttons[headMoveCenterButtonIndex_] == 1);
  }
}

int8_t JoystickNode::getSpeedValue(float joystickValue)
{
  int8_t result = (int8_t) round(joystickValue * driveMaxValue);
  if (result < -driveMaxValue) return -driveMaxValue;
  else if (result > driveMaxValue) return driveMaxValue;
  return result;
}

void JoystickNode::publishDriveMessage(float x, float y, float boost)
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

  // Calculating boost_factor according to the <boost> argument:
  // <boost> value is in interval [+1..-1].
  // When <boost> is +1 - the trigger is released and boost_factor should be equal to driveBoostMinFactor_.
  // When <boost> is -1 - the trigger is fully pressed and boost_factor should be equal to driveBoostMaxFactor_.
  float boost_factor = driveBoostMinFactor_ + (driveBoostMaxFactor_ - driveBoostMinFactor_) * (boost - 1.0) / (-2.0);
  left *= boost_factor;
  right *= boost_factor;

  //ROS_INFO("x=%+5.3f y=%+5.3f R=%+5.3f A=%+8.3f    Left=%+6.3f Right=%+6.3f", x, y, radius, alpha, left, right);
  //ROS_INFO("Left=%+6.3f  Right=%+6.3f  Boost=%+6.3f  BoostFactor=%+6.3f", left, right, boost, boost_factor);

  driveMessage_.left = getSpeedValue(left);
  driveMessage_.right = getSpeedValue(right);
  drivePublisher_.publish(driveMessage_);
}

void JoystickNode::publishHeadPositionMessage(float x, float y)
{
  x *= headHorizontalAmplitude;
  x += headHorizontalCenterDegree;
  if (x < headHorizontalMinDegree) x = headHorizontalMinDegree;
  else if (x > headHorizontalMaxDegree) x = headHorizontalMaxDegree;
  headPositionMessage_.horizontal = x;

  y *= headVerticalAmplitude;
  y += headVerticalCenterDegree;
  if (y < headVerticalMinDegree) y = headVerticalMinDegree;
  else if (y > headVerticalMaxDegree) y = headVerticalMaxDegree;
  headPositionMessage_.vertical = y;

  headPositionPublisher_.publish(headPositionMessage_);
}

void JoystickNode::publishSwitchLed1Message()
{
  std_msgs::String msg;
  msg.data = RoboCom::getSwitchLed1Command();
  arduinoInputPublisher_.publish(msg);
}

void JoystickNode::publishSwitchLed2Message()
{
  std_msgs::String msg;
  msg.data = RoboCom::getSwitchLed2Command();
  arduinoInputPublisher_.publish(msg);
}

void JoystickNode::publishSwingTailMessage()
{
  std_msgs::String msg;
  msg.data = RoboCom::getSwingTailCommand();
  arduinoInputPublisher_.publish(msg);
}

void JoystickNode::publishCenterHerkulex(uint8_t address)
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "n";
  out << YAML::Value << "center";
  out << YAML::Key << "a";
  out << YAML::Value << (int) address;
  out << YAML::EndMap;

  std_msgs::String stringMessage;
  stringMessage.data = out.c_str();
  herkulexInputPublisher_.publish(stringMessage);
}

void JoystickNode::publishRebootHerkulex()
{
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "n";
  out << YAML::Value << "reboot";
  out << YAML::Key << "a";
  out << YAML::Value << (int) HEAD_BROADCAST_SERVO_ID;
  out << YAML::EndMap;

  std_msgs::String stringMessage;
  stringMessage.data = out.c_str();
  herkulexInputPublisher_.publish(stringMessage);
}

void JoystickNode::rebootButtonHandler(bool state)
{
  if (!state) return;
  publishRebootHerkulex();
}

void JoystickNode::led1ButtonHandler(bool state)
{
  if (!state) return;
  publishSwitchLed1Message();
}

void JoystickNode::led2ButtonHandler(bool state)
{
  if (!state) return;
  publishSwitchLed2Message();
}

void JoystickNode::tailButtonHandler(bool state)
{
  if (!state) return;
  publishSwingTailMessage();
}

void JoystickNode::headModeButtonHandler(bool state)
{
  if (!state) return;
  if (headControlMode_ == HEAD_MOVE)
    headControlMode_ = HEAD_POSITION;
  else if (headControlMode_ == HEAD_POSITION)
    headControlMode_ = HEAD_MOVE;
}

void JoystickNode::headMoveLeftButtonHandler(bool state)
{
  headMoveMessage_.horizontal = state ? 1 : 0;
  if (headInvertHorizontal)
    headMoveMessage_.horizontal = -headMoveMessage_.horizontal;
  headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveRightButtonHandler(bool state)
{
  headMoveMessage_.horizontal = state ? -1 : 0;
  if (headInvertHorizontal)
    headMoveMessage_.horizontal = -headMoveMessage_.horizontal;
  headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveUpButtonHandler(bool state)
{
  headMoveMessage_.vertical = state ? 1 : 0;
  if (headInvertVertical)
    headMoveMessage_.vertical = -headMoveMessage_.vertical;
  headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveDownButtonHandler(bool state)
{
  headMoveMessage_.vertical = state ? -1 : 0;
  if (headInvertVertical)
    headMoveMessage_.vertical = -headMoveMessage_.vertical;
  headMovePublisher_.publish(headMoveMessage_);
}

void JoystickNode::headMoveCenterButtonHandler(bool state)
{
  if (!state) return;
  publishCenterHerkulex(HEAD_BROADCAST_SERVO_ID);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_JOYSTICK_NODE_NAME);
  JoystickNode joystickNode;
  ros::spin();

  return 0;
}
