/*
 * herkulex_node.cpp
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
 *  Created on: Aug 2, 2017
 *      Author: Dmitry Dzakhov
 */

#include <ros/ros.h>
#include <signal.h>
#include "consts.h"
#include "mitya_teleop/HeadPosition.h"
#include "mitya_teleop/HeadMove.h"
#include <diagnostic_msgs/KeyValue.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "herkulex.h"
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include "madgwick.h"

//#define SERVO_H 1
//#define SERVO_V 2
//#define SERVO_ALL 0xFE
#define CORRECTION_DURATION 0

#define MAX(x, y) (((x) > (y)) ? (x) : (y))

class HerkulexNode
{
public:
  std::string serialPortName;
  int serialBaudRate;

  HerkulexNode();
  void update();
  void stopHead();
  void setTorqueMode(HerkulexTorqueState mode);
  void logPosition();
private:
  HerkulexClass herkulex_;
  void initServos();

  float headHorizontalMinDegree;
  float headHorizontalCenterDegree;
  float headHorizontalMaxDegree;
  float headVerticalMinDegree;
  float headVerticalCenterDegree;
  float headVerticalMaxDegree;
  float headMoveMinSpeed_; // degrees per second

  float headMoveSpeed_; // degrees per second

  bool targetMode_;
  tf2::Quaternion imuQuaternion_;
  tf2::Quaternion targetQuaternion_;
  void updateToTarget();

  // Topic RM_HERKULEX_INPUT_TOPIC_NAME ('herkulex_input') subscriber:
  ros::Subscriber herkulexInputSubscriber_;
  void herkulexInputCallback(const std_msgs::StringConstPtr& msg);

  // Topic RM_HERKULEX_OUTPUT_TOPIC_NAME ('herkulex_output') publisher:
  ros::Publisher herkulexOutputPublisher_;

  //TODO Убрать!
  ros::Publisher herkulexLogPositionPublisher_;
  //TODO Убрать!
  ros::Publisher imuLogPositionPublisher_;

  // Topic RM_HEAD_POSITION_TOPIC_NAME ('head_position') subscriber:
  ros::Subscriber headPositionSubscriber_;
  void headPositionCallback(const mitya_teleop::HeadPosition::ConstPtr& msg);

  // Topic RM_HEAD_MOVE_TOPIC_NAME ('head_move') subscriber:
  ros::Subscriber headMoveSubscriber_;
  void headMoveCallback(const mitya_teleop::HeadMove::ConstPtr& msg);

  // Topic RM_HEAD_IMU_INPUT_TOPIC_NAME ('head_imu_input') publisher:
  ros::Publisher imuInputPublisher_;
  // Topic RM_HEAD_IMU_OUTPUT_TOPIC_NAME ('herkulex_output') subscriber:
  ros::Subscriber imuOutputSubscriber_;
  void imuOutputCallback(const sensor_msgs::Imu::ConstPtr& msg);

  struct HeadMoveValues
  {
    int horizontal;
    int vertical;
  };
  HeadMoveValues previousHeadMoveValues_;
  int calculateDurationInMillis(float deltaAngle, float degreesPerSecond);

  /*
   * Returns movement duration in millis.
   */
  int headMoveCenter(int servoAddress);

  void setHeadPositionHorizontal(float angle, int duration);
  void setHeadPositionVertical(float angle, int duration);

  void headServoReboot(int servoAddress);

  void centerHeadImu(double millis);
  ros::Time centerHeadImuStartTime_;
  bool centerHeadImuStarted_;

  float factor1_;
  float factor2_;
  static const tf2Scalar POINTING_DEFAULT = 1000.0f;
};

HerkulexNode::HerkulexNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  herkulexInputSubscriber_ = nodeHandle.subscribe(RM_HERKULEX_INPUT_TOPIC_NAME, 1000, &HerkulexNode::herkulexInputCallback, this);
  herkulexOutputPublisher_ = nodeHandle.advertise<std_msgs::String>(RM_HERKULEX_OUTPUT_TOPIC_NAME, 1000);
  herkulexLogPositionPublisher_ = nodeHandle.advertise<mitya_teleop::HeadPosition>("herkulex_log", 1000);
  imuLogPositionPublisher_ = nodeHandle.advertise<mitya_teleop::HeadPosition>("imu_log", 1000);
  headPositionSubscriber_ = nodeHandle.subscribe(RM_HEAD_POSITION_TOPIC_NAME, 1000, &HerkulexNode::headPositionCallback, this);
  headMoveSubscriber_ = nodeHandle.subscribe(RM_HEAD_MOVE_TOPIC_NAME, 1000, &HerkulexNode::headMoveCallback, this);
  imuInputPublisher_ = nodeHandle.advertise<std_msgs::String>(RM_HEAD_IMU_INPUT_TOPIC_NAME, 10);
  imuOutputSubscriber_ = nodeHandle.subscribe(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, &HerkulexNode::imuOutputCallback, this);

  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("serial_port", serialPortName, (std::string) "/dev/ttyUSB0");
  privateNodeHandle.param("baud_rate", serialBaudRate, 115200);

  initServos();

  ros::NodeHandle commonNodeHandle("");
  commonNodeHandle.param("head_horizontal_min_degree", headHorizontalMinDegree, -120.0f);
  commonNodeHandle.param("head_horizontal_center_degree", headHorizontalCenterDegree, 0.0f);
  commonNodeHandle.param("head_horizontal_max_degree", headHorizontalMaxDegree, 120.0f);
  commonNodeHandle.param("head_vertical_min_degree", headVerticalMinDegree, -120.0f);
  commonNodeHandle.param("head_vertical_center_degree", headVerticalCenterDegree, -15.0f);
  commonNodeHandle.param("head_vertical_max_degree", headVerticalMaxDegree, 10.0f);

  // 2856 mSec is the longest duration for single movement of the Herkulex servo.
  // To calculate to lowest speed we divide the longest path in degrees by this time.
  // See herkulex.cpp (search "2856") for more information.
  headMoveMinSpeed_ = floor(
      MAX(abs(headHorizontalMaxDegree - headHorizontalMinDegree), abs(headVerticalMaxDegree - headVerticalMinDegree)) *
      1000.0f / 2856.0f + 1);
  headMoveSpeed_ = headMoveMinSpeed_;

  previousHeadMoveValues_.horizontal = 0;
  previousHeadMoveValues_.vertical = 0;

  centerHeadImuStarted_ = false;

  targetQuaternion_ = tf2::Quaternion::getIdentity();
  targetMode_ = false;

  factor1_ = 0.0f;
  factor2_ = 0.0f;
}

void HerkulexNode::initServos()
{
  bool portOpened = herkulex_.begin(serialPortName.c_str(), serialBaudRate);
  if (portOpened)
    ROS_INFO("Serial port \'%s\' is opened", serialPortName.c_str());
  else
    ROS_ERROR("Error %d opening %s: %s", errno, serialPortName.c_str(), strerror(errno));
  herkulex_.initialize();
}

void HerkulexNode::herkulexInputCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_DEBUG("Received in %s.%s: %s", RM_HERKULEX_NODE_NAME, RM_HERKULEX_INPUT_TOPIC_NAME, msg->data.c_str());

  YAML::Node node = YAML::Load(msg->data);

  std::string commandName = node["n"] ? node["n"].as<std::string>() : "null name";

  if (commandName.compare("mode") == 0)
  {
    if (!node["m"])
    {
      ROS_ERROR("HerkuleX command (%s) processor error: mode is not defined", commandName.c_str());
      return;
    }
    int mode = node["m"].as<int>();
    ROS_INFO("HerkuleX command (%s): mode = %d", commandName.c_str(), mode);
    herkulex_.torqueState(HEAD_HORIZONTAL_SERVO_ID, (TorqueState) mode);
    herkulex_.torqueState(HEAD_VERTICAL_SERVO_ID, (TorqueState) mode);
  }
  else if (commandName.compare("pointing") == 0)
  {
    if (!node["v"])
    {
      ROS_ERROR("HerkuleX command (%s) processor error: value is not defined", commandName.c_str());
      return;
    }
    int value = node["v"].as<int>();
    ROS_INFO("HerkuleX command (%s): value = %d", commandName.c_str(), value);
    stopHead();
    targetMode_ = value != 0;
  }
  else if (commandName.compare("f1") == 0)
  {
    if (!node["v"])
    {
      ROS_ERROR("HerkuleX command (%s) processor error: value is not defined", commandName.c_str());
      return;
    }
    float value = node["v"].as<float>();
    ROS_INFO("HerkuleX command (%s): value = %f", commandName.c_str(), value);
    factor1_ = value;
  }
  else if (commandName.compare("f2") == 0)
  {
    if (!node["v"])
    {
      ROS_ERROR("HerkuleX command (%s) processor error: value is not defined", commandName.c_str());
      return;
    }
    float value = node["v"].as<float>();
    ROS_INFO("HerkuleX command (%s): value = %f", commandName.c_str(), value);
    factor2_ = value;
  }
  else
  {
    if (!node["a"])
    {
      ROS_ERROR("HerkuleX command (%s) processor error: servo ID is not defined", commandName.c_str());
      return;
    }
    int address = node["a"].as<int>();
    if (address != HEAD_HORIZONTAL_SERVO_ID && address != HEAD_VERTICAL_SERVO_ID && address != HEAD_BROADCAST_SERVO_ID)
    {
      ROS_ERROR("HerkuleX command (%s) processor error: unknown servo ID (%d)", commandName.c_str(), address);
      return;
    }

    if (commandName.compare("stat") == 0)
    {
      uint8_t statusError;
      uint8_t statusDetail;
      signed char result = herkulex_.stat(address, &statusError, &statusDetail);
      if (result == 0)
      {
        ROS_DEBUG("Sending HerkuleX response to %s: statusError=%d, statusDetail=%d", RM_HERKULEX_INPUT_TOPIC_NAME, statusError, statusDetail);

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "n";
        out << YAML::Value << "stat";
        out << YAML::Key << "a";
        out << YAML::Value << (int) address;
        out << YAML::Key << "e";
        out << YAML::Value << (int) statusError;
        out << YAML::Key << "d";
        out << YAML::Value << (int) statusDetail;
        out << YAML::EndMap;

        std_msgs::String stringMessage;
        stringMessage.data = out.c_str();
        herkulexOutputPublisher_.publish(stringMessage);
      }
      else
      {
        ROS_ERROR("HerkuleX command (%s) processor error: wrong checksum in response (error code %d)", commandName.c_str(), result);
      }
    }
    else if (commandName.compare("led") == 0)
    {
      if (!node["c"])
      {
        ROS_ERROR("HerkuleX command (%s) processor error: LED color is not defined", commandName.c_str());
        return;
      }
      int color = node["c"].as<int>();
      if (color < 0 || color > 7)
      {
        ROS_ERROR("HerkuleX command (%s) processor error: bad color value (%d)", commandName.c_str(), color);
        return;
      }
      herkulex_.setLed(address, color);
    }
    else if (commandName.compare("center") == 0)
    {
      int delay = 0;
      if (!targetMode_)
      {
        if (address == HEAD_BROADCAST_SERVO_ID)
        {
          int durationH = headMoveCenter(HEAD_HORIZONTAL_SERVO_ID);
          int durationV = headMoveCenter(HEAD_VERTICAL_SERVO_ID);
          delay = MAX(durationH, durationV);
        }
        else
          delay = headMoveCenter(address);
      }
      centerHeadImu(delay);
    }
    else if (commandName.compare("reboot") == 0)
    {
      headServoReboot(address);
    }
    else
    {
      ROS_ERROR("Unknown command name: \'%s\'", commandName.c_str());
      return;
    }
  }
}

void HerkulexNode::headPositionCallback(const mitya_teleop::HeadPosition::ConstPtr& msg)
{
  if (targetMode_) return;
  //ROS_INFO("Received in %s.%s: %f, %f", RM_HERKULEX_NODE_NAME, RM_HEAD_POSITION_TOPIC_NAME, msg->horizontal, msg->vertical);
  setHeadPositionHorizontal(msg->horizontal, 0);
  setHeadPositionVertical(msg->vertical, 0);
}

void HerkulexNode::setHeadPositionHorizontal(float angle, int duration)
{
  if (angle < headHorizontalMinDegree)
    angle = headHorizontalMinDegree;
  else if (angle > headHorizontalMaxDegree)
    angle = headHorizontalMaxDegree;
  herkulex_.moveOneAngle(HEAD_HORIZONTAL_SERVO_ID, angle, duration, 0);
}

void HerkulexNode::setHeadPositionVertical(float angle, int duration)
{
  if (angle < headVerticalMinDegree)
    angle = headVerticalMinDegree;
  else if (angle > headVerticalMaxDegree)
    angle = headVerticalMaxDegree;
  herkulex_.moveOneAngle(HEAD_VERTICAL_SERVO_ID, angle, 0, 0);
}

void HerkulexNode::headMoveCallback(const mitya_teleop::HeadMove::ConstPtr& msg)
{
  //TODO Вернуть!
  //if (targetMode_) return;
  //ROS_INFO("Received in %s.%s: %d, %d", RM_HERKULEX_NODE_NAME, RM_HEAD_MOVE_TOPIC_NAME, msg->horizontal, msg->vertical);

  if (msg->horizontal != previousHeadMoveValues_.horizontal)
  {
    if (msg->horizontal != 0)
    {
      float currentAngle = herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID);
      float targetAngle = msg->horizontal > 0 ? headHorizontalMaxDegree : headHorizontalMinDegree;
      int duration = calculateDurationInMillis(targetAngle - currentAngle, headMoveSpeed_);
//ROS_DEBUG("H: currentAngle=%.3f, targetAngle=%.3f, duration=%d", currentAngle, targetAngle, duration);
      herkulex_.moveOneAngle(HEAD_HORIZONTAL_SERVO_ID, targetAngle, duration, 0);
    }
    else
    {
      herkulex_.moveOneAngle(HEAD_HORIZONTAL_SERVO_ID, herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID), CORRECTION_DURATION, 0);
    }
    previousHeadMoveValues_.horizontal = msg->horizontal;
  }

  if (msg->vertical != previousHeadMoveValues_.vertical)
  {
    if (msg->vertical != 0)
    {
      float currentAngle = herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID);
      float targetAngle = msg->vertical > 0 ? headVerticalMaxDegree : headVerticalMinDegree;
      int duration = calculateDurationInMillis(targetAngle - currentAngle, headMoveSpeed_);
//ROS_DEBUG("V: currentAngle=%.3f, targetAngle=%.3f, duration=%d", currentAngle, targetAngle, duration);
      herkulex_.moveOneAngle(HEAD_VERTICAL_SERVO_ID, targetAngle, duration, 0);
    }
    else
    {
      herkulex_.moveOneAngle(HEAD_VERTICAL_SERVO_ID, herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID), CORRECTION_DURATION, 0);
    }
    previousHeadMoveValues_.vertical = msg->vertical;
  }
}

void HerkulexNode::imuOutputCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imuQuaternion_.setValue(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void HerkulexNode::logPosition()
{
  mitya_teleop::HeadPosition headPosition;
  headPosition.horizontal = herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID);
  headPosition.vertical = herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID);
  herkulexLogPositionPublisher_.publish(headPosition);
}

int HerkulexNode::calculateDurationInMillis(float deltaAngle, float degreesPerSecond)
{
/*  deltaAngle = fabs(deltaAngle);
  if (factor1_ > 0 && factor2_ > 0)
  {
    if (deltaAngle < factor1_)
    {
      // f(t) = 1 - (1 - K*x)^2, x=[0, 1/K];
      // f(t) = 1, x>1/K.
      float t = deltaAngle / factor1_;
      // K = pointingFactor_
      if (t < 1.0f / factor2_)
      {
        float f = 1.0f - factor2_ * t;
        degreesPerSecond *= 1.0f - f * f;
      }
    }
  }
*/
  if (degreesPerSecond < headMoveMinSpeed_)
    degreesPerSecond = headMoveMinSpeed_;
  float result = (int)(deltaAngle * 1000.0f / degreesPerSecond);
  return result > 0 ? result : -result;
}

int HerkulexNode::headMoveCenter(int servoAddress)
{
  float currentAngle = herkulex_.getAngle(servoAddress);
  float targetAngle = servoAddress == HEAD_HORIZONTAL_SERVO_ID ? headHorizontalCenterDegree : headVerticalCenterDegree;
  int duration = calculateDurationInMillis(targetAngle - currentAngle, headMoveSpeed_);
  herkulex_.moveOneAngle(servoAddress, targetAngle, duration, 0);
  return duration;
}

void HerkulexNode::headServoReboot(int servoAddress)
{
  herkulex_.reboot(servoAddress);

  usleep(1000000);
  initServos();
}

void HerkulexNode::update()
{
  ros::Time now = ros::Time::now();
  if (centerHeadImuStarted_ && now >= centerHeadImuStartTime_)
  {
    //ROS_DEBUG("Centering Head Imu [nowSeconds = %f]", now.toSec());
    centerHeadImuStarted_ = false;

    std_msgs::String stringMessage;
    stringMessage.data = "center";
    imuInputPublisher_.publish(stringMessage);
  }

  if (targetMode_)
  {
    updateToTarget();
  }
}

void HerkulexNode::centerHeadImu(double millis)
{
  ros::Time now = ros::Time::now();
  //ROS_DEBUG("Function centerHeadImu(%f) is called [nowSeconds = %f]", millis, now.toSec());
  centerHeadImuStartTime_ = now + ros::Duration(millis / 1000.0);
  centerHeadImuStarted_ = true;
}

void HerkulexNode::updateToTarget()
{
  tf2Scalar imuYaw;
  tf2Scalar imuPitch;
  MadgwickImu::getEulerYP(imuQuaternion_, imuYaw, imuPitch);
  tf2Scalar targetYaw;
  tf2Scalar targetPitch;
  MadgwickImu::getEulerYP(targetQuaternion_, targetYaw, targetPitch);

  //TODO Убрать!
  mitya_teleop::HeadPosition headPosition;
  headPosition.horizontal = imuYaw;
  headPosition.vertical = imuPitch;
  imuLogPositionPublisher_.publish(headPosition);

  tf2Scalar deltaYaw = targetYaw - imuYaw;
  tf2Scalar deltaPitch = targetPitch - imuPitch;
  float speed = headMoveSpeed_ * 4.0f;
  int yawDuration = calculateDurationInMillis(deltaYaw, speed);
  int pitchDuration = calculateDurationInMillis(deltaPitch, speed);
  int duration = MAX(yawDuration, pitchDuration);
  float aYaw = herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID);
  float aPitch = herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID);
  float yaw = aYaw + deltaYaw;
  float pitch = aPitch - deltaPitch; // (should be plus, but pitch servo's axis is directed in negative direction)
  ROS_INFO("iY/iP: %+9.3f    %+9.3f    tY/tP: %+9.3f    %+9.3f    aY/aP: %+9.3f    %+9.3f    Y/P: %+9.3f    %+9.3f",
           imuYaw, imuPitch, targetYaw, targetPitch, aYaw, aPitch, yaw, pitch);
//  ROS_INFO("iY(aY): %+9.3f (%+9.3f)    iP(aP): %+9.3f (%+9.3f)", imuYaw, aYaw, imuPitch, aPitch);
  setHeadPositionHorizontal(yaw, duration);
  setHeadPositionVertical(pitch, duration);
}

void HerkulexNode::stopHead()
{
  setHeadPositionHorizontal(herkulex_.getAngle(HEAD_HORIZONTAL_SERVO_ID), 0);
  setHeadPositionVertical(herkulex_.getAngle(HEAD_VERTICAL_SERVO_ID), 0);
}

void HerkulexNode::setTorqueMode(HerkulexTorqueState mode)
{
  switch (mode)
  {
    case HTS_BREAK_ON:
    case HTS_TORQUE_ON:
      herkulex_.torqueState(HEAD_HORIZONTAL_SERVO_ID, (TorqueState) mode);
      herkulex_.torqueState(HEAD_VERTICAL_SERVO_ID, (TorqueState) mode);
      break;
    default:
      herkulex_.torqueState(HEAD_HORIZONTAL_SERVO_ID, TS_TORQUE_FREE);
      herkulex_.torqueState(HEAD_VERTICAL_SERVO_ID, TS_TORQUE_FREE);
      break;
  }
}

HerkulexNode *herkulexNode = NULL;

// Calls on shutting down the node.
void sigintHandler(int sig)
{
  ROS_INFO("Shutting down %s", RM_HERKULEX_NODE_NAME);
  if (herkulexNode != NULL)
  {
    herkulexNode->stopHead();
    herkulexNode->setTorqueMode(HTS_TORQUE_FREE);
  }

  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_HERKULEX_NODE_NAME, ros::init_options::NoSigintHandler);

  herkulexNode = new HerkulexNode();

  signal(SIGINT, sigintHandler);

  //TODO !
  //ros::Rate loop_rate(100); // (Hz)
  ros::Rate loop_rate(0.100); // (Hz)
  while (ros::ok())
  {
    herkulexNode->update();
    herkulexNode->logPosition();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
