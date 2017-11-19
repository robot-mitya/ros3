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

#include "ros/ros.h"
#include "consts.h"
#include "mitya_teleop/HeadPosition.h"
#include "diagnostic_msgs/KeyValue.h"
#include "yaml-cpp/yaml.h"
#include "std_msgs/String.h"
#include "herkulex.h"
#include <unistd.h>

class HerkulexNode
{
public:
  std::string serialPortName;
  int serialBaudRate;

  HerkulexNode();

  void logPosition();
private:
  HerkulexClass herkulex;

  float headHorizontalMinDegree;
  float headHorizontalCenterDegree;
  float headHorizontalMaxDegree;
  float headVerticalMinDegree;
  float headVerticalCenterDegree;
  float headVerticalMaxDegree;

  // Topic RM_HERKULEX_INPUT_TOPIC_NAME ('herkulex_input') subscriber:
  ros::Subscriber herkulexInputSubscriber_;
  void herkulexInputCallback(const std_msgs::StringConstPtr& msg);

  // Topic RM_HERKULEX_OUTPUT_TOPIC_NAME ('herkulex_output') publisher:
  ros::Publisher herkulexOutputPublisher_;

  // Topic RM_HEAD_POSITION_TOPIC_NAME ('head_position') subscriber:
  ros::Subscriber headPositionSubscriber_;
  void headPositionCallback(const mitya_teleop::HeadPosition::ConstPtr& msg);
};

HerkulexNode::HerkulexNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  herkulexInputSubscriber_ = nodeHandle.subscribe(RM_HERKULEX_INPUT_TOPIC_NAME, 1000, &HerkulexNode::herkulexInputCallback, this);
  herkulexOutputPublisher_ = nodeHandle.advertise<std_msgs::String>(RM_HERKULEX_OUTPUT_TOPIC_NAME, 1000);
  headPositionSubscriber_ = nodeHandle.subscribe(RM_HEAD_POSITION_TOPIC_NAME, 1000, &HerkulexNode::headPositionCallback, this);

  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("serial_port", serialPortName, (std::string) "/dev/ttyUSB0");
  privateNodeHandle.param("baud_rate", serialBaudRate, 115200);

  bool portOpened = herkulex.begin(serialPortName.c_str(), serialBaudRate);
  if (portOpened)
    ROS_INFO("Serial port \'%s\' is opened", serialPortName.c_str());
  else
    ROS_ERROR("Error %d opening %s: %s", errno, serialPortName.c_str(), strerror(errno));
  herkulex.initialize();

  ros::NodeHandle commonNodeHandle("");
  commonNodeHandle.param("head_horizontal_min_degree", headHorizontalMinDegree, -120.0f);
  commonNodeHandle.param("head_horizontal_center_degree", headHorizontalCenterDegree, 0.0f);
  commonNodeHandle.param("head_horizontal_max_degree", headHorizontalMaxDegree, 120.0f);
  commonNodeHandle.param("head_vertical_min_degree", headVerticalMinDegree, -120.0f);
  commonNodeHandle.param("head_vertical_center_degree", headVerticalCenterDegree, -15.0f);
  commonNodeHandle.param("head_vertical_max_degree", headVerticalMaxDegree, 10.0f);
}

void HerkulexNode::herkulexInputCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_DEBUG("Received in %s.%s: %s", RM_HERKULEX_NODE_NAME, RM_HERKULEX_INPUT_TOPIC_NAME, msg->data.c_str());

  YAML::Node node = YAML::Load(msg->data);

  std::string commandName = node["n"] ? node["n"].as<std::string>() : "null name";

  if (!node["a"])
  {
    ROS_ERROR("HerkuleX command (%s) processor error: servo ID is not defined", commandName.c_str());
    return;
  }
  int address = node["a"].as<int>();
  if (address != 1 && address != 2 && address != 0xFE)
  {
    ROS_ERROR("HerkuleX command (%s) processor error: unknown servo ID (%d)", commandName.c_str(), address);
    return;
  }

  if (commandName.compare("stat") == 0)
  {
    uint8_t statusError;
    uint8_t statusDetail;
    signed char result = herkulex.stat(address, &statusError, &statusDetail);
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
    herkulex.setLed(address, color);
  }
  else
  {
    ROS_ERROR("Unknown command name: \'%s\'", commandName.c_str());
    return;
  }
}

void HerkulexNode::headPositionCallback(const mitya_teleop::HeadPosition::ConstPtr& msg)
{
  ROS_INFO("Received in %s.%s: %f, %f", RM_HERKULEX_NODE_NAME, RM_HEAD_POSITION_TOPIC_NAME, msg->horizontal, msg->vertical);

  float angleH = msg->horizontal;
  if (angleH < headHorizontalMinDegree)
    angleH = headHorizontalMinDegree;
  else if (angleH > headHorizontalMaxDegree)
    angleH = headHorizontalMaxDegree;

  float angleV = msg->vertical;
  if (angleV < headVerticalMinDegree)
    angleV = headVerticalMinDegree;
  else if (angleV > headVerticalMaxDegree)
    angleV = headVerticalMaxDegree;

  herkulex.moveOneAngle(1, angleH, 0, 0);
  herkulex.moveOneAngle(2, angleV, 0, 0);
}

void HerkulexNode::logPosition()
{
  ROS_INFO("++++++ Angle=%.3f", herkulex.getAngle(1));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_HERKULEX_NODE_NAME);

  HerkulexNode herkulexNode;

  ros::Rate loop_rate(100); // 100 Hz
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();

//    herkulexNode.logPosition();
  }

  return 0;
}