/*
 * arduino_node.cpp
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
#include "std_msgs/String.h"
#include "mitya_teleop/Drive.h"
#include "consts.h"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#define MAX_MESSAGE_SIZE 200
#define SERIAL_BUFFER_SIZE 1000

class ArduinoNode
{
public:
  ArduinoNode();
  void openSerial();
  void readSerial(void (*func)(ArduinoNode*, char*));
  void writeSerial(char const* message);
  void publish(char *message);
private:
  std::string serialPortName;
  int fd;
  char serialOutcomingMessage[MAX_MESSAGE_SIZE];
  char serialIncomingMessage[MAX_MESSAGE_SIZE];
  int serialIncomingMessageSize;
  char buffer[SERIAL_BUFFER_SIZE];

  ros::Subscriber arduinoInputSubscriber_;
  void arduinoInputCallback(const std_msgs::StringConstPtr& msg);
  ros::Publisher arduinoOutputPublisher_;
  ros::Subscriber driveSubscriber_;
  void driveCallback(const mitya_teleop::Drive::ConstPtr& msg);

  bool setInterfaceAttribs(int fd, int speed, int parity);
  bool setBlocking(int fd, int should_block);
};

ArduinoNode::ArduinoNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  arduinoInputSubscriber_ = nodeHandle.subscribe(RM_ARDUINO_INPUT_TOPIC_NAME, 1000, &ArduinoNode::arduinoInputCallback, this);
  arduinoOutputPublisher_ = nodeHandle.advertise<std_msgs::String>(RM_ARDUINO_OUTPUT_TOPIC_NAME, 1000);
  driveSubscriber_ = nodeHandle.subscribe(RM_DRIVE_TOPIC_NAME, 1000, &ArduinoNode::driveCallback, this);

  ros::NodeHandle privateNodeHandle("~");
  std::string serialPortParamName = "serial_port";
  if (privateNodeHandle.getParam(serialPortParamName, serialPortName))
  {
    ROS_INFO("Serial port name: %s", serialPortName.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get parameter '%s'", serialPortParamName.c_str());
  }

  fd = -1;
  serialIncomingMessageSize = 0;
  serialIncomingMessage[0] = '\0';
}

void ArduinoNode::arduinoInputCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO("Node \'%s\' topic \'%s\' received \'%s\'", RM_ARDUINO_NODE_NAME, RM_ARDUINO_INPUT_TOPIC_NAME, msg->data.c_str());
  writeSerial(msg->data.c_str());
}

void ArduinoNode::driveCallback(const mitya_teleop::Drive::ConstPtr& msg)
{
  ROS_INFO("Node \'%s\' topic \'%s\' received \'%d,%d\'", RM_ARDUINO_NODE_NAME, RM_DRIVE_TOPIC_NAME, msg->left, msg->right);
  sprintf(serialOutcomingMessage, "ML %d;", msg->left);
  writeSerial(serialOutcomingMessage);
  sprintf(serialOutcomingMessage, "MR %d;", msg->right);
  writeSerial(serialOutcomingMessage);
}

bool ArduinoNode::setInterfaceAttribs(int fd, int speed, int parity)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
  {
    ROS_ERROR("Error %d from tcgetattr", errno);
    return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    ROS_ERROR("Error %d from tcsetattr", errno);
    return false;
  }
  return true;
}

bool ArduinoNode::setBlocking(int fd, int should_block)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
  {
    ROS_ERROR("Error %d from tcgetattr", errno);
    return false;
  }

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    ROS_ERROR("Error %d setting term attributes", errno);
    return false;
  }

  return true;
}

void ArduinoNode::openSerial()
{
  fd = open(serialPortName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    ROS_ERROR("Error %d opening %s: %s", errno, serialPortName.c_str(), strerror(errno));
    return;
  }
  if (!setInterfaceAttribs(fd, B9600, 0)) // set speed to 9,600 bps, 8n1 (no parity)
    return;
  if (!setBlocking(fd, 0))                // set no blocking
    return;
  ROS_INFO("Serial port is opened");
}

void ArduinoNode::readSerial(void (*func)(ArduinoNode*, char*))
{
  int n = read(fd, buffer, SERIAL_BUFFER_SIZE);
  char ch;
  for (int i = 0; i < n; i++)
  {
    ch = buffer[i];
    if (ch < 32) continue;
    serialIncomingMessage[serialIncomingMessageSize++] = ch;
    if (ch == ';')
    {
      serialIncomingMessage[serialIncomingMessageSize] = '\0';
      func(this, serialIncomingMessage);
      serialIncomingMessageSize = 0;
      serialIncomingMessage[serialIncomingMessageSize] = '\0';
    }
  }
}

void ArduinoNode::writeSerial(char const* message)
{
  write(fd, message, strlen(message));
}

void ArduinoNode::publish(char *message)
{
  std_msgs::String stringMessage;
  stringMessage.data = message;
  arduinoOutputPublisher_.publish(stringMessage);
}

void onReceiveSerialMessage(ArduinoNode *arduinoNode, char *message)
{
  ROS_INFO("Message from the controller: %s", message);
  arduinoNode->publish(message);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_ARDUINO_NODE_NAME);
  ArduinoNode arduinoNode;
  arduinoNode.openSerial();

  ros::Rate loop_rate(100); // 100 Hz
  while (ros::ok())
  {
    arduinoNode.readSerial(onReceiveSerialMessage);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
