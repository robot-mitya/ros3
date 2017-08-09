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
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "mitya_teleop/Drive.h"
#include "consts.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include "robo_com.h"

#define MAX_MESSAGE_SIZE 200
#define SERIAL_BUFFER_SIZE 1000

class ArduinoNode
{
public:
  ArduinoNode();
  void openSerial();
  void closeSerial();
  void readSerial(void (*func)(ArduinoNode*, char*));
  void writeSerial(char const* message);
  void publishArduinoOutput(char *message);
  void publishLED(int ledState);
  void publishDistance(float distance);
  void publishSpeed(float speed);
private:
  std::string serialPortName;
  int serialBaudRate;
  int fd;
  bool isPortOpened;
  char serialIncomingMessage[MAX_MESSAGE_SIZE];
  int serialIncomingMessageSize;
  char buffer[SERIAL_BUFFER_SIZE];
  int baudRateToBaudRateConst(int baudRate);

  // Topic RM_ARDUINO_INPUT_TOPIC_NAME ('arduino_input') subscriber:
  ros::Subscriber arduinoInputSubscriber_;
  void arduinoInputCallback(const std_msgs::StringConstPtr& msg);

  // Topic RM_ARDUINO_OUTPUT_TOPIC_NAME ('arduino_output') publisher:
  ros::Publisher arduinoOutputPublisher_;

  // Topic RM_DRIVE_TOPIC_NAME ('drive') subscriber:
  ros::Subscriber driveSubscriber_;
  void driveCallback(const mitya_teleop::Drive::ConstPtr& msg);

  // Topic RM_LED_TOPIC_NAME ('led') publisher:
  ros::Publisher ledPublisher_;

  // Topic RM_DISTANCE_TOPIC_NAME ('distance') publisher:
  ros::Publisher distancePublisher_;

  // Topic RM_SPEED_TOPIC_NAME ('speed') publisher:
  ros::Publisher speedPublisher_;

  bool setInterfaceAttribs(int fd, int speed, int parity);
  bool setBlocking(int fd, int should_block);
};

ArduinoNode::ArduinoNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  arduinoInputSubscriber_ = nodeHandle.subscribe(RM_ARDUINO_INPUT_TOPIC_NAME, 1000, &ArduinoNode::arduinoInputCallback, this);
  arduinoOutputPublisher_ = nodeHandle.advertise<std_msgs::String>(RM_ARDUINO_OUTPUT_TOPIC_NAME, 1000);
  driveSubscriber_ = nodeHandle.subscribe(RM_DRIVE_TOPIC_NAME, 1000, &ArduinoNode::driveCallback, this);
  ledPublisher_ = nodeHandle.advertise<std_msgs::Int8>(RM_LED_TOPIC_NAME, 1000);
  distancePublisher_ = nodeHandle.advertise<std_msgs::Float32>(RM_DISTANCE_TOPIC_NAME, 1000);
  speedPublisher_ = nodeHandle.advertise<std_msgs::Float32>(RM_SPEED_TOPIC_NAME, 1000);

  ros::NodeHandle privateNodeHandle("~");
  std::string serialPortParamName = "serial_port";
  if (privateNodeHandle.getParam(serialPortParamName, serialPortName))
  {
    ROS_INFO("Serial port name: %s", serialPortName.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get parameter '%s'", serialPortParamName.c_str());
    serialPortName = "unassigned";
  }
  std::string baudRateParamName = "baud_rate";
  if (privateNodeHandle.getParam(baudRateParamName, serialBaudRate))
  {
    ROS_INFO("Serial port baud rate: %d", serialBaudRate);
  }
  else
  {
    serialBaudRate = 9600;
    ROS_ERROR("Failed to get parameter '%s'. The default value is %d.", baudRateParamName.c_str(), serialBaudRate);
  }

  fd = -1;
  isPortOpened = false;
  serialIncomingMessageSize = 0;
  serialIncomingMessage[0] = '\0';
}

void ArduinoNode::arduinoInputCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_DEBUG("Node \'%s\' topic \'%s\' received \'%s\'", RM_ARDUINO_NODE_NAME, RM_ARDUINO_INPUT_TOPIC_NAME, msg->data.c_str());
  writeSerial(msg->data.c_str());
}

void ArduinoNode::driveCallback(const mitya_teleop::Drive::ConstPtr& msg)
{
  ROS_DEBUG("Node \'%s\' topic \'%s\' received \'%d,%d\'", RM_ARDUINO_NODE_NAME, RM_DRIVE_TOPIC_NAME, msg->left, msg->right);
  writeSerial(RoboCom::getDriveLeftCommand((signed char) (msg->left)));
  writeSerial(RoboCom::getDriveRightCommand((signed char) (msg->right)));
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
  closeSerial();
  fd = open(serialPortName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
    ROS_ERROR("Error %d opening %s: %s", errno, serialPortName.c_str(), strerror(errno));
    return;
  }
  isPortOpened = true;

  if (!setInterfaceAttribs(fd, baudRateToBaudRateConst(serialBaudRate), 0)) // set speed bps, 8n1 (no parity)
    return;
  if (!setBlocking(fd, 0)) // set no blocking
    return;

  ROS_INFO("Serial port is opened");
}

void ArduinoNode::closeSerial()
{
  if (isPortOpened)
  {
    close(fd);
    isPortOpened = false;
  }
}

int ArduinoNode::baudRateToBaudRateConst(int baudRate)
{
  switch (baudRate)
  {
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      ROS_ERROR("Wrong serial baud rate value. The default value is 9600.");
      return B9600;
  }
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
    if (ch == COMMAND_SEPARATOR)
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

void ArduinoNode::publishArduinoOutput(char *message)
{
  std_msgs::String stringMessage;
  stringMessage.data = message;
  arduinoOutputPublisher_.publish(stringMessage);
}

void ArduinoNode::publishLED(int ledState)
{
  std_msgs::Int8 intMessage;
  intMessage.data = ledState;
  ledPublisher_.publish(intMessage);
}

void ArduinoNode::publishDistance(float distance)
{
  std_msgs::Float32 floatMessage;
  floatMessage.data = distance;
  distancePublisher_.publish(floatMessage);
}

void ArduinoNode::publishSpeed(float speed)
{
  std_msgs::Float32 floatMessage;
  floatMessage.data = speed;
  speedPublisher_.publish(floatMessage);
}

void onReceiveSerialMessage(ArduinoNode *arduinoNode, char *message)
{
  ROS_DEBUG("Message from the controller: %s", message);
  arduinoNode->publishArduinoOutput(message);
  Command command;
  int param1, param2, param3;
  RoboCom::parseMessage(message, command, param1, param2, param3);
  switch (command)
  {
    case CMD_STATUS_RESPONSE:
    {
      if (param1 > 0)
        ROS_ERROR("Arduino controller's status response error: %s (%d)", RoboCom::getStatusText(param1), param1);
      break;
    }
    case CMD_LED_RESPONSE:
    {
      arduinoNode->publishLED(param1);
      break;
    }
    case CMD_DIST_RESPONSE:
    {
      float meters = param2;
      meters /= 1000;
      meters += param1;
      arduinoNode->publishDistance(meters);
      break;
    }
    case CMD_SPD_RESPONSE:
    {
      float kilometersPerHour = param1;
      kilometersPerHour /= 1000;
      arduinoNode->publishSpeed(kilometersPerHour);
      break;
    }
  }
}
/*
void checkResult(int testId,
                 Command expectedCommand, int expectedParam1, int expectedParam2, int expectedParam3,
                 Command actualCommand, int actualParam1, int actualParam2, int actualParam3)
{
  if (expectedCommand != actualCommand)
    ROS_ERROR("%d: Bad command", testId);
  if (expectedParam1 != actualParam1)
    ROS_ERROR("%d: Expected param1 %d is not equal to actual %d", testId, expectedParam1, actualParam1);
  if (expectedParam2 != actualParam2)
    ROS_ERROR("%d: Expected param2 %d is not equal to actual %d", testId, expectedParam2, actualParam2);
  if (expectedParam3 != actualParam3)
    ROS_ERROR("%d: Expected param3 %d is not equal to actual %d", testId, expectedParam3, actualParam3);

  if (expectedCommand == actualCommand && expectedParam1 == actualParam1
    && expectedParam2 == actualParam2 && expectedParam3 == actualParam3)
    ROS_INFO("%d: Test OK", testId);
}

void test()
{
  Command command;
  int param1, param2, param3;
  RoboCom::parseMessage("foo -2 2 128;", command, param1, param2, param3);
  checkResult(1, CMD_UNKNOWN, -2, 2, 128, command, param1, param2, param3);
  RoboCom::parseMessage("!DIST -1974 1974 32767;", command, param1, param2, param3);
  checkResult(2, CMD_DIST_RESPONSE, -1974, 1974, 32767, command, param1, param2, param3);
  RoboCom::parseMessage("!SPD -1974 1974 32767;", command, param1, param2, param3);
  checkResult(3, CMD_SPD_RESPONSE, -1974, 1974, 32767, command, param1, param2, param3);
  RoboCom::parseMessage("?SPD -1974 1974 32767;", command, param1, param2, param3);
  checkResult(4, CMD_UNKNOWN, -1974, 1974, 32767, command, param1, param2, param3);
  RoboCom::parseMessage("!DIST -19.74 197a4 32768;", command, param1, param2, param3);
  checkResult(5, CMD_DIST_RESPONSE, -19, 197, -1, command, param1, param2, param3);
}
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_ARDUINO_NODE_NAME);
  ArduinoNode arduinoNode;
  arduinoNode.openSerial();
  //test();

  ros::Rate loop_rate(100); // 100 Hz
  while (ros::ok())
  {
    arduinoNode.readSerial(onReceiveSerialMessage);
    loop_rate.sleep();
    ros::spinOnce();
  }

  arduinoNode.closeSerial();
  return 0;
}
