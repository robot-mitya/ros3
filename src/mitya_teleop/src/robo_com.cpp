/*
 * robo_cmd.cpp
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
 *  Created on: Jul 26, 2017
 *      Author: Dmitry Dzakhov
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robo_com.h"

//#include "ros/ros.h"

#define LOBYTE(v)        ((unsigned char) (v))
#define HIBYTE(v)        ((unsigned char) (((unsigned int) (v)) >> 8))
#define MAKEWORD(lo, hi) ((uint16_t) (lo) | ((uint16_t) (hi) << 8))

static uint8_t message_[RoboCom::MAX_MESSAGE_SIZE];
static int messagePos_ = 0;
static int messageSize_;
static int errorStatus_ = 0; // 0 means no error.

static char textMessage_[RoboCom::MAX_TEXT_MESSAGE_SIZE];

static const int NO_PARAMS_MESSAGE_SIZE = 6;
static const int ONE_PARAM_MESSAGE_SIZE = 8;
static const int TWO_PARAMS_MESSAGE_SIZE = 10;
static const int THREE_PARAMS_MESSAGE_SIZE = 12;

static const int HEADER_INDEX_0 = 0;
static const int HEADER_INDEX_1 = 1;
static const int SIZE_INDEX = 2;
static const int COMMAND_INDEX = 3;
static const int CS_1_INDEX = 4;
static const int CS_2_INDEX = 5;
static const int PARAM_1_LSB_INDEX = 6;
static const int PARAM_1_MSB_INDEX = 7;
static const int PARAM_2_LSB_INDEX = 8;
static const int PARAM_2_MSB_INDEX = 9;
static const int PARAM_3_LSB_INDEX = 10;
static const int PARAM_3_MSB_INDEX = 11;

static char charArray_[2] = "\0";
static char commandText_[11] = "";
static char param1Text_[7] = "";
static char param2Text_[7] = "";
static char param3Text_[7] = "";
static int wordCounter_ = 0;
static bool inWordSeparator_ = false;

char const* RoboCom::getStatusText(int status)
{
  switch (status)
  {
    case RET_OK:
      return "OK";
    case RET_BAD_PARAMETER:
      return "Bad parameter";
    case RET_WRONG_PARAMS_COUNT:
      return "Wrong number of parameters";
    case RET_BAD_COMMAND:
      return "Bad command";
    case RET_NOISE_RECEIVED:
      return "Noise received";
    case RET_CS_ERROR:
      return "Checksum error";
    default:
      return "Unknown error";
  }
}

int RoboCom::checksum1(uint8_t* data, int size)
{
  return 0x00;
}

int RoboCom::checksum2(int cs1)
{
  return 0x00;
}

int RoboCom::buildBinaryMessage(Command command, int param1, int param2, int param3, uint8_t* message)
{
  message[HEADER_INDEX_0] = 0xFF;
  message[HEADER_INDEX_1] = 0xFF;
  int length = THREE_PARAMS_MESSAGE_SIZE;
  message[SIZE_INDEX] = length;
  message[COMMAND_INDEX] = command;
  message[PARAM_1_LSB_INDEX] = LOBYTE(param1);
  message[PARAM_1_MSB_INDEX] = HIBYTE(param1);
  message[PARAM_2_LSB_INDEX] = LOBYTE(param2);
  message[PARAM_2_MSB_INDEX] = HIBYTE(param2);
  message[PARAM_3_LSB_INDEX] = LOBYTE(param3);
  message[PARAM_3_MSB_INDEX] = HIBYTE(param3);
  message[CS_1_INDEX] = checksum1(message, length);
  message[CS_2_INDEX] = checksum2(message[CS_1_INDEX]);
  return length;
}

int RoboCom::buildDriveMessage(Command command, signed char speed, uint8_t* message)
{
  message[HEADER_INDEX_0] = 0xFF;
  message[HEADER_INDEX_1] = 0xFF;
  int length = ONE_PARAM_MESSAGE_SIZE;
  message[SIZE_INDEX] = length;
  message[COMMAND_INDEX] = command;
  message[PARAM_1_LSB_INDEX] = speed;
  message[PARAM_1_MSB_INDEX] = 0x00;
  message[CS_1_INDEX] = checksum1(message, length);
  message[CS_2_INDEX] = checksum2(message[CS_1_INDEX]);
  return length;
}

int RoboCom::buildDriveLeftCommand(signed char speed, uint8_t* message)
{
  return buildDriveMessage(CMD_MOTOR_LEFT, speed, message);
}

int RoboCom::buildDriveRightCommand(signed char speed, uint8_t* message)
{
  return buildDriveMessage(CMD_MOTOR_RIGHT, speed, message);
}

int RoboCom::buildSwitchLedMessage(Command command, uint8_t* message)
{
  message[HEADER_INDEX_0] = 0xFF;
  message[HEADER_INDEX_1] = 0xFF;
  int length = ONE_PARAM_MESSAGE_SIZE;
  message[SIZE_INDEX] = length;
  message[COMMAND_INDEX] = command;
  int value = -1;
  message[PARAM_1_LSB_INDEX] = LOBYTE(value);
  message[PARAM_1_MSB_INDEX] = HIBYTE(value);
  message[CS_1_INDEX] = checksum1(message, length);
  message[CS_2_INDEX] = checksum2(message[CS_1_INDEX]);
  return length;
}

int RoboCom::buildSwitchLed1Message(uint8_t* message)
{
  return buildSwitchLedMessage(CMD_LED_1, message);
}

int RoboCom::buildSwitchLed2Message(uint8_t* message)
{
  return buildSwitchLedMessage(CMD_LED_2, message);
}

int RoboCom::buildSwingTailMessage(uint8_t* message)
{
  message[HEADER_INDEX_0] = 0xFF;
  message[HEADER_INDEX_1] = 0xFF;
  int length = THREE_PARAMS_MESSAGE_SIZE;
  message[SIZE_INDEX] = length;
  message[COMMAND_INDEX] = CMD_TAIL_SWING_A;
  int period = 300;
  int amplitude = 30;
  int count = 10;
  message[PARAM_1_LSB_INDEX] = LOBYTE(period);
  message[PARAM_1_MSB_INDEX] = HIBYTE(period);
  message[PARAM_2_LSB_INDEX] = LOBYTE(amplitude);
  message[PARAM_2_MSB_INDEX] = HIBYTE(amplitude);
  message[PARAM_3_LSB_INDEX] = LOBYTE(count);
  message[PARAM_3_MSB_INDEX] = HIBYTE(count);
  message[CS_1_INDEX] = checksum1(message, length);
  message[CS_2_INDEX] = checksum2(message[CS_1_INDEX]);
  return length;
}

int RoboCom::buildRebootMessage(uint8_t* message)
{
  message[HEADER_INDEX_0] = 0xFF;
  message[HEADER_INDEX_1] = 0xFF;
  int length = NO_PARAMS_MESSAGE_SIZE;
  message[SIZE_INDEX] = length;
  message[COMMAND_INDEX] = CMD_RESET;
  message[CS_1_INDEX] = checksum1(message, length);
  message[CS_2_INDEX] = checksum2(message[CS_1_INDEX]);
  return length;
}

//TODO Process several commands in one message.
/*  int n = read(fd, buffer, SERIAL_BUFFER_SIZE);
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
  }*/
StatusCode RoboCom::parseTextMessage(const char* message, Command &command, int &param1, int &param2, int &param3)
{
  command = CMD_UNKNOWN;
  param1 = 0;
  param2 = 0;
  param3 = 0;

  char ch;
  strcpy(commandText_, "");
  strcpy(param1Text_, "");
  strcpy(param2Text_, "");
  strcpy(param3Text_, "");
  wordCounter_ = 0;
  inWordSeparator_ = false;
  int messageLength = strlen(message);
  for (int i = 0; i < messageLength; i++)
  {
    ch = message[i];
    if (ch == COMMAND_SEPARATOR)
    {
      command = getCommand(commandText_);
      param1 = atoi(param1Text_);
      param2 = atoi(param2Text_);
      param3 = atoi(param3Text_);
      return RET_OK;
    }
    else
    {
      if (ch <= 32) //(white space)
      {
        if (!inWordSeparator_)
        {
          inWordSeparator_ = true;
          wordCounter_++;
        }
        continue;
      }

      inWordSeparator_ = false;
      charArray_[0] = ch;
      switch (wordCounter_)
      {
        case 0:
          strcat(commandText_, charArray_);
          break;
        case 1:
          strcat(param1Text_, charArray_);
          break;
        case 2:
          strcat(param2Text_, charArray_);
          break;
        case 3:
          strcat(param3Text_, charArray_);
          break;
      }
    }
  }
  return RET_BAD_COMMAND;
}

Command RoboCom::getCommand(char *commandText)
{
  if (strlen(commandText) == 0)
    return CMD_UNKNOWN;

  if (strcmp(commandText, "ML") == 0)
    return CMD_MOTOR_LEFT;
  if (strcmp(commandText, "MR") == 0)
    return CMD_MOTOR_RIGHT;
  if (strcmp(commandText, "MB") == 0)
    return CMD_MOTOR_BOTH;

  if (strcmp(commandText, "TLR") == 0)
    return CMD_TAIL_ROTATE;
  if (strcmp(commandText, "TLS") == 0)
    return CMD_TAIL_SWING;
  if (strcmp(commandText, "TLSA") == 0)
    return CMD_TAIL_SWING_A;
  if (strcmp(commandText, "TLF") == 0)
    return CMD_TAIL_FREEZE;

  if (strcmp(commandText, "L1") == 0)
    return CMD_LED_1;
  if (strcmp(commandText, "?L1") == 0)
    return CMD_LED_1_REQUEST;
  if (strcmp(commandText, "!L1") == 0)
    return CMD_LED_1_RESPONSE;
  if (strcmp(commandText, "L2") == 0)
    return CMD_LED_2;
  if (strcmp(commandText, "?L2") == 0)
    return CMD_LED_2_REQUEST;
  if (strcmp(commandText, "?L1") == 0)
    return CMD_LED_2_RESPONSE;

  if (strcmp(commandText, "?ENCL") == 0)
    return CMD_ENCL_REQUEST;
  if (strcmp(commandText, "?ENCR") == 0)
    return CMD_ENCR_REQUEST;
  if (strcmp(commandText, "?ENCB") == 0)
    return CMD_ENCB_REQUEST;
  if (strcmp(commandText, "!ENCL") == 0)
    return CMD_ENCL_RESPONSE;
  if (strcmp(commandText, "!ENCR") == 0)
    return CMD_ENCR_RESPONSE;
  if (strcmp(commandText, "?DIST") == 0)
    return CMD_DIST_REQUEST;
  if (strcmp(commandText, "!DIST") == 0)
    return CMD_DIST_RESPONSE;
  if (strcmp(commandText, "?SPD") == 0)
    return CMD_SPD_REQUEST;
  if (strcmp(commandText, "!SPD") == 0)
    return CMD_SPD_RESPONSE;
  if (strcmp(commandText, "?MCPS") == 0)
    return CMD_MCPS_REQUEST;
  if (strcmp(commandText, "!MCPS") == 0)
    return CMD_MCPS_RESPONSE;

  if (strcmp(commandText, "?BV") == 0)
    return CMD_BV_REQUEST;
  if (strcmp(commandText, "!BV") == 0)
    return CMD_BV_RESPONSE;
  if (strcmp(commandText, "?DV") == 0)
    return CMD_DV_REQUEST;
  if (strcmp(commandText, "!DV") == 0)
    return CMD_DV_RESPONSE;
  if (strcmp(commandText, "?BVF") == 0)
    return CMD_BVF_REQUEST;
  if (strcmp(commandText, "!BVF") == 0)
    return CMD_BVF_RESPONSE;
  if (strcmp(commandText, "?DVF") == 0)
    return CMD_DVF_REQUEST;
  if (strcmp(commandText, "!DVF") == 0)
    return CMD_DVF_RESPONSE;

  if (strcmp(commandText, "RST") == 0)
    return CMD_RESET;
  if (strcmp(commandText, "?") == 0)
    return CMD_STATUS_REQUEST;
  if (strcmp(commandText, "!") == 0)
    return CMD_STATUS_RESPONSE;

  return CMD_UNKNOWN;
}

const char* RoboCom::getCommandText(Command command)
{
  switch (command)
  {
    case CMD_MOTOR_LEFT:
      return  "ML";
    case CMD_MOTOR_RIGHT:
      return "MR";
    case CMD_MOTOR_BOTH:
      return "MB";
    case CMD_TAIL_ROTATE:
      return "TLR";
    case CMD_TAIL_SWING:
      return "TLS";
    case CMD_TAIL_SWING_A:
      return "TLSA";
    case CMD_TAIL_FREEZE:
      return "TLF";
    case CMD_LED_1:
      return "L1";
    case CMD_LED_1_REQUEST:
      return "?L1";
    case CMD_LED_1_RESPONSE:
      return "!L1";
    case CMD_LED_2:
      return "L2";
    case CMD_LED_2_REQUEST:
      return "?L2";
    case CMD_LED_2_RESPONSE:
      return "!L2";
    case CMD_ENCL_REQUEST:
      return "?ENCL";
    case CMD_ENCR_REQUEST:
      return "?ENCR";
    case CMD_ENCB_REQUEST:
      return "?ENCB";
    case CMD_ENCL_RESPONSE:
      return "!ENCL";
    case CMD_ENCR_RESPONSE:
      return "!ENCR";
    case CMD_DIST_REQUEST:
      return "?DIST";
    case CMD_DIST_RESPONSE:
      return "!DIST";
    case CMD_SPD_REQUEST:
      return "?SPD";
    case CMD_SPD_RESPONSE:
      return "!SPD";
    case CMD_MCPS_REQUEST:
      return "?MCPS";
    case CMD_MCPS_RESPONSE:
      return "!MCPS";
    case CMD_BV_REQUEST:
      return "?BV";
    case CMD_BV_RESPONSE:
      return "!BV";
    case CMD_DV_REQUEST:
      return "?DV";
    case CMD_DV_RESPONSE:
      return "!DV";
    case CMD_BVF_REQUEST:
      return "?BVF";
    case CMD_BVF_RESPONSE:
      return "!BVF";
    case CMD_DVF_REQUEST:
      return "?DVF";
    case CMD_DVF_RESPONSE:
      return "!DVF";
    case CMD_RESET:
      return "RST";
    case CMD_STATUS_REQUEST:
      return "?";
    case CMD_STATUS_RESPONSE:
      return "!";
    default:
      return "";
  }
}

StatusCode RoboCom::parseBinaryMessage(uint8_t* message, int size, Command &command, int &param1, int &param2, int &param3)
{
  if ((size != NO_PARAMS_MESSAGE_SIZE) ||
      (size != ONE_PARAM_MESSAGE_SIZE) ||
      (size != TWO_PARAMS_MESSAGE_SIZE) ||
      (size != THREE_PARAMS_MESSAGE_SIZE)) return RET_WRONG_PARAMS_COUNT;
  param1 = 0;
  param2 = 0;
  param3 = 0;

  if ((message[HEADER_INDEX_0] != 0xFF) && (message[HEADER_INDEX_1] != 0xFF))
    return RET_NOISE_RECEIVED;

  command = (Command) message[COMMAND_INDEX];
  if (size >= ONE_PARAM_MESSAGE_SIZE)
    param1 = MAKEWORD(message[PARAM_1_LSB_INDEX], message[PARAM_1_MSB_INDEX]);
  if (size >= TWO_PARAMS_MESSAGE_SIZE)
    param2 = MAKEWORD(message[PARAM_2_LSB_INDEX], message[PARAM_2_MSB_INDEX]);
  if (size >= THREE_PARAMS_MESSAGE_SIZE)
    param3 = MAKEWORD(message[PARAM_3_LSB_INDEX], message[PARAM_3_MSB_INDEX]);
  return RET_OK;
}

const char* RoboCom::buildTextMessage(Command command)
{
  sprintf(textMessage_, "%s;", getCommandText(command));
  return textMessage_;
}

const char* RoboCom::buildTextMessage(Command command, int param1)
{
  sprintf(textMessage_, "%s %d;", getCommandText(command), param1);
  return textMessage_;
}

const char* RoboCom::buildTextMessage(Command command, int param1, int param2)
{
  sprintf(textMessage_, "%s %d %d;", getCommandText(command), param1, param2);
  return textMessage_;
}

const char* RoboCom::buildTextMessage(Command command, int param1, int param2, int param3)
{
  sprintf(textMessage_, "%s %d %d %d;", getCommandText(command), param1, param2, param3);
  return textMessage_;
}

const char* RoboCom::buildSwitchLed1TextMessage()
{
  return buildTextMessage(CMD_LED_1, -1);
}

const char* RoboCom::buildSwitchLed2TextMessage()
{
  return buildTextMessage(CMD_LED_2, -1);
}

const char* RoboCom::buildSwingTailTextMessage()
{
  return buildTextMessage(CMD_TAIL_SWING_A, 300, 30, 10);
}

const char* RoboCom::buildRebootTextMessage()
{
  return buildTextMessage(CMD_RESET);
}
