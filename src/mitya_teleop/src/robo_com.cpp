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

char RoboCom::driveLeftMessage_[MAX_MESSAGE_SIZE];
char RoboCom::driveRightMessage_[MAX_MESSAGE_SIZE];
char RoboCom::led1Message_[MAX_MESSAGE_SIZE];
char RoboCom::led2Message_[MAX_MESSAGE_SIZE];
char RoboCom::tailMessage_[MAX_MESSAGE_SIZE];
char RoboCom::rebootMessage_[MAX_MESSAGE_SIZE];

static char charArray[2] = "\0";
static char commandText[11] = "";
static char param1Text[7] = "";
static char param2Text[7] = "";
static char param3Text[7] = "";
static int wordCounter = 0;
static bool inWordSeparator = false;

char const* RoboCom::getStatusText(int status)
{
  switch (status)
  {
    case RET_OK:
      return "OK";
    case RET_BAD_PARAMETER:
      return "Bad parameter";
    case RET_TOO_MANY_WORDS:
      return "Too many words";
    case RET_BAD_COMMAND:
      return "Bad command";
    default:
      return "Unknown error";
  }
}

char* RoboCom::getDriveLeftCommand(signed char speed)
{
  sprintf(RoboCom::driveLeftMessage_, "ML %d;", speed);
  return RoboCom::driveLeftMessage_;
}

char* RoboCom::getDriveRightCommand(signed char speed)
{
  sprintf(RoboCom::driveRightMessage_, "MR %d;", speed);
  return RoboCom::driveRightMessage_;
}

char* RoboCom::getSwitchLed1Command()
{
  sprintf(RoboCom::led1Message_, "L1 %d;", -1);
  return RoboCom::led1Message_;
}

char* RoboCom::getSwitchLed2Command()
{
  sprintf(RoboCom::led2Message_, "L2 %d;", -1);
  return RoboCom::led2Message_;
}

char* RoboCom::getSwingTailCommand()
{
  sprintf(RoboCom::tailMessage_, "TLSA %d %d %d;", 300, 30, 10);
  return RoboCom::tailMessage_;
}

const char* RoboCom::getFaceAngryCommand()
{
  return "angry";
}

const char* RoboCom::getFaceBlueCommand()
{
  return "blue";
}

const char* RoboCom::getFaceHappyCommand()
{
  return "happy";
}

const char* RoboCom::getFaceIllCommand()
{
  return "ill";
}

const char* RoboCom::getFaceOkCommand()
{
  return "ok";
}

char* RoboCom::getRebootCommand()
{
  sprintf(RoboCom::rebootMessage_, "RST;");
  return RoboCom::rebootMessage_;
}

void RoboCom::parseMessage(const char* message, Command &command, int &param1, int &param2, int &param3)
{
  command = CMD_UNKNOWN;
  param1 = 0;
  param2 = 0;
  param3 = 0;

  char ch;
  strcpy(commandText, "");
  strcpy(param1Text, "");
  strcpy(param2Text, "");
  strcpy(param3Text, "");
  wordCounter = 0;
  inWordSeparator = false;
  int messageLength = strlen(message);
  for (int i = 0; i < messageLength; i++)
  {
    ch = message[i];
    if (ch == COMMAND_SEPARATOR)
    {
      command = getCommand(commandText);
      param1 = atoi(param1Text);
      param2 = atoi(param2Text);
      param3 = atoi(param3Text);
      return;
    }
    else
    {
      if (ch <= 32) //(white space)
      {
        if (!inWordSeparator)
        {
          inWordSeparator = true;
          wordCounter++;
        }
        continue;
      }

      inWordSeparator = false;
      charArray[0] = ch;
      switch (wordCounter)
      {
        case 0:
          strcat(commandText, charArray);
          break;
        case 1:
          strcat(param1Text, charArray);
          break;
        case 2:
          strcat(param2Text, charArray);
          break;
        case 3:
          strcat(param3Text, charArray);
          break;
      }
    }
  }
}

Command RoboCom::getCommand(char *text)
{
  if (strlen(text) == 0)
    return CMD_UNKNOWN;
  if (strcmp(text, "!") == 0)
    return CMD_STATUS_RESPONSE;
  if (strcmp(text, "!L1") == 0)
    return CMD_L1_RESPONSE;
  if (strcmp(text, "!L2") == 0)
    return CMD_L2_RESPONSE;
  if (strcmp(text, "!DIST") == 0)
    return CMD_DIST_RESPONSE;
  if (strcmp(text, "!SPD") == 0)
    return CMD_SPD_RESPONSE;
  return CMD_UNKNOWN;
}
