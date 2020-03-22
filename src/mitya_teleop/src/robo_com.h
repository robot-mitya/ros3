/*
 * robo_cmd.h
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

#ifndef MITYA_TELEOP_SRC_ROBO_COM_H_
#define MITYA_TELEOP_SRC_ROBO_COM_H_

#define MAX_MESSAGE_SIZE 200
#define COMMAND_SEPARATOR ';'

enum Command
{
  CMD_UNKNOWN = 0,
  CMD_STATUS_REQUEST = 10,
  CMD_STATUS_RESPONSE = 20,
  CMD_MOTOR_LEFT = 30,
  CMD_MOTOR_RIGHT = 40,
  CMD_MOTOR_BOTH = 50,
  CMD_L1 = 60,
  CMD_L1_REQUEST = 70,
  CMD_L1_RESPONSE = 80,
  CMD_L2 = 90,
  CMD_L2_REQUEST = 100,
  CMD_L2_RESPONSE = 110,
  CMD_ENCL_REQUEST = 120,
  CMD_ENCR_REQUEST = 130,
  CMD_ENCB_REQUEST = 140,
  CMD_ENCL_RESPONSE = 150,
  CMD_ENCR_RESPONSE = 160,
  CMD_DIST_REQUEST = 170,
  CMD_DIST_RESPONSE = 180,
  CMD_SPD_REQUEST = 190,
  CMD_SPD_RESPONSE = 200,
  CMD_MCPS_REQUEST = 210,
  CMD_MCPS_RESPONSE = 220
};

enum StatusCode
{
  RET_OK = 0,
  RET_BAD_PARAMETER = 1,
  RET_TOO_MANY_WORDS = 2,
  RET_BAD_COMMAND = 3
};

class RoboCom
{
public:
  static char const* getStatusText(int status);
  static char* getDriveLeftCommand(signed char speed);
  static char* getDriveRightCommand(signed char speed);
  static char* getSwitchLed1Command();
  static char* getSwitchLed2Command();
  static char* getSwingTailCommand();
  static const char* getFaceAngryCommand();
  static const char* getFaceBlueCommand();
  static const char* getFaceHappyCommand();
  static const char* getFaceIllCommand();
  static const char* getFaceOkCommand();
  static char* getRebootCommand();
  static void parseMessage(const char* message, Command &command, int &param1, int &param2, int &param3);
private:
  static char driveLeftMessage_[MAX_MESSAGE_SIZE];
  static char driveRightMessage_[MAX_MESSAGE_SIZE];
  static char led1Message_[MAX_MESSAGE_SIZE];
  static char led2Message_[MAX_MESSAGE_SIZE];
  static char tailMessage_[MAX_MESSAGE_SIZE];
  static char rebootMessage_[MAX_MESSAGE_SIZE];
  static Command getCommand(char *text);
};

#endif /* MITYA_TELEOP_SRC_ROBO_COM_H_ */
