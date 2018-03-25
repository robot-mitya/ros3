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

#include <stdint.h>

//#define MAX_MESSAGE_SIZE 200
//#define COMMAND_SEPARATOR ';'

enum Command
{
  CMD_UNKNOWN         = 0x00,
  CMD_MOTOR_LEFT      = 0x01,
  CMD_MOTOR_RIGHT     = 0x02,
  CMD_MOTOR_BOTH      = 0x03,

  CMD_TAIL_ROTATE     = 0x08,
  CMD_TAIL_SWING      = 0x09,
  CMD_TAIL_SWING_A    = 0x0A,
  CMD_TAIL_FREEZE     = 0x0F,

  CMD_LED_1           = 0x10,
  CMD_LED_1_REQUEST   = 0x11,
  CMD_LED_1_RESPONSE  = 0x12,
  CMD_LED_2           = 0x13,
  CMD_LED_2_REQUEST   = 0x14,
  CMD_LED_2_RESPONSE  = 0x15,

  CMD_ENCL_REQUEST    = 0x40,
  CMD_ENCR_REQUEST    = 0x41,
  CMD_ENCB_REQUEST    = 0x42,
  CMD_ENCL_RESPONSE   = 0x43,
  CMD_ENCR_RESPONSE   = 0x44,
  CMD_DIST_REQUEST    = 0x45,
  CMD_DIST_RESPONSE   = 0x46,
  CMD_SPD_REQUEST     = 0x47,
  CMD_SPD_RESPONSE    = 0x48,
  CMD_MCPS_REQUEST    = 0x49,
  CMD_MCPS_RESPONSE   = 0x4A,

  CMD_BV_REQUEST      = 0x50,
  CMD_BV_RESPONSE     = 0x51,
  CMD_DV_REQUEST      = 0x52,
  CMD_DV_RESPONSE     = 0x53,
  CMD_BVF_REQUEST     = 0x54,
  CMD_BVF_RESPONSE    = 0x55,
  CMD_DVF_REQUEST     = 0x56,
  CMD_DVF_RESPONSE    = 0x57,

  CMD_RESET           = 0xF0,
  CMD_STATUS_REQUEST  = 0xF1,
  CMD_STATUS_RESPONSE = 0xF2
};

enum StatusCode
{
  RET_OK = 0,
  RET_BAD_PARAMETER = 1,
  RET_WRONG_PARAMS_COUNT = 2,
  RET_BAD_COMMAND = 3,
  RET_NOISE_RECEIVED = 4,
  RET_CS_ERROR = 5
};

class RoboCom
{
public:
  static const int MAX_MESSAGE_SIZE = 12;
  static const int MAX_TEXT_MESSAGE_SIZE = 200;
  static const char COMMAND_SEPARATOR = ';';

  // buildDriveLeftCommand fills left drive message array, returns message length:
  static int buildDriveLeftCommand(signed char speed, uint8_t* message);
  // buildDriveRightCommand fills right drive message array, returns message length:
  static int buildDriveRightCommand(signed char speed, uint8_t* message);

  static char const* getStatusText(int status);
  static int buildBinaryMessage(Command command, int param1, int param2, int param3, uint8_t* message);
  static int buildSwitchLed1Message(uint8_t* message);
  static int buildSwitchLed2Message(uint8_t* message);
  static int buildSwingTailMessage(uint8_t* message);
  static int buildRebootMessage(uint8_t* message);

  static const char* buildTextMessage(Command command);
  static const char* buildTextMessage(Command command, int param1);
  static const char* buildTextMessage(Command command, int param1, int param2);
  static const char* buildTextMessage(Command command, int param1, int param2, int param3);

  static const char* buildSwitchLed1TextMessage();
  static const char* buildSwitchLed2TextMessage();
  static const char* buildSwingTailTextMessage();
  static const char* buildRebootTextMessage();

  static StatusCode parseTextMessage(const char* message, Command &command, int &param1, int &param2, int &param3);
  static StatusCode parseBinaryMessage(uint8_t* message, int size, Command &command, int &param1, int &param2, int &param3);
private:
  static int checksum1(uint8_t* data, int size);
  static int checksum2(int cs1);
  static int buildDriveMessage(Command command, signed char speed, uint8_t* message);
  static int buildSwitchLedMessage(Command command, uint8_t* message);
  static Command getCommand(char *commandText);
  static const char* getCommandText(Command command);
};

#endif /* MITYA_TELEOP_SRC_ROBO_COM_H_ */
