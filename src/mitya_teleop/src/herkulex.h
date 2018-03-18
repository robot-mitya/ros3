/*
 * Herkulex.h
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
 *  Created on: Aug 4, 2017
 *      Author: Dmitry Dzakhov
 */

#ifndef MITYA_TELEOP_SRC_HERKULEX_H_
#define MITYA_TELEOP_SRC_HERKULEX_H_

#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#define DATA_SIZE        30             // buffer for input data
#define DATA_MOVE        50             // max 10 servos <---- change this for more servos!
#define DATA_MOVE_ALL    58
#define TIME_OUT     5          //timeout serial communication

// SERVO HERKULEX COMMAND - See Manual p40
#define HEEPWRITE    0x01       //ROM write
#define HEEPREAD     0x02       //ROM read
#define HRAMWRITE        0x03   //Ram write
#define HRAMREAD         0x04   //Ram read
#define HIJOG            0x05   //Write n servo with different timing
#define HSJOG            0x06   //Write n servo with same time
#define HSTAT            0x07   //Read error
#define HROLLBACK        0x08   //Back to factory value
#define HREBOOT          0x09   //Reboot

// HERKULEX LED - See Manual p29
static int LED_OFF = 0x00;
static int LED_GREEN = 0x01;
static int LED_BLUE = 0x02;
static int LED_CYAN = 0x03;
static int LED_RED = 0x04;
static int LED_GREEN2 = 0x05;
static int LED_PINK = 0x06;
static int LED_WHITE = 0x07;

// HERKULEX STATUS ERROR - See Manual p39
static uint8_t H_STATUS_OK = 0x00;
static uint8_t H_ERROR_INPUT_VOLTAGE = 0x01;
static uint8_t H_ERROR_POS_LIMIT = 0x02;
static uint8_t H_ERROR_TEMPERATURE_LIMIT = 0x04;
static uint8_t H_ERROR_INVALID_PKT = 0x08;
static uint8_t H_ERROR_OVERLOAD = 0x10;
static uint8_t H_ERROR_DRIVER_FAULT = 0x20;
static uint8_t H_ERROR_EEPREG_DISTORT = 0x40;

// HERKULEX Broadcast Servo ID
static uint8_t BROADCAST_ID = 0xFE;

enum TorqueState
{
  TS_TORQUE_FREE = 0x00, TS_BREAK_ON = 0x40, TS_TORQUE_ON = 0x60
};

class HerkulexClass
{
public:
  HerkulexClass();
  ~HerkulexClass();

  bool begin(char const* portName, long baud);
  void end();

  void initialize();
  uint8_t stat(int servoID, uint8_t *statusError, uint8_t *statusDetail);
  void ACK(int valueACK);
  uint8_t model();
  void set_ID(int ID_Old, int ID_New);
  void clearError(int servoID);

  void torqueState(int servoID, TorqueState state);
  void setTorqueFree(int servoID);
  void setBreakOn(int servoID);
  void setTorqueOn(int servoID);

  void moveAll(int servoID, int Goal, int iLed);
  void moveSpeedAll(int servoID, int Goal, int iLed);
  void moveAllAngle(int servoID, float angle, int iLed);
  void actionAll(int pTime);

  void moveSpeedOne(int servoID, int Goal, int pTime, int iLed);
  void moveOne(int servoID, int Goal, int pTime, int iLed);
  void moveOneAngle(int servoID, float angle, int pTime, int iLed);

  int getPosition(int servoID);
  float getAngle(int servoID);
  int getSpeed(int servoID);

  void reboot(int servoID);
  void setLed(int servoID, int valueLed);

  void writeRegistryRAM(int servoID, int address, int writeByte);
  void writeRegistryEEP(int servoID, int address, int writeByte);

// private area
private:
  void sendData(uint8_t* buffer, int length);
  bool readData(int size);
  void addData(int GoalLSB, int GoalMSB, int set, int servoID);
  int checksum1(uint8_t* data, int lengthString);
  int checksum2(int XOR);
  void delay(long millis);

  int pSize;
  int pID;
  int cmd;
  int lengthString;
  int ck1;
  int ck2;

  int conta;

  int XOR;
  int playTime;

  uint8_t data[DATA_SIZE];
  uint8_t dataEx[DATA_MOVE_ALL];
  uint8_t moveData[DATA_MOVE];

  int fd;
  bool isPortOpened;
  bool setInterfaceAttribs(int fd, int speed, int parity);
  bool setBlocking(int fd, int should_block);
  int baudRateToBaudRateConst(int baudRate);
};

extern HerkulexClass Herkulex;

#endif /* MITYA_TELEOP_SRC_HERKULEX_H_ */
