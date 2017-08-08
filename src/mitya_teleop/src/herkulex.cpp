/*
 * Herkulex.cpp
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

#include "herkulex.h"

#include <unistd.h>
#include <string.h>
#include <time.h>

// Herkulex serial port initialization
bool HerkulexClass::begin(char const* portName, long baud)
{
  fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
    return false;
  if (!setInterfaceAttribs(fd, baudRateToBaudRateConst(baud), 0)) // set speed bps, 8n1 (no parity)
    return false;
  if (!setBlocking(fd, 0)) // set no blocking
    return false;

  return true;
}

// Herkulex serial port initialization
void HerkulexClass::end()
{
}

// initialize servos
void HerkulexClass::initialize()
{
  conta = 0;
  lengthString = 0;
  delay(100);
  clearError(BROADCAST_ID); // clear error for all servos
  delay(10);
  ACK(1); // set ACK
  delay(10);
  torqueON(BROADCAST_ID); // torqueON for all servos
  delay(10);
}

// stat
uint8_t HerkulexClass::stat(int servoID, uint8_t *statusError, uint8_t *statusDetail)
{
  pSize = 0x07; //3.Packet size
  pID = servoID; //4.Servo ID - 0XFE=All servos
  cmd = HSTAT; //5.CMD

  ck1 = (pSize ^ pID ^ cmd) & 0xFE;
  ck2 = (~(pSize ^ pID ^ cmd)) & 0xFE;

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2

  sendData(dataEx, pSize);
  delay(2);
  readData(9); // read 9 bytes from serial

  pSize = dataEx[2]; // 3.Packet size 7-58
  pID = dataEx[3]; // 4. Servo ID
  cmd = dataEx[4]; // 5. CMD
  data[0] = dataEx[7];
  data[1] = dataEx[8];
  lengthString = 2;

  ck1 = (dataEx[2] ^ dataEx[3] ^ dataEx[4] ^ dataEx[7] ^ dataEx[8]) & 0xFE;
  ck2 = checksum2(ck1);

  if (ck1 != dataEx[5])
    return -1; //checksum verify
  if (ck2 != dataEx[6])
    return -2;

  if (statusError != NULL)
    *statusError = dataEx[7];
  if (statusDetail != NULL)
    *statusDetail = dataEx[8];
  return 0;
}

// torque on -
void HerkulexClass::torqueON(int servoID)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID
  cmd = HRAMWRITE; // 5. CMD
  data[0] = 0x34; // 8. Address
  data[1] = 0x01; // 9. Length
  data[2] = 0x60; // 10. 0x60=Torque ON
  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Torque ON

  sendData(dataEx, pSize);
}

// torque off - the torque is FREE, not Break
void HerkulexClass::torqueOFF(int servoID)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID
  cmd = HRAMWRITE; // 5. CMD
  data[0] = 0x34; // 8. Address
  data[1] = 0x01; // 9. Length
  data[2] = 0x00; // 10. 0x00=Torque Free
  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Torque Free

  sendData(dataEx, pSize);
}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::ACK(int valueACK)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = 0xFE; // 4. Servo ID
  cmd = HRAMWRITE; // 5. CMD
  data[0] = 0x34; // 8. Address
  data[1] = 0x01; // 9. Length
  data[2] = valueACK; // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Value

  sendData(dataEx, pSize);
}

// model - 1=0101 - 2=0201
uint8_t HerkulexClass::model()
{
  pSize = 0x09; // 3.Packet size 7-58
  pID = 0xFE; // 4. Servo ID
  cmd = HEEPREAD; // 5. CMD
  data[0] = 0x00; // 8. Address
  data[1] = 0x01; // 9. Length
  lengthString = 2; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address
  dataEx[8] = data[1]; // Length

  sendData(dataEx, pSize);

  delay(1);
  readData(9);

  pSize = dataEx[2]; // 3.Packet size 7-58
  pID = dataEx[3]; // 4. Servo ID
  cmd = dataEx[4]; // 5. CMD
  data[0] = dataEx[7]; // 8. 1st uint8_t
  lengthString = 1; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  if (ck1 != dataEx[5])
    return -1; //checksum verify
  if (ck2 != dataEx[6])
    return -2;

  return dataEx[7]; // return status
}

// setID - Need to restart the servo
void HerkulexClass::set_ID(int ID_Old, int ID_New)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = ID_Old; // 4. Servo ID OLD - original servo ID
  cmd = HEEPWRITE; // 5. CMD
  data[0] = 0x06; // 8. Address
  data[1] = 0x01; // 9. Length
  data[2] = ID_New; // 10. ServoID NEW
  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Value

  sendData(dataEx, pSize);
}

// clearError
void HerkulexClass::clearError(int servoID)
{
  pSize = 0x0B; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID - 253=all servos
  cmd = HRAMWRITE; // 5. CMD
  data[0] = 0x30; // 8. Address
  data[1] = 0x02; // 9. Length
  data[2] = 0x00; // 10. Write error=0
  data[3] = 0x00; // 10. Write detail error=0

  lengthString = 4; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Value1
  dataEx[10] = data[3]; // Value2

  sendData(dataEx, pSize);
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAll(int servoID, int Goal, int iLed)
{
  if (Goal > 1023 || Goal < 0)
    return; //0 <--> 1023 range

  int iMode = 0; //mode=position
  int iStop = 0; //stop=0

  // Position definition
  int posLSB = Goal & 0X00FF; // MSB Pos
  int posMSB = (Goal & 0XFF00) >> 8; // LSB Pos

  //led
  int iBlue = 0;
  int iGreen = 0;
  int iRed = 0;
  switch (iLed)
  {
    case 1:
      iGreen = 1;
      break;
    case 2:
      iBlue = 1;
      break;
    case 3:
      iRed = 1;
      break;
  }

  int SetValue = iStop + iMode * 2 + iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

  addData(posLSB, posMSB, SetValue, servoID); //add servo data to list, pos mode
}

// move all servo at the same time to a position: servo list building
void HerkulexClass::moveAllAngle(int servoID, float angle, int iLed)
{
  if (angle > 160.0 || angle < -160.0)
    return; // out of the range
  int position = (int)(angle / 0.325) + 512;
  moveAll(servoID, position, iLed);
}

// move all servo at the same time with different speeds: servo list building
void HerkulexClass::moveSpeedAll(int servoID, int Goal, int iLed)
{
  if (Goal > 1023 || Goal < -1023)
    return; //-1023 <--> 1023 range

  int iMode = 1; // mode=continous rotation
  int iStop = 0; // Stop=0

  // Speed definition
  int GoalSpeedSign;
  if (Goal < 0)
  {
    GoalSpeedSign = (-1) * Goal;
    GoalSpeedSign |= 0x4000; //bit n�14
  }
  else
  {
    GoalSpeedSign = Goal;
  }

  int speedGoalLSB = GoalSpeedSign & 0X00FF; // MSB speedGoal
  int speedGoalMSB = (GoalSpeedSign & 0xFF00) >> 8; // LSB speedGoal

  //led
  int iBlue = 0;
  int iGreen = 0;
  int iRed = 0;
  switch (iLed)
  {
    case 1:
      iGreen = 1;
      break;
    case 2:
      iBlue = 1;
      break;
    case 3:
      iRed = 1;
      break;
  }

  int SetValue = iStop + iMode * 2 + iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

  addData(speedGoalLSB, speedGoalMSB, SetValue, servoID); //add servo data to list, speed mode
}

// move all servo with the same execution time
void HerkulexClass::actionAll(int pTime)
{
  if ((pTime < 0) || (pTime > 2856))
    return;

  pSize = 0x08 + conta; // 3.Packet size 7-58
  cmd = HSJOG; // 5. CMD SJOG Write n servo with same execution time
  playTime = int((float)pTime / 11.2); // 8. Execution time

  pID = 0xFE ^ playTime;
  ck1 = checksum1(moveData, conta); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  pID = 0xFE;
  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = playTime; // Execution time

  for (int i = 0; i < conta; i++)
    dataEx[i + 8] = moveData[i]; // Variable servo data

  sendData(dataEx, pSize);

  conta = 0; //reset counter
}

// get Position
int HerkulexClass::getPosition(int servoID)
{
  int Position = 0;

  pSize = 0x09; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID - 253=all servos
  cmd = HRAMREAD; // 5. CMD
  data[0] = 0x3A; // 8. Address
  data[1] = 0x02; // 9. Length

  lengthString = 2; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address
  dataEx[8] = data[1]; // Length

  sendData(dataEx, pSize);

  delay(1);
  readData(13);

  pSize = dataEx[2]; // 3.Packet size 7-58
  pID = dataEx[3]; // 4. Servo ID
  cmd = dataEx[4]; // 5. CMD
  data[0] = dataEx[7];
  data[1] = dataEx[8];
  data[2] = dataEx[9];
  data[3] = dataEx[10];
  data[4] = dataEx[11];
  data[5] = dataEx[12];
  lengthString = 6;

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  if (ck1 != dataEx[5])
    return -1;
  if (ck2 != dataEx[6])
    return -1;

  Position = ((dataEx[10] & 0x03) << 8) | dataEx[9];
  return Position;
}

float HerkulexClass::getAngle(int servoID)
{
  int pos = (int)getPosition(servoID);
  return (pos - 512) * 0.325;
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void HerkulexClass::reboot(int servoID)
{

  pSize = 0x07; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID - 253=all servos
  cmd = HREBOOT; // 5. CMD
  ck1 = (pSize ^ pID ^ cmd) & 0xFE;
  ck2 = (~(pSize ^ pID ^ cmd)) & 0xFE;
  ;

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2

  sendData(dataEx, pSize);
}

// LED  - see table of colors
void HerkulexClass::setLed(int servoID, int valueLed)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID
  cmd = HRAMWRITE; // 5. CMD
  data[0] = 0x35; // 8. Address 53
  data[1] = 0x01; // 9. Length
  data[2] = valueLed; // 10.LedValue
  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Value

  sendData(dataEx, pSize);
}

// get the speed for one servo - values betweeb -1023 <--> 1023
int HerkulexClass::getSpeed(int servoID)
{
  int speedy = 0;

  pSize = 0x09; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID
  cmd = HRAMREAD; // 5. CMD
  data[0] = 0x40; // 8. Address
  data[1] = 0x02; // 9. Length

  lengthString = 2; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address
  dataEx[8] = data[1]; // Length

  sendData(dataEx, pSize);

  delay(1);
  readData(13);

  pSize = dataEx[2]; // 3.Packet size 7-58
  pID = dataEx[3]; // 4. Servo ID
  cmd = dataEx[4]; // 5. CMD
  data[0] = dataEx[7];
  data[1] = dataEx[8];
  data[2] = dataEx[9];
  data[3] = dataEx[10];
  data[4] = dataEx[11];
  data[5] = dataEx[12];
  lengthString = 6;

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  if (ck1 != dataEx[5])
    return -1;
  if (ck2 != dataEx[6])
    return -1;

  speedy = ((dataEx[10] & 0xFF) << 8) | dataEx[9];
  return speedy;
}

// move one servo with continous rotation
void HerkulexClass::moveSpeedOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < -1023)
    return; // speed (goal) non correct
  if ((pTime < 0) || (pTime > 2856))
    return;

  int GoalSpeedSign;
  if (Goal < 0)
  {
    GoalSpeedSign = (-1) * Goal;
    GoalSpeedSign |= 0x4000; //bit n�14
  }
  else
  {
    GoalSpeedSign = Goal;
  }
  int speedGoalLSB = GoalSpeedSign & 0X00FF; // MSB speedGoal
  int speedGoalMSB = (GoalSpeedSign & 0xFF00) >> 8; // LSB speedGoal

  //led
  int iBlue = 0;
  int iGreen = 0;
  int iRed = 0;
  switch (iLed)
  {
    case 1:
      iGreen = 1;
      break;
    case 2:
      iBlue = 1;
      break;
    case 3:
      iRed = 1;
      break;
  }
  int SetValue = 2 + iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

  playTime = int((float)pTime / 11.2); // 8. Execution time

  pSize = 0x0C; // 3.Packet size 7-58
  cmd = HSJOG; // 5. CMD

  data[0] = speedGoalLSB; // 8. speedLSB
  data[1] = speedGoalMSB; // 9. speedMSB
  data[2] = SetValue; // 10. Mode=0;
  data[3] = servoID; // 11. ServoID

  pID = servoID ^ playTime;

  lengthString = 4; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  pID = servoID;

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = playTime; // Execution time
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];

  sendData(dataEx, pSize);
}

// move one servo at goal position 0 - 1024
void HerkulexClass::moveOne(int servoID, int Goal, int pTime, int iLed)
{
  if (Goal > 1023 || Goal < 0)
    return; // speed (goal) non correct
  if ((pTime < 0) || (pTime > 2856))
    return;

  // Position definition
  int posLSB = Goal & 0X00FF; // MSB Pos
  int posMSB = (Goal & 0XFF00) >> 8; // LSB Pos

  //led
  int iBlue = 0;
  int iGreen = 0;
  int iRed = 0;
  switch (iLed)
  {
    case 1:
      iGreen = 1;
      break;
    case 2:
      iBlue = 1;
      break;
    case 3:
      iRed = 1;
      break;
  }
  int SetValue = iGreen * 4 + iBlue * 8 + iRed * 16; //assign led value

  playTime = int((float)pTime / 11.2); // 8. Execution time

  pSize = 0x0C; // 3.Packet size 7-58
  cmd = HSJOG; // 5. CMD

  data[0] = posLSB; // 8. speedLSB
  data[1] = posMSB; // 9. speedMSB
  data[2] = SetValue; // 10. Mode=0;
  data[3] = servoID; // 11. ServoID

  pID = servoID ^ playTime;

  lengthString = 4; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  pID = servoID;

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = playTime; // Execution time
  dataEx[8] = data[0];
  dataEx[9] = data[1];
  dataEx[10] = data[2];
  dataEx[11] = data[3];

  sendData(dataEx, pSize);
}

// move one servo to an angle between -160 and 160
void HerkulexClass::moveOneAngle(int servoID, float angle, int pTime, int iLed)
{
  if (angle > 160.0 || angle < -160.0)
    return;
  int position = (int)(angle / 0.325) + 512;
  moveOne(servoID, position, pTime, iLed);
}

// write registry in the RAM: one uint8_t
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID - 253=all servos
  cmd = HRAMWRITE; // 5. CMD
  data[0] = address; // 8. Address
  data[1] = 0x01; // 9. Length
  data[2] = writeByte; // 10. Write error=0

  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Value1
  dataEx[10] = data[3]; // Value2

  sendData(dataEx, pSize);
}

// write registry in the EEP memory (ROM): one uint8_t
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte)
{
  pSize = 0x0A; // 3.Packet size 7-58
  pID = servoID; // 4. Servo ID - 253=all servos
  cmd = HEEPWRITE; // 5. CMD
  data[0] = address; // 8. Address
  data[1] = 0x01; // 9. Length
  data[2] = writeByte; // 10. Write error=0

  lengthString = 3; // lengthData

  ck1 = checksum1(data, lengthString); //6. Checksum1
  ck2 = checksum2(ck1); //7. Checksum2

  dataEx[0] = 0xFF; // Packet Header
  dataEx[1] = 0xFF; // Packet Header
  dataEx[2] = pSize; // Packet Size
  dataEx[3] = pID; // Servo ID
  dataEx[4] = cmd; // Command Ram Write
  dataEx[5] = ck1; // Checksum 1
  dataEx[6] = ck2; // Checksum 2
  dataEx[7] = data[0]; // Address 52
  dataEx[8] = data[1]; // Length
  dataEx[9] = data[2]; // Value1
  dataEx[10] = data[3]; // Value2

  sendData(dataEx, pSize);
}

// Private Methods //////////////////////////////////////////////////////////////

// checksum1
int HerkulexClass::checksum1(uint8_t* data, int lengthString)
{
  XOR = 0;
  XOR = XOR ^ pSize;
  XOR = XOR ^ pID;
  XOR = XOR ^ cmd;
  for (int i = 0; i < lengthString; i++)
  {
    XOR = XOR ^ data[i];
  }
  return XOR & 0xFE;
}

// checksum2
int HerkulexClass::checksum2(int XOR)
{
  return (~XOR) & 0xFE;
}

// add data to variable list servo for syncro execution
void HerkulexClass::addData(int GoalLSB, int GoalMSB, int set, int servoID)
{
  moveData[conta++] = GoalLSB;
  moveData[conta++] = GoalMSB;
  moveData[conta++] = set;
  moveData[conta++] = servoID;
}

// Sending the buffer long length to Serial port
void HerkulexClass::sendData(uint8_t* buffer, int length)
{
  //clear the serial port input buffer - try to do it!
  tcflush(fd, TCIFLUSH);

  write(fd, buffer, length);
}

const clock_t TIMEOUT_CLOCK = TIME_OUT * CLOCKS_PER_SEC;

// * Receiving the length of bytes from Serial port
bool HerkulexClass::readData(int size)
{
  clock_t start = clock();
  int totalReadBytes = 0;
  int totalBytesToRead = size;
  int bytesToRead;
  while (totalReadBytes < size)
  {
    bytesToRead = totalBytesToRead <= DATA_MOVE_ALL ? totalBytesToRead : DATA_MOVE_ALL;
    int readBytes = read(fd, &dataEx[totalReadBytes], bytesToRead);
    totalReadBytes += readBytes;
    totalBytesToRead -= readBytes;

    clock_t now = clock();
    if (now - start > TIMEOUT_CLOCK)
      return false;
  }
  return true;
/*  int i = 0;
  int beginsave = 0;
  int Time_Counter = 0;
  switch (port)
  {
    case SSerial:

      while ((SwSerial.available() < size) & (Time_Counter < TIME_OUT))
      {
        Time_Counter++;
        delayMicroseconds(1000); //wait 1 millisecond for 10 times
      }

      while (SwSerial.available() > 0)
      {
        uint8_t inchar = (uint8_t)SwSerial.read();
        if ((inchar == 0xFF) & ((uint8_t)SwSerial.peek() == 0xFF))
        {
          beginsave = 1;
          i = 0; // if found new header, begin again
        }
        if (beginsave == 1 && i < size)
        {
          dataEx[i] = inchar;
          i++;
        }
      }
      SwSerial.flush();
      break;
  }*/
}

void HerkulexClass::delay(long millis)
{
  usleep(millis * 1000);
}

bool HerkulexClass::setInterfaceAttribs(int fd, int speed, int parity)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
    return false;

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
    return false;
  }
  return true;
}

bool HerkulexClass::setBlocking(int fd, int should_block)
{
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0)
  {
    return false;
  }

  tty.c_cc[VMIN] = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    return false;
  }

  return true;
}

int HerkulexClass::baudRateToBaudRateConst(int baudRate)
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
      return B9600;
  }
}


HerkulexClass Herkulex;
