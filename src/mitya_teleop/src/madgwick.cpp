/*
 * madgwick.cpp
 * Copyright (c) 2018, Robot Mitya.
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
 *  Created on: Feb 17, 2018
 *      Author: Dmitry Dzakhov
 *    Based on: Implementation of Madgwick's IMU and AHRS algorithms.
 *              See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 */

#include <math.h>
#include "madgwick.h"

//#define betaDef	0.1f		// 2 * proportional gain
#define betaDef 0.075574974f
#define PI 3.141592654f
#define toDeg 57.295779513f

volatile float beta = betaDef; // 2 * proportional gain (Kp)
volatile float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
volatile float roll = 0, pitch = 0, yaw = 0;

float invSqrt(float x);
float getRoll(float x, float y, float z, float w);
float getPitch(float x, float y, float z, float w);
float getYaw(float x, float y, float z, float w);

void MadgwickAHRSupdateIMU(float deltaTime, float gx, float gy, float gz, float ax, float ay, float az)
{
  float recipNorm;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3, _8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {

    // Normalize accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q4 = 2.0f * q4;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _4q3 = 4.0f * q3;
    _8q2 = 8.0f * q2;
    _8q3 = 8.0f * q3;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    q4q4 = q4 * q4;

    // Gradient decent algorithm corrective step
    s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
    s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
    s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
    recipNorm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // Normalize step magnitude
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    s4 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s1;
    qDot2 -= beta * s2;
    qDot3 -= beta * s3;
    qDot4 -= beta * s4;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q1 += qDot1 * deltaTime;
  q2 += qDot2 * deltaTime;
  q3 += qDot3 * deltaTime;
  q4 += qDot4 * deltaTime;

  // Normalize quaternion
  recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  q4 *= recipNorm;

  roll = getRoll(q2, q3, q4, q1);
  pitch = getPitch(q2, q3, q4, q1);
  yaw = getYaw(q2, q3, q4, q1);
}

/**
 * Fast inverse square-root
 * See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float clamp(float value, float min, float max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

/** Get the pole of the gimbal lock, if any.
 * @return positive (+1) for north pole, negative (-1) for south pole, zero (0) when no gimbal lock */
int getGimbalPole(float x, float y, float z, float w)
{
  float t = y * x + z * w;
  return t > 0.499f ? 1 : (t < -0.499f ? -1 : 0);
}

/** Get the roll euler angle in radians, which is the rotation around the z axis. Requires that this quaternion is normalized.
 * @return the rotation around the z axis in radians (between -PI and +PI) */
float getRollRad(float x, float y, float z, float w)
{
  int pole = getGimbalPole(x, y, z, w);
  return pole == 0 ? atan2(2.0f * (w * z + y * x), 1.0f - 2.0f * (x * x + z * z)) : (float)pole * 2.0f * atan2(y, w);
}

/** Get the roll euler angle in degrees, which is the rotation around the z axis. Requires that this quaternion is normalized.
 * @return the rotation around the z axis in degrees (between -180 and +180) */
float getRoll(float x, float y, float z, float w)
{
  return getRollRad(x, y, z, w) * toDeg;
}

/** Get the pitch euler angle in radians, which is the rotation around the x axis. Requires that this quaternion is normalized.
 * @return the rotation around the x axis in radians (between -(PI/2) and +(PI/2)) */
float getPitchRad(float x, float y, float z, float w)
{
  int pole = getGimbalPole(x, y, z, w);
  return pole == 0 ? (float)asin(clamp(2.0f * (w * x - z * y), -1.0f, 1.0f)) : (float)pole * PI * 0.5f;
}

/** Get the pitch euler angle in degrees, which is the rotation around the x axis. Requires that this quaternion is normalized.
 * @return the rotation around the x axis in degrees (between -90 and +90) */
float getPitch(float x, float y, float z, float w)
{
  return getPitchRad(x, y, z, w) * toDeg;
}

/** Get the yaw euler angle in radians, which is the rotation around the y axis. Requires that this quaternion is normalized.
 * @return the rotation around the y axis in radians (between -PI and +PI) */
float getYawRad(float x, float y, float z, float w)
{
  return getGimbalPole(x, y, z, w) == 0 ? atan2(2.0f * (y * w + x * z), 1.0f - 2.0f * (y * y + x * x)) : 0.0f;
}

/** Get the yaw euler angle in degrees, which is the rotation around the y axis. Requires that this quaternion is normalized.
 * @return the rotation around the y axis in degrees (between -180 and +180) */
float getYaw(float x, float y, float z, float w)
{
  return getYawRad(x, y, z, w) * toDeg;
}
