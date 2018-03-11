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

static tf2::Matrix3x3 m_;
static tf2::Quaternion eulerQuaternion_;
static tf2::Quaternion rotationZMinus90_(tf2::Vector3(0, 0, 1), -MadgwickImu::PI / 2.0f);
static tf2::Vector3 x_(1, 0, 0);
static tf2::Vector3 y_(0, 1, 0);
static tf2::Vector3 z_(0, 0, 1);

MadgwickImu::MadgwickImu()
{
  qSource_.setValue(0, 0, 0, 1);
  qCenter_.setValue(0, 0, 0, 1);
  qResult_ = qSource_ * qCenter_;
}

void MadgwickImu::center()
{
  m_.setRotation(qSource_);
  x_ = m_.getColumn(0);
  y_ = z_.cross(x_).normalize();
  x_ = y_.cross(z_);

  m_.setValue(x_.x(), y_.x(), z_.x(),
              x_.y(), y_.y(), z_.y(),
              x_.z(), y_.z(), z_.z());
  m_.getRotation(qCenter_);
  qCenter_ = qCenter_.inverse();

//  qCenter_ = qSource_.inverse();
}

void MadgwickImu::update(float deltaTime,
            float gx, float gy, float gz,
            float ax, float ay, float az)
{
  float q1 = qSource_.w();
  float q2 = qSource_.x();
  float q3 = qSource_.y();
  float q4 = qSource_.z();
  float norm;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _4q3 = 4.0f * q3;
  float _8q2 = 8.0f * q2;
  float _8q3 = 8.0f * q3;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;
  float q4q4 = q4 * q4;

  // Normalize accelerometer measurement
  norm = (float) sqrt(ax * ax + ay * ay + az * az);
  if (norm < 0.0001f) return; // handle NaN
  norm = 1.0f / norm;         // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Gradient decent algorithm corrective step
  s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
  s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
  s3 = 4.0f * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
  s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
  norm = 1.0f / (float) sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalize step magnitude
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - BETA * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - BETA * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - BETA * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - BETA * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltaTime;
  q2 += qDot2 * deltaTime;
  q3 += qDot3 * deltaTime;
  q4 += qDot4 * deltaTime;
  norm = 1.0f / (float) sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalize quaternion
  q1 *= norm;
  q2 *= norm;
  q3 *= norm;
  q4 *= norm;
  qSource_.setValue(q2, q3, q4, q1);
  qResult_ = qSource_ * qCenter_;
}

void MadgwickImu::getQuaternion(tf2Scalar& x, tf2Scalar& y, tf2Scalar& z, tf2Scalar& w)
{
  x = qResult_.x();
  y = qResult_.y();
  z = qResult_.z();
  w = qResult_.w();
}

void MadgwickImu::getEulerYP(tf2::Quaternion & quaternion, tf2Scalar& yaw, tf2Scalar& pitch)
{
  eulerQuaternion_ = quaternion * rotationZMinus90_;
  m_.setRotation(eulerQuaternion_);
  tf2Scalar temp;
  m_.getEulerYPR(yaw, temp, pitch, 1);
  yaw *= TO_DEG;
  yaw += 90;
  if (yaw > 180) yaw -= 360;
  pitch *= TO_DEG;
}

void MadgwickImu::getEulerYP(tf2Scalar& yaw, tf2Scalar& pitch)
{
  MadgwickImu::getEulerYP(qResult_, yaw, pitch);
}

/**
 * Fast inverse square-root
 * See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
 */
float MadgwickImu::invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
//  return 1.0f / sqrt(x);
}
