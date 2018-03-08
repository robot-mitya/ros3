/*
 * madgwick.h
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

#ifndef MITYA_TELEOP_SRC_MADGWICK_H_
#define MITYA_TELEOP_SRC_MADGWICK_H_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class MadgwickImu
{
public:
  MadgwickImu();
  void center();
  void update(float deltaTime,
              float gx, float gy, float gz,
              float ax, float ay, float az);
  void getQuaternion(tf2Scalar& x, tf2Scalar& y, tf2Scalar& z, tf2Scalar& w);
  void getEulerYPR(tf2Scalar& yaw, tf2Scalar& pitch, tf2Scalar& roll);
  static void getEulerYPR(tf2::Quaternion & quaternion, tf2Scalar& yaw, tf2Scalar& pitch, tf2Scalar& roll);
private:
  tf2::Quaternion q_;
  float invSqrt(float x);

  static const float BETA = 0.075574974f;
};

#endif
