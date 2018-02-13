/*
 * mpu6050_helper.h
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
 *  Created on: Feb 11, 2018
 *      Author: Dmitry Dzakhov
 */

#ifndef MITYA_TELEOP_SRC_MPU6050_HELPER_H_
#define MITYA_TELEOP_SRC_MPU6050_HELPER_H_

class MpuHelper
{
public:
  MpuHelper();
  void startCalibration();
  bool processCalibration(float vX, float vY, float vZ, float aX, float aY, float aZ);
  void correctMpuData(float *vX, float *vY, float *vZ, float *aX, float *aY, float *aZ);
private:
  static const int medianIndex = 500;
  static const int valuesCount = medianIndex * 2 + 1;

  float *angularVelocitiesX_;
  float *angularVelocitiesY_;
  float *angularVelocitiesZ_;
  float *accelerationsX_;
  float *accelerationsY_;
  float *accelerationsZ_;

  int arrayIndex_;
  bool calibrating_;

  float deltaAngularVelocityX_;
  float deltaAngularVelocityY_;
  float deltaAngularVelocityZ_;
  float deltaAccelerationX_;
  float deltaAccelerationY_;
  float deltaAccelerationZ_;

  float calculateMedian(float values[]);

  void loadCalibrationParamsFromFile();
  void saveCalibrationParamsToFile();
};

#endif /* MITYA_TELEOP_SRC_MPU6050_HELPER_H_ */