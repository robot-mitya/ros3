/*
 * mpu6050_helper.cpp
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

#include "mpu6050_helper.h"
#include <algorithm>

MpuHelper::MpuHelper()
{
  arrayIndex_ = 0;
  calibrating_ = false;

  angularVelocitiesX_ = new float[valuesCount];
  angularVelocitiesY_ = new float[valuesCount];
  angularVelocitiesZ_ = new float[valuesCount];
  accelerationsX_ = new float[valuesCount];
  accelerationsY_ = new float[valuesCount];
  accelerationsZ_ = new float[valuesCount];

  deltaAngularVelocityX_ = 0;
  deltaAngularVelocityY_ = 0;
  deltaAngularVelocityZ_ = 0;
  deltaAccelerationX_ = 0;
  deltaAccelerationY_ = 0;
  deltaAccelerationZ_ = 0;
  loadCalibrationParamsFromFile();
}

void MpuHelper::correctMpuData(float *vX, float *vY, float *vZ, float *aX, float *aY, float *aZ)
{
  *vX -= deltaAngularVelocityX_;
  *vY -= deltaAngularVelocityY_;
  *vZ -= deltaAngularVelocityZ_;
  *aX -= deltaAccelerationX_;
  *aY -= deltaAccelerationY_;
  *aZ -= deltaAccelerationZ_;
}

void MpuHelper::startCalibration()
{
  arrayIndex_ = 0;
  calibrating_ = true;
  deltaAngularVelocityX_ = 0;
  deltaAngularVelocityY_ = 0;
  deltaAngularVelocityZ_ = 0;
  deltaAccelerationX_ = 0;
  deltaAccelerationY_ = 0;
  deltaAccelerationZ_ = 0;
}

bool MpuHelper::processCalibration(float vX, float vY, float vZ, float aX, float aY, float aZ)
{
  if (!calibrating_) return false;

  angularVelocitiesX_[arrayIndex_] = vX;
  angularVelocitiesY_[arrayIndex_] = vY;
  angularVelocitiesZ_[arrayIndex_] = vZ;
  accelerationsX_[arrayIndex_] = aX;
  accelerationsY_[arrayIndex_] = aY;
  accelerationsZ_[arrayIndex_] = aZ;

  arrayIndex_++;

  if (arrayIndex_ == valuesCount)
  {
    deltaAngularVelocityX_ = calculateMedian(angularVelocitiesX_);
    deltaAngularVelocityY_ = calculateMedian(angularVelocitiesY_);
    deltaAngularVelocityZ_ = calculateMedian(angularVelocitiesZ_);
    deltaAccelerationX_ = calculateMedian(accelerationsX_);
    deltaAccelerationY_ = calculateMedian(accelerationsY_);
    deltaAccelerationZ_ = calculateMedian(accelerationsZ_);
    calibrating_ = false;
    saveCalibrationParamsToFile();
  }
  return true;
}

float MpuHelper::calculateMedian(float values[])
{
  std::sort(values, values + valuesCount);
  return values[medianIndex];
}

void MpuHelper::loadCalibrationParamsFromFile()
{

}

void MpuHelper::saveCalibrationParamsToFile()
{

}
