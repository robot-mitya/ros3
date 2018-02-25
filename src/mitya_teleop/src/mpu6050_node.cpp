/*
 * mpu6050_node.cpp
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
 *  Created on: Dec 3, 2017
 *      Author: Dmitry Dzakhov
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "consts.h"
#include "mpu6050_helper.h"

#define VALUES_TO_CALIBRATE 1000

class Mpu6050Node
{
public:
  Mpu6050Node();
  void readImuData(float *vX, float *vY, float *vZ, float *aX, float *aY, float *aZ);
  bool performingCalibration(float vX, float vY, float vZ);

  void fillImuMessage(const sensor_msgs::ImuPtr& msg, float vX, float vY, float vZ, float aX, float aY, float aZ);
  void publishImuMessage(const sensor_msgs::ImuPtr& msg);
private:
  int i2cAddress_;
  int fileDescriptor_;
  uint8_t buffer_[14];
  float readWord2c(int addr);

  // Topic RM_IMU_TOPIC_NAME ('imu') publisher:
  ros::Publisher imuPublisher_;

  ros::Subscriber imuInputSubscriber_;
  void imuInputCallback(const std_msgs::StringConstPtr& msg);

  MpuHelper mpuHelper_;

  uint32_t seq_;
};

const int PWR_MGMT_1 = 0x6B;

Mpu6050Node::Mpu6050Node()
{
  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("i2c_address", i2cAddress_, 0x68);
//  privateNodeHandle.

  // Connect to device.
  fileDescriptor_ = wiringPiI2CSetup(i2cAddress_);
  if (fileDescriptor_ == -1)
  {
    ROS_ERROR("No i2c device found?");
    return;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fileDescriptor_, PWR_MGMT_1, 0);

  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  imuPublisher_ = nodeHandle.advertise<sensor_msgs::Imu>(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100);

  imuInputSubscriber_ = nodeHandle.subscribe(RM_HEAD_IMU_INPUT_TOPIC_NAME, 10, &Mpu6050Node::imuInputCallback, this);

  seq_ = 0;
}

float Mpu6050Node::readWord2c(int addr)
{
  uint16_t val = wiringPiI2CReadReg16(fileDescriptor_, addr);
  uint16_t swappedVal = (val >> 8) | (val << 8);
  return float((swappedVal >= 0x8000) ? -((65535 - swappedVal) + 1) : swappedVal);
}

void Mpu6050Node::readImuData(float *vX, float *vY, float *vZ, float *aX, float *aY, float *aZ)
{
  // Read gyroscope values.
  // At default sensitivity of 250deg/s we need to scale by 131.
  *vX = -readWord2c(0x47) / 131; // -z (caused by chip orientation in robot)
  *vY = readWord2c(0x45) / 131;  // y
  *vZ = readWord2c(0x43) / 131;  // x (caused by chip orientation in robot)

  // Read accelerometer values.
  // At default sensitivity of 2g we need to scale by 16384.
  // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
  // But! Imu message documentations say acceleration should be in m/2 so need to *9.807
  const float la_rescale = 16384.0 / 9.807;
  *aX = -readWord2c(0x3f) / la_rescale; // -z (caused by chip orientation in robot)
  *aY = readWord2c(0x3d) / la_rescale; // y
  *aZ = readWord2c(0x3b) / la_rescale; // x (caused by chip orientation in robot)

  mpuHelper_.correctMpuData(vX, vY, vZ);
}

void Mpu6050Node::fillImuMessage(const sensor_msgs::ImuPtr& msg, float vX, float vY, float vZ, float aX, float aY, float aZ)
{
  msg->header.stamp = ros::Time::now();
  msg->header.seq = seq_++;
  msg->header.frame_id = '0';  // no frame

  msg->angular_velocity.x = vX;
  msg->angular_velocity.y = vY;
  msg->angular_velocity.z = vZ;

  msg->linear_acceleration.x = aX;
  msg->linear_acceleration.y = aY;
  msg->linear_acceleration.z = aZ;
}

void Mpu6050Node::publishImuMessage(const sensor_msgs::ImuPtr& msg)
{
  imuPublisher_.publish(msg);
}

void Mpu6050Node::imuInputCallback(const std_msgs::StringConstPtr& msg)
{
  if (msg->data.compare("calibrate") == 0)
  {
    ROS_INFO("Starting to calibrate head IMU...");
    mpuHelper_.startCalibration();
  }
  else
  {
    ROS_ERROR("%s.%s: Unknown command \"%s\"", RM_MPU6050_NODE_NAME, RM_HEAD_IMU_INPUT_TOPIC_NAME, msg->data.c_str());
  }
}

bool Mpu6050Node::performingCalibration(float vX, float vY, float vZ)
{
  return mpuHelper_.processCalibration(vX, vY, vZ);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_MPU6050_NODE_NAME);

  Mpu6050Node mpu6050Node;
  sensor_msgs::ImuPtr imuPrt(new sensor_msgs::Imu());

  float vX, vY, vZ, aX, aY, aZ;
  ros::Rate rate(50);  // hz
  while(ros::ok())
  {
    mpu6050Node.readImuData(&vX, &vY, &vZ, &aX, &aY, &aZ);

    if (!mpu6050Node.performingCalibration(vX, vY, vZ))
    {
      mpu6050Node.fillImuMessage(imuPrt, vX, vY, vZ, aX, aY, aZ);
      mpu6050Node.publishImuMessage(imuPrt);
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
