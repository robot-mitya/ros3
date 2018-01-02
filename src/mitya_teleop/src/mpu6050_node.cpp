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
#include "consts.h"

//#define MPU6050_INCLUDE_DMP_MOTIONAPPS20

#include "MPU6050_6Axis_MotionApps20.h"

class Mpu6050Node
{
public:
  Mpu6050Node();

  void prepareImuMessage(const sensor_msgs::ImuPtr& msg);
  void publishImuMessage(const sensor_msgs::ImuPtr& msg);

  void setup();
  void loop(const sensor_msgs::ImuPtr& msg);
private:
  MPU6050 *mpu_;

  int update_rate_;
  int i2cAddress_;

  // Topic RM_IMU_TOPIC_NAME ('imu') publisher:
  ros::Publisher imuPublisher_;

  // MPU control/status variants
  bool dmpReady_;  // set true if DMP initialization was successful
  uint8_t mpuIntStatus_;   // holds actual interrupt status byte from MPU
  uint8_t devStatus_;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize_;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount_;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer_[64]; // FIFO storage buffer

  // orientation/motion variants
  Quaternion q_;           // [w, x, y, z]         quaternion container
  VectorInt16 aa_;         // [x, y, z]            accelerator sensor measurements
  VectorInt16 aaReal_;     // [x, y, z]            gravity-free accelerator sensor measurements
  VectorInt16 aaWorld_;    // [x, y, z]            world-frame accelerator sensor measurements
  VectorFloat gravity_;    // [x, y, z]            gravity vector
  float ypr_[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  float last_ypr_[3];      // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  ros::Publisher imu_pub_;
  boost::array<double, 9> linear_acceleration_cov_;
  boost::array<double, 9> angular_velocity_cov_;
  boost::array<double, 9> orientation_cov_;
  boost::array<double, 9> unk_orientation_cov_;
  boost::array<double, 9> magnetic_cov_;
  ros::Time last_update_;

  void setupCovariance(boost::array<double, 9> &cov, double stdev);
};

Mpu6050Node::Mpu6050Node()
{
  dmpReady_ = false;

  ros::NodeHandle privateNodeHandle("~");
  int i2cAddress = 0x68;
  privateNodeHandle.param("i2c_address", i2cAddress_, 0x68);

  double linear_stdev, angular_stdev, orientation_stdev;
  privateNodeHandle.param("linear_acceleration_stdev", linear_stdev, 0.0003);
  privateNodeHandle.param("angular_velocity_stdev", angular_stdev, 0.02 * (M_PI / 180.0));
  privateNodeHandle.param("orientation_stdev", orientation_stdev, 1.0);
  privateNodeHandle.param("update_rate", update_rate_, 100);

  setupCovariance(linear_acceleration_cov_, linear_stdev);
  setupCovariance(angular_velocity_cov_, angular_stdev);
  setupCovariance(orientation_cov_, orientation_stdev);

  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  imuPublisher_ = nodeHandle.advertise<sensor_msgs::Imu>(RM_IMU_TOPIC_NAME, 100);
}

void Mpu6050Node::setupCovariance(boost::array<double, 9> &cov, double stdev)
{
    std::fill(cov.begin(), cov.end(), 0.0);
    if (stdev == 0.0)
      cov[0] = -1.0;
    else
      cov[0 + 0] = cov[3 + 1] = cov[6 + 2] = std::pow(stdev, 2);
}

void Mpu6050Node::prepareImuMessage(const sensor_msgs::ImuPtr& msg)
{
  msg->header.frame_id = "imu";
  msg->orientation_covariance = orientation_cov_;
  msg->angular_velocity_covariance = angular_velocity_cov_;
  msg->linear_acceleration_covariance = linear_acceleration_cov_;
}

void Mpu6050Node::publishImuMessage(const sensor_msgs::ImuPtr& msg)
{
  imuPublisher_.publish(msg);
}

void Mpu6050Node::setup()
{
  mpu_ = new MPU6050(i2cAddress_);

  // initialize device
  ROS_INFO("Initializing I2C devices...\n");
  mpu_->initialize();

  // verify connection
  ROS_INFO("Testing device connections...\n");
  ROS_INFO(mpu_->testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

  // load and configure the DMP
  ROS_INFO("Initializing DMP...\n");
  devStatus_ = mpu_->dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus_ == 0)
  {
    // turn on the DMP, now that it's ready
    ROS_INFO("Enabling DMP...\n");
    mpu_->setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus_ = mpu_->getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    ROS_INFO("DMP ready!\n");
    dmpReady_ = true;

    // get expected DMP packet size for later comparison
    packetSize_ = mpu_->dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    ROS_ERROR_STREAM("DMP Initialization failed (code " << devStatus_ <<")\n");
  }
}

void Mpu6050Node::loop(const sensor_msgs::ImuPtr& msg)
{
  // if programming failed, don't try to do anything
  if (!dmpReady_) return;
  // get current FIFO count
  fifoCount_ = mpu_->getFIFOCount();

  if (fifoCount_ == 1024)
  {
    // reset so we can continue cleanly
    mpu_->resetFIFO();
    ROS_WARN("FIFO overflow!\n");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (fifoCount_ >= 42)
  {
    // read a packet from FIFO
    mpu_->getFIFOBytes(fifoBuffer_, packetSize_);

    ros::Time now = ros::Time::now();
    msg->header.stamp = now;
    ros::Duration dt_r = now - last_update_;
    last_update_ = now;

    // display quaternion values in easy matrix form: w x y z
    mpu_->dmpGetQuaternion(&q_, fifoBuffer_);
    msg->orientation.x = q_.x;
    msg->orientation.y = q_.y;
    msg->orientation.z = q_.z;
    msg->orientation.w = q_.w;

    mpu_->dmpGetGravity(&gravity_, &q_);
    mpu_->dmpGetYawPitchRoll(ypr_, &q_, &gravity_);
    float yaw_ang_vel = ypr_[0] - last_ypr_[0] / dt_r.toSec();
    float pitch_ang_vel = ypr_[1] - last_ypr_[1] / dt_r.toSec();
    float roll_ang_vel = ypr_[2] - last_ypr_[2] / dt_r.toSec();

    msg->angular_velocity.x = roll_ang_vel;
    msg->angular_velocity.y = pitch_ang_vel;
    msg->angular_velocity.z = yaw_ang_vel;

    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu_->dmpGetAccel(&aa_, fifoBuffer_);
    mpu_->dmpGetLinearAccelInWorld(&aaWorld_, &aaReal_, &q_);
    //printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
    msg->linear_acceleration.x = aaWorld_.x;
    msg->linear_acceleration.y = aaWorld_.y;
    msg->linear_acceleration.z = aaWorld_.z;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_MPU6050_NODE_NAME);

  Mpu6050Node mpu6050Node;

  sensor_msgs::ImuPtr imuPrt(new sensor_msgs::Imu());
  mpu6050Node.prepareImuMessage(imuPrt);

  mpu6050Node.setup();

  ros::Rate rate(100);  // Hz
  while(ros::ok())
  {
    mpu6050Node.loop(imuPrt);
    mpu6050Node.publishImuMessage(imuPrt);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

/*
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "consts.h"

class Mpu6050Node
{
public:
  Mpu6050Node();
  void fillImuMessage(const sensor_msgs::ImuPtr& msg);
  void publishImuMessage(const sensor_msgs::ImuPtr& msg);
private:
  int i2cAddress_;
  int fileDescriptor_;
  float readWord2c(int addr);

  // Topic RM_IMU_TOPIC_NAME ('imu') publisher:
  ros::Publisher imuPublisher_;
};

const int PWR_MGMT_1 = 0x6B;

Mpu6050Node::Mpu6050Node()
{
  ros::NodeHandle privateNodeHandle("~");
  privateNodeHandle.param("i2c_address", i2cAddress_, 0x68);

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
  imuPublisher_ = nodeHandle.advertise<sensor_msgs::Imu>(RM_IMU_TOPIC_NAME, 100);
}

float Mpu6050Node::readWord2c(int addr)
{
  int high = wiringPiI2CReadReg8(fileDescriptor_, addr);
  int low = wiringPiI2CReadReg8(fileDescriptor_, addr + 1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

void Mpu6050Node::fillImuMessage(const sensor_msgs::ImuPtr& msg)
{
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = '0';  // no frame

  // Read gyroscope values.
  // At default sensitivity of 250deg/s we need to scale by 131.
  msg->angular_velocity.x = readWord2c(0x43) / 131;
  msg->angular_velocity.y = readWord2c(0x45) / 131;
  msg->angular_velocity.z = readWord2c(0x47) / 131;

  // Read accelerometer values.
  // At default sensitivity of 2g we need to scale by 16384.
  // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
  // But! Imu message documentations say acceleration should be in m/2 so need to *9.807
  const float la_rescale = 16384.0 / 9.807;
  msg->linear_acceleration.x = readWord2c(0x3b) / la_rescale;
  msg->linear_acceleration.y = readWord2c(0x3d) / la_rescale;
  msg->linear_acceleration.z = readWord2c(0x3f) / la_rescale;
}

void Mpu6050Node::publishImuMessage(const sensor_msgs::ImuPtr& msg)
{
  imuPublisher_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, RM_MPU6050_NODE_NAME);

  Mpu6050Node mpu6050Node;
  sensor_msgs::ImuPtr imuPrt(new sensor_msgs::Imu());

  ros::Rate rate(10);  // hz
  while(ros::ok())
  {
    mpu6050Node.fillImuMessage(imuPrt);
    mpu6050Node.publishImuMessage(imuPrt);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
*/
