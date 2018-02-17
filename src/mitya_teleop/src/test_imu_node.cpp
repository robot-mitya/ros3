/*
 * test_imu_node.cpp
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
#include <ros/platform.h>
#include <sensor_msgs/Imu.h>
#include "consts.h"
#include "mpu6050_helper.h"
#include "madgwick.h"

class TestImuNode
{
public:
  TestImuNode();
private:
  ros::Subscriber imuSubscriber_;
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  ros::Time prevStamp_;
};

TestImuNode::TestImuNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  imuSubscriber_ = nodeHandle.subscribe<sensor_msgs::Imu>(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, &TestImuNode::imuCallback, this);

  prevStamp_ = ros::Time::now();
}

void TestImuNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  ros::Duration deltaTime = imu->header.stamp - prevStamp_;
  prevStamp_ = imu->header.stamp;

//  float vx = imu->angular_velocity.x * 131;
//  float vy = imu->angular_velocity.y * 131;
//  float vz = imu->angular_velocity.z * 131;
//  float ax = imu->linear_acceleration.x / 16384.0 * 9.807;
//  float ay = imu->linear_acceleration.y / 16384.0 * 9.807;
//  float az = imu->linear_acceleration.z / 16384.0 * 9.807;
  float vx = imu->angular_velocity.x;
  float vy = imu->angular_velocity.y;
  float vz = imu->angular_velocity.z;
  float ax = imu->linear_acceleration.x;
  float ay = imu->linear_acceleration.y;
  float az = imu->linear_acceleration.z;

  ROS_INFO("Time: %.3f; Angular velocity: %.3f, %.3f, %.3f; Acceleration: %.3f, %.3f, %.3f", deltaTime.toSec(), vx, vy, vz, ax, ay, az);

  float dt = deltaTime.toSec();
//  MadgwickAHRSupdateIMU(dt, vx, vy, vz, ax, ay, az);
  MadgwickAHRSupdateIMU(dt, -vz, vy, vx, -az, ay, ax);

  ROS_INFO("Quaternion: %.3f, %.3f, %.3f, %.3f", q0, q1, q2, q3);
  float toDeg = 180.0 / 3.1416;
  ROS_INFO("Roll/Pitch/Yaw: %.3f, %.3f, %.3f", roll * toDeg, pitch * toDeg, yaw * toDeg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_imu_node");

  TestImuNode testImuNode;

  ros::Rate loop_rate(100); // 100 Hz
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
