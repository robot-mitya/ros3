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
#include <std_msgs/String.h>
#include "consts.h"
#include "mpu6050_helper.h"
#include "madgwick.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

class TestImuNode
{
public:
  TestImuNode();
private:
  ros::Subscriber imuSubscriber_;
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  ros::Subscriber inputSubscriber_;
  void inputCallback(const std_msgs::StringConstPtr& command);

  ros::Time prevStamp_;
  tf2::Quaternion q_;
  tf2::Quaternion qSrc_;
  tf2::Quaternion qZero_;
  tf2::Vector3 x_;
  tf2::Vector3 y_;
  tf2::Vector3 z_;
  tf2::Vector3 zero3_;
  tf2::Transform t_;

  tf2::Vector3 acc_;
  tf2::Vector3 vel_;
};

TestImuNode::TestImuNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  imuSubscriber_ = nodeHandle.subscribe<sensor_msgs::Imu>(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, &TestImuNode::imuCallback, this);
  inputSubscriber_ = nodeHandle.subscribe<std_msgs::String>("test_imu_input", 100, &TestImuNode::inputCallback, this);

  prevStamp_ = ros::Time::now();
  q_ = tf2::Quaternion::getIdentity();
  qSrc_ = tf2::Quaternion::getIdentity();
  qZero_ = tf2::Quaternion::getIdentity();

  x_.setValue(1, 0, 0);
  y_.setValue(0, 1, 0);
  z_.setValue(0, 0, 1);
  zero3_.setValue(0, 0, 0);
  t_.setRotation(qZero_);
  t_.setOrigin(zero3_);
}

void TestImuNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  ros::Duration deltaTime = imu->header.stamp - prevStamp_;
  prevStamp_ = imu->header.stamp;

  float vx = imu->angular_velocity.x;
  float vy = imu->angular_velocity.y;
  float vz = imu->angular_velocity.z;
  float ax = imu->linear_acceleration.x;
  float ay = imu->linear_acceleration.y;
  float az = imu->linear_acceleration.z;
  tf2::Vector3 vel(vx, vy, vz);
  tf2::Vector3 acc(ax, ay, az);

//  ROS_INFO("Time: %.3f; Angular vel: %.3f, %.3f, %.3f; Linear acc: %.3f, %.3f, %.3f", deltaTime.toSec(),
//           vel.m_floats[0], vel.m_floats[1], vel.m_floats[2],
//           acc.m_floats[0], acc.m_floats[1], acc.m_floats[2]);

  float dt = deltaTime.toSec();
  MadgwickAHRSupdateIMU(dt,
                        vel.m_floats[0], vel.m_floats[1], vel.m_floats[2],
                        acc.m_floats[0], acc.m_floats[1], acc.m_floats[2],
                        &qSrc_);
  q_ = qSrc_ * qZero_;
//  ROS_INFO("Quaternion: %.3f, %.3f, %.3f, %.3f", q_.w(), q_.x(), q_.y(), q_.z());
  ROS_INFO("Quaternion: %.3f, %.3f, %.3f, %.3f", qSrc_.w(), qSrc_.x(), qSrc_.y(), qSrc_.z());

  float roll, pitch, yaw;
//  getEulerAngles(q_.w(), q_.x(), q_.y(), q_.z(), &roll, &pitch, &yaw);
  getEulerAngles(qSrc_.w(), qSrc_.x(), qSrc_.y(), qSrc_.z(), &roll, &pitch, &yaw);
  ROS_INFO("Roll/Pitch/Yaw: %.3f, %.3f, %.3f", roll, pitch, yaw);

  t_.setRotation(q_);
  tf2::Vector3 y = t_ * y_;
  ROS_INFO("Vector y: %.3f, %.3f, %.3f", y.x(), y.y(), y.z());

//  vel_ = vel_.lerp(vel, 0.02f);
//  ROS_INFO("Velocity: %.3f, %.3f, %.3f", vel_.x(), vel_.y(), vel_.z());
//  acc_ = acc_.lerp(acc, 0.02f);
//  ROS_INFO("Acceleration: %.3f, %.3f, %.3f", acc_.x(), acc_.y(), acc_.z());
}

void TestImuNode::inputCallback(const std_msgs::StringConstPtr& command)
{
  if (command->data.compare("zero") == 0)
  {
    ROS_INFO("Setting head zero orientation...");
    qZero_ = qSrc_.inverse();
  }
  else
  {
    ROS_ERROR("Unknown command \"%s\"", command->data.c_str());
  }
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
