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
  uint32_t prevSeq_;
  tf2::Quaternion qSensor_;
  tf2::Vector3 x_;
  tf2::Vector3 y_;
  tf2::Vector3 z_;
  tf2::Vector3 zero3_;
  tf2::Transform t_;

  tf2::Quaternion qTest_;
};

TestImuNode::TestImuNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  imuSubscriber_ = nodeHandle.subscribe<sensor_msgs::Imu>(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, &TestImuNode::imuCallback, this);
  inputSubscriber_ = nodeHandle.subscribe<std_msgs::String>("test_imu_input", 100, &TestImuNode::inputCallback, this);

  prevStamp_ = ros::Time::now();
  prevSeq_ = 0;
  qSensor_.setValue(0, 0, 0, 1);
  qTest_.setValue(0, 0, 0, 1);

  x_.setValue(1, 0, 0);
  y_.setValue(0, 1, 0);
  z_.setValue(0, 0, 1);
  zero3_.setValue(0, 0, 0);
  t_.setRotation(tf2::Quaternion::getIdentity());
  t_.setOrigin(zero3_);
}

void TestImuNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  ros::Duration deltaTime = imu->header.stamp - prevStamp_;
  prevStamp_ = imu->header.stamp;

  if (imu->header.seq - prevSeq_ != 1)
  {
    ROS_INFO("seq=%d  prevSeq=%d", imu->header.seq, prevSeq_);
  }
  else
  {
    ROS_INFO("seq ok: %d", imu->header.seq);
  }
  prevSeq_ = imu->header.seq;

  ROS_INFO("Time: %.3f; Velocities: %.3f, %.3f, %.3f; Accelerations: %.3f, %.3f, %.3f",
           deltaTime.toSec(),
           imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z,
           imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);

  float dt = deltaTime.toSec();
  madgwickAHRSupdateIMU(dt,
                        imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z,
                        imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z,
                        qSensor_);
  ROS_INFO("Quaternion: %.3f, %.3f, %.3f, %.3f", qSensor_.w(), qSensor_.x(), qSensor_.y(), qSensor_.z());

  float roll, pitch, yaw;
  int pole;
  getEulerAngles(qSensor_.w(), qSensor_.x(), qSensor_.y(), qSensor_.z(), &roll, &pitch, &yaw, &pole);
  ROS_INFO("Roll/Pitch/Yaw/Pole: %.3f, %.3f, %.3f %s", roll, pitch, yaw, pole == 0 ? "" : "+++++++++++++");

  t_.setRotation(qSensor_);
  tf2::Vector3 x = t_ * x_;
  tf2::Vector3 y = t_ * y_;
  tf2::Vector3 z = t_ * z_;
  int branch;
  float result2;
  float ya = getYaw(x, y, z, &branch, &result2);
  ROS_INFO("Vector x: %.3f, %.3f, %.3f    Vector y: %.3f, %.3f, %.3f    Vector z: %.3f, %.3f, %.3f    Yaw: %.3f / %.3f (%d)",
           x.x(), x.y(), x.z(), y.x(), y.y(), y.z(), z.x(), z.y(), z.z(), ya, result2, branch);
}

void TestImuNode::inputCallback(const std_msgs::StringConstPtr& command)
{
  if (command->data.compare("center") == 0)
  {
    ROS_INFO("Setting head zero orientation...");
    qSensor_.setValue(0, 0, 0, 1);
  }
  else if (command->data.compare("test") == 0)
  {
    ROS_INFO("Test: x=%.3f  y=%.3f  z=%.3f  w=%.3f", qTest_.x(), qTest_.y(), qTest_.z(), qTest_.w());
    madgwickAHRSupdateIMU(0.02f, 5.0f, 3.0f, 1.0f, 0.1f, 0.2f, 0.8f, qTest_);
    ROS_INFO("Test: x=%.3f  y=%.3f  z=%.3f  w=%.3f", qTest_.x(), qTest_.y(), qTest_.z(), qTest_.w());
    madgwickAHRSupdateIMU(0.02f, 5.0f, 3.0f, 1.0f, 0.1f, 0.2f, 0.8f, qTest_);
    ROS_INFO("Test: x=%.3f  y=%.3f  z=%.3f  w=%.3f", qTest_.x(), qTest_.y(), qTest_.z(), qTest_.w());
    madgwickAHRSupdateIMU(0.02f, 5.0f, 3.0f, 1.0f, 0.1f, 0.2f, 0.8f, qTest_);
    ROS_INFO("Test: x=%.3f  y=%.3f  z=%.3f  w=%.3f", qTest_.x(), qTest_.y(), qTest_.z(), qTest_.w());

    qTest_.setValue(0, 0, 0, 1);
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
