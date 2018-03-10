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
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include "consts.h"
#include "madgwick.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

class TestImuNode
{
public:
  TestImuNode();
private:
  ros::Subscriber imuSubscriber_;
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  ros::Subscriber inputSubscriber_;
  void inputCallback(const std_msgs::StringConstPtr& command);

  tf2::Quaternion q_;
  tf2::Transform t_;
  tf2::Vector3 x_;
  tf2::Vector3 y_;
  tf2::Vector3 z_;

  tf2::Quaternion extraQuaternion_;

  static const float PI = 3.14159265358979f;
};

TestImuNode::TestImuNode()
{
  ros::NodeHandle nodeHandle(RM_NAMESPACE);
  imuSubscriber_ = nodeHandle.subscribe<sensor_msgs::Imu>(RM_HEAD_IMU_OUTPUT_TOPIC_NAME, 100, &TestImuNode::imuCallback, this);
  inputSubscriber_ = nodeHandle.subscribe<std_msgs::String>("test_imu_input", 100, &TestImuNode::inputCallback, this);

  x_.setValue(1, 0, 0);
  y_.setValue(0, 1, 0);
  z_.setValue(0, 0, 1);

  tf2::Vector3 z(0, 0, 0);
  t_.setOrigin(z);

  tf2::Vector3 temp(0, 0, 1);
  extraQuaternion_.setRotation(temp, -PI / 2.0f);
}

void TestImuNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  // q_: local to world.
  q_.setValue(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);

  tf2Scalar yaw, pitch, roll;
  q_ *= extraQuaternion_;
  MadgwickImu::getEulerYPR(q_, yaw, pitch, roll);
  yaw += 90;
  if (yaw > 180) yaw -= 360;
  ROS_INFO("Roll/Pitch/Yaw: %+9.3f, %+9.3f, %+9.3f", roll, pitch, yaw);

  t_.setRotation(q_);
  tf2::Vector3 x = t_ * x_;
  tf2::Vector3 y = t_ * y_;
  tf2::Vector3 z = t_ * z_;
  ROS_INFO("Vectors x/y/z: %+9.3f, %+9.3f, %+9.3f  /  %+9.3f, %+9.3f, %+9.3f  /  %+9.3f, %+9.3f, %+9.3f",
           x.x(), x.y(), x.z(), y.x(), y.y(), y.z(), z.x(), z.y(), z.z());
}

void TestImuNode::inputCallback(const std_msgs::StringConstPtr& command)
{
//  if (command->data.compare("center") == 0)
//  {
//    ROS_INFO("Setting head zero orientation...");
//    madgwick_.center();
//  }
//  else
//  {
//    ROS_ERROR("Unknown command \"%s\"", command->data.c_str());
//  }
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
