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

  ros::Time prevStamp_;
  MadgwickImu madgwick_;
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

  prevStamp_ = ros::Time::now();
  madgwick_.center();
}

void TestImuNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  // q_: local to world.
  q_.setValue(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);

  tf2Scalar yaw, pitch;
  MadgwickImu::getEulerYP(q_, yaw, pitch);
  ROS_INFO("Yaw/Pitch: %+9.3f, %+9.3f", yaw, pitch);

  t_.setRotation(q_);
  tf2::Vector3 x = t_ * x_;
  tf2::Vector3 y = t_ * y_;
  tf2::Vector3 z = t_ * z_;
//  ROS_INFO("Vectors x/y/z: %+9.3f, %+9.3f, %+9.3f  /  %+9.3f, %+9.3f, %+9.3f  /  %+9.3f, %+9.3f, %+9.3f",
//           x.x(), x.y(), x.z(), y.x(), y.y(), y.z(), z.x(), z.y(), z.z());

/*
  ros::Duration deltaTime = imu->header.stamp - prevStamp_;
  prevStamp_ = imu->header.stamp;
  madgwick_.update(deltaTime.toSec(),
                   imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z,
                   imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
  tf2Scalar yaw, pitch;
  madgwick_.getEulerYP(yaw, pitch);
//  ROS_INFO("Src Yaw/Pitch: %+9.3f, %+9.3f", yaw, pitch);

  tf2Scalar qx, qy, qz, qw;
  madgwick_.getQuaternion(qx, qy, qz, qw);

  tf2::Quaternion q;
  q.setValue(qx, qy, qz, qw);

  tf2::Matrix3x3 m;
  m.setRotation(q);

//  ROS_INFO("Matrix: colX   %+9.3f, %+9.3f, %+9.3f  /  colY   %+9.3f, %+9.3f, %+9.3f  /  colZ   %+9.3f, %+9.3f, %+9.3f",
//           m.getColumn(0).x(), m.getColumn(0).y(), m.getColumn(0).z(),
//           m.getColumn(1).x(), m.getColumn(1).y(), m.getColumn(1).z(),
//           m.getColumn(2).x(), m.getColumn(2).y(), m.getColumn(2).z());

  tf2::Vector3 x = m.getColumn(0);
  tf2::Vector3 z = z_;
  tf2::Vector3 y = z.cross(x).normalize();
  x = y.cross(z);

  m.setValue(x.x(), y.x(), z.x(),
             x.y(), y.y(), z.y(),
             x.z(), y.z(), z.z());
  m.getRotation(q);
  //q = q.inverse();
  //madgwick_.center();

  tf2Scalar yaw2, pitch2;
  MadgwickImu::getEulerYP(q, yaw2, pitch2);
//  ROS_INFO("Yaw/Pitch: %+9.3f (%+9.3f), %+9.3f (%+9.3f)", yaw, yaw2, pitch, pitch2);

//  ROS_INFO("Matrix2: colX   %+9.3f, %+9.3f, %+9.3f  /  colY   %+9.3f, %+9.3f, %+9.3f  /  colZ   %+9.3f, %+9.3f, %+9.3f",
//           m.getColumn(0).x(), m.getColumn(0).y(), m.getColumn(0).z(),
//           m.getColumn(1).x(), m.getColumn(1).y(), m.getColumn(1).z(),
//           m.getColumn(2).x(), m.getColumn(2).y(), m.getColumn(2).z());
*/
}

void TestImuNode::inputCallback(const std_msgs::StringConstPtr& command)
{
  if (command->data.compare("center") == 0)
  {
    ROS_INFO("Setting head zero orientation...");
    madgwick_.center();
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
