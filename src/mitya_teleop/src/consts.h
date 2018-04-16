/*
 * consts.h
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
 *  Created on: Apr 24, 2017
 *      Author: Dmitry Dzakhov
 */

#ifndef MITYA_TELEOP_SRC_CONSTS_H_
#define MITYA_TELEOP_SRC_CONSTS_H_


#define HEAD_HORIZONTAL_SERVO_ID 1
#define HEAD_VERTICAL_SERVO_ID 2
#define HEAD_BROADCAST_SERVO_ID 0xFE

enum HerkulexTorqueState
{
  HTS_TORQUE_FREE = 0x00, HTS_BREAK_ON = 0x40, HTS_TORQUE_ON = 0x60
};

/**
 * ROS namespace for Robot Mitay's packages.
 * Actually namespace is defined in launch file.
 */
#define RM_NAMESPACE ""

/**
 * Topic name from package joy, joy_node.
 * Run "sudo apt-get install ros-kinetic-joy" to install package.
 */
#define RM_JOY_TOPIC_NAME "joy"

#define RM_ARDUINO_NODE_NAME "arduino_node"
#define RM_ARDUINO_OUTPUT_TOPIC_NAME "arduino_output"
#define RM_ARDUINO_INPUT_TOPIC_NAME "arduino_input"

#define RM_JOYSTICK_NODE_NAME "joystick_node"
#define RM_DRIVE_TOPIC_NAME "drive"

#define RM_LED_TOPIC_NAME "led"
#define RM_DISTANCE_TOPIC_NAME "distance"
#define RM_SPEED_TOPIC_NAME "speed"

#define RM_MPU6050_NODE_NAME "mpu6050_node"
#define RM_HEAD_IMU_OUTPUT_TOPIC_NAME "head_imu_output"
#define RM_HEAD_IMU_INPUT_TOPIC_NAME "head_imu_input"

#define RM_CONTROLLER_IMU_TOPIC_NAME "controller_imu"

#define RM_HERKULEX_NODE_NAME "herkulex_node"
#define RM_HEAD_POSITION_TOPIC_NAME "head_position"
#define RM_HEAD_MOVE_TOPIC_NAME "head_move"
#define RM_HERKULEX_OUTPUT_TOPIC_NAME "herkulex_output"
#define RM_HERKULEX_INPUT_TOPIC_NAME "herkulex_input"
#define RM_DRIVE_TOWARDS_TOPIC_NAME "drive_towards"
#define RM_FACE_TOPIC_NAME "face"


#endif /* MITYA_TELEOP_SRC_CONSTS_H_ */
