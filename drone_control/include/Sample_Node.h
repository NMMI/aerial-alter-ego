/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2022, NMMI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ----------------------------------------------------------------------------

/**
 * \file      Sample_Node.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#ifndef DRONE_CONTROL_NODE_H
#define DRONE_CONTROL_NODE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>

#include <fstream>
#include <utility> // std::pair

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define NODEFREQ_ARMINVKINE ((double)200.0)

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

std::mutex mutex_attitude, mutex_angularRate, mutex_velocity, mutex_acceleration, mutex_gps_position, mutex_rc, mutex_control_device, mutex_motor_speed;

typedef struct
{
  double thrust = 0;
  double yawRate = 0;
  double pitch = 0;
  double roll = 0;
} ManualCtrlCmd;
// /*!
//  * @brief a bare bone state machine to track the stage of the mission
//  */
// class Mission
// {
// public:
//   // The basic state transition flow is:
//   // 0---> 1 ---> 2 ---> ... ---> N ---> 0
//   // where state 0 means the mission is note started
//   // and each state i is for the process of moving to a target point.
//   int state;

//   int inbound_counter;
//   int outbound_counter;
//   int break_counter;

//   float target_offset_x;
//   float target_offset_y;
//   float target_offset_z;
//   float target_yaw;
//   sensor_msgs::NavSatFix start_gps_location;
//   geometry_msgs::Point start_local_position;

//   bool finished;

//   Mission() : state(0), inbound_counter(0), outbound_counter(0), break_counter(0),
//               target_offset_x(0.0), target_offset_y(0.0), target_offset_z(0.0),
//               finished(false)
//   {
//   }

//   void step();

//   void setTarget(float x, float y, float z, float yaw)
//   {
//     target_offset_x = x;
//     target_offset_y = y;
//     target_offset_z = z;
//     target_yaw      = yaw;
//   }

//   void reset()
//   {
//     inbound_counter = 0;
//     outbound_counter = 0;
//     break_counter = 0;
//     finished = false;
//   }

// };

// void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
//                          sensor_msgs::NavSatFix& target,
//                          sensor_msgs::NavSatFix& origin);

// geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr &msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr &msg);

void thrustCmdCallback(const std_msgs::Float64::ConstPtr &msg);
void yawCmdCallback(const std_msgs::Float64::ConstPtr &msg);
void pitchCmdCallback(const std_msgs::Float64::ConstPtr &msg);
void rollCmdCallback(const std_msgs::Float64::ConstPtr &msg);

void droneMovementCtrl(ManualCtrlCmd cmd);
void drone_arm_callback(const std_msgs::Bool::ConstPtr &msg);
// void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

// void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

// void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

// bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool monitoredTakeoff();

// bool M100monitoredTakeoff();

bool set_local_position();

#endif