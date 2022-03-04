/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2019, NMMI
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

/*
 * Modification NMMI 2022:
 * reconstruct program, rename variables.
 */

/**
 * \file      JoyBridge_Class.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>

#define NODEFREQ_JOYBRIDGE ((double)50.0)

class JoyBridge
{
public:
  JoyBridge();
  ~JoyBridge();
  void run();

private:
  bool buf_button1;
  ros::NodeHandle nodeHandler;
  std::string arm_name;
  ros::Subscriber subHandler_index, subHandler_track, subHandler_middle, subHandler_joy_x, subHandler_joy_y, subHandler_button1, subHandler_button2, subHandler_state_joy;
  ros::Publisher pubHandler_tracking, pubHandler_state_joy, pubHandler_indexcmd, pubHandler_joy_x, pubHandler_joy_y, pubHandler_button1;
  std_msgs::Int16 state_joy;
  bool flag_calibrate_init;

  double msg_index[2] = {0, 0}, msg_middle[2] = {0, 0};
  double length_arm_user, length_arm_robot;
  Eigen::Matrix4d Trans_oculusbase2shoulder, Trans_robotbase2shoulder;

  std_msgs::Float64 closure_hand, joy_x_cmd, joy_y_cmd;

  Eigen::Vector3d position_joy_cur, position_joy_buf;
  Eigen::Quaterniond quat_joy_cur, quat_robothand_buf;

  unsigned int cntTimeout = 0;

  void button1Callback(const std_msgs::Bool::ConstPtr &msg);
  void button2Callback(const std_msgs::Bool::ConstPtr &msg);
  void joyXCallback(const std_msgs::Float64::ConstPtr &msg);
  void joyYCallback(const std_msgs::Float64::ConstPtr &msg);

  // INDEX TRIGGER
  void indexCallback(const std_msgs::Float64::ConstPtr &msg);
  // HAND TRACKING
  void trackCallback(const geometry_msgs::Pose::ConstPtr &msg);
  // MIDDLE TRIGGER
  void middleCallback(const std_msgs::Float64::ConstPtr &msg);
  void joyStateCallback(const std_msgs::Int16::ConstPtr &msg);

}; // End of class SubscribeAndPublish