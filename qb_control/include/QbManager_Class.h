/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2019, NMMI
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
 * \file      QbManager_Class.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include "qbmove_communications.h"
#include "imuboard_communications.h"

#define NODEFREQ_ARMINVKINE ((double)200.0)

#define HAND_CLOSURE ((short int)19000)
#define CUBE_ID      \
  {                  \
    2, 3, 4, 6, 7, 8 \
  }
#define HAND_ID \
  {             \
    1, 5        \
  }
#define NECK_ID \
  {             \
    9, 10       \
  }
#define CUBE_ACTIVATE true
class QbManager
{
public:
  QbManager();

  int initialize();
  void run();
  void terminate();
  double dt;

private:
  ros::NodeHandle nodeHandler;
  ros::Subscriber subHandler_ref_neck, subHandler_posi_motor_rightArm, subHandler_posi_motor_leftArm, subHandler_closure_hand_rightArm, subHandler_closure_hand_leftArm;

  comm_settings comm_settings_t;

  std::map<int, std::vector<short int>> cmd_cube;
  std::vector<int> cube_index;
  double preset;
  void activate(comm_settings *cs, int id);
  void deactivate(comm_settings *cs, int id);

  void handClosureCallback_rightArm(const std_msgs::Float64 &msg);
  void handClosureCallback_leftArm(const std_msgs::Float64 &msg);
  void motorPosiCallback_rightArm(const sensor_msgs::JointState &msg);
  void motorPosiCallback_leftArm(const sensor_msgs::JointState &msg);
  void neckPosiCallback(const sensor_msgs::JointState &msg);
  ;
  double rad2tick(double cmd);
  inline void compute_hand_motor_position(double d, short int &tick);
};
