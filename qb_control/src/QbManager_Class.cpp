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
 * \file      QbManager_Class.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "QbManager_Class.h"
QbManager::QbManager()
{

  cube_index = CUBE_ID;
  std::vector<int> hand_id = HAND_ID;
  std::vector<int> neck_id = NECK_ID;
  cube_index.insert(cube_index.end(), hand_id.begin(), hand_id.end());
  cube_index.insert(cube_index.end(), neck_id.begin(), neck_id.end());

  subHandler_posi_motor_rightArm = nodeHandler.subscribe("/right_arm/motor_position", 1, &QbManager::motorPosiCallback_rightArm, this);
  subHandler_posi_motor_leftArm = nodeHandler.subscribe("/left_arm/motor_position", 1, &QbManager::motorPosiCallback_leftArm, this);
  subHandler_closure_hand_rightArm = nodeHandler.subscribe("/right_arm/hand_closure", 1, &QbManager::handClosureCallback_rightArm, this);
  subHandler_closure_hand_leftArm = nodeHandler.subscribe("/left_arm/hand_closure", 1, &QbManager::handClosureCallback_leftArm, this);
  subHandler_ref_neck = nodeHandler.subscribe("/ref_neck", 1, &QbManager::neckPosiCallback, this);
}

int QbManager::initialize()
{
  openRS485(&comm_settings_t, "/dev/ttyUSB0");
  usleep(500000);

  std::cout << std::endl;
  std::cout << "QbManager" << std::endl;
  std::cout << " - Cubes (" << cube_index.size() << ") = " << std::endl;
  for (auto id : cube_index)
  {
    cmd_cube[id] = {0, 0};
    if (CUBE_ACTIVATE)
      activate(&comm_settings_t, id);
    std::cout << id << ", ";
  }
  ROS_INFO("Cube Activated Successful!");
  return 0;
}

void QbManager::run()
{
  for (auto id : cube_index)
  {
    short int inputs[2];
    inputs[0] = cmd_cube[id].at(0);
    inputs[1] = cmd_cube[id].at(1);
    commSetInputs(&comm_settings_t, id, inputs);
    usleep(500);
  }
}
void QbManager::terminate()
{
  for (auto id : cube_index)
    deactivate(&comm_settings_t, id);
  closeRS485(&comm_settings_t);
  usleep(1000);
}

double QbManager::rad2tick(double cmd)
{
  return cmd * 32768.0 / (2.0 * M_PI);
}
inline void QbManager::compute_hand_motor_position(double d, short int &tick)
{
  tick = (short int)(d * HAND_CLOSURE);
}

void QbManager::activate(comm_settings *cs, int id)
{
  commActivate(cs, id, 1);
  usleep(1000);
}

void QbManager::deactivate(comm_settings *cs, int id)
{
  commActivate(cs, id, 0);
  usleep(1000);
}

void QbManager::handClosureCallback_rightArm(const std_msgs::Float64 &msg)
{
  if ((msg.data > 0.0) && (msg.data <= 1.0))
  {
    compute_hand_motor_position(msg.data, cmd_cube[1].at(0));
  }
  else if (msg.data <= 0.0)
  {
    compute_hand_motor_position(0.0, cmd_cube[1].at(0));
  }
  else if (msg.data > 1)
  {
    compute_hand_motor_position(1.0, cmd_cube[1].at(0));
  }
}
void QbManager::handClosureCallback_leftArm(const std_msgs::Float64 &msg)
{
  if ((msg.data > 0.0) && (msg.data <= 1.0))
  {
    compute_hand_motor_position(msg.data, cmd_cube[5].at(0));
  }
  else if (msg.data <= 0.0)
  {
    compute_hand_motor_position(0.0, cmd_cube[5].at(0));
  }
  else if (msg.data > 1)
  {
    compute_hand_motor_position(1.0, cmd_cube[5].at(0));
  }
}
void QbManager::motorPosiCallback_rightArm(const sensor_msgs::JointState &msg)
{
  cmd_cube[2].at(0) = (short int)rad2tick(msg.position[0]);
  cmd_cube[2].at(1) = (short int)rad2tick(msg.position[5]);
  cmd_cube[3].at(0) = (short int)rad2tick(msg.position[3]);
  cmd_cube[3].at(1) = (short int)rad2tick(msg.position[4]);
  cmd_cube[4].at(0) = (short int)rad2tick(msg.position[2]);
  cmd_cube[4].at(1) = (short int)rad2tick(msg.position[1]);
}

void QbManager::motorPosiCallback_leftArm(const sensor_msgs::JointState &msg)
{
  cmd_cube[6].at(0) = (short int)rad2tick(msg.position[0]);
  cmd_cube[6].at(1) = (short int)rad2tick(msg.position[5]);
  cmd_cube[7].at(0) = (short int)rad2tick(msg.position[3]);
  cmd_cube[7].at(1) = (short int)rad2tick(msg.position[4]);
  cmd_cube[8].at(0) = (short int)rad2tick(msg.position[2]);
  cmd_cube[8].at(1) = (short int)rad2tick(msg.position[1]);
}

void QbManager::neckPosiCallback(const sensor_msgs::JointState &msg)
{
  cmd_cube[9].at(0) = (short int)rad2tick(msg.position[0]);
  cmd_cube[9].at(1) = (short int)rad2tick(msg.position[0]);
  cmd_cube[10].at(0) = (short int)rad2tick(msg.position[1]);
  cmd_cube[10].at(1) = (short int)rad2tick(msg.position[1]);
}