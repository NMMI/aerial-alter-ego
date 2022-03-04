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
 * \file      ArmInvRout_Class.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "ArmInvRout_Class.h"
ArmInvRout::ArmInvRout()
{

  subHandler_state_joint_rightArm = nodeHandler.subscribe("/right_arm/state_joint", 10, &ArmInvRout::invRoutCallback_rightArm, this);
  subHandler_state_joint_leftArm = nodeHandler.subscribe("/left_arm/state_joint", 10, &ArmInvRout::invRoutCallback_leftArm, this);
  motorPosi_rightArm.name.resize(6);
  motorPosi_rightArm.position.resize(6);
  motorPosi_leftArm.name.resize(6);
  motorPosi_leftArm.position.resize(6);

  motorPosi_rightArm.name[0] = "shoulder_right";
  motorPosi_rightArm.name[1] = "cube_1_right";
  motorPosi_rightArm.name[2] = "cube_2_right";
  motorPosi_rightArm.name[3] = "cube_3_right";
  motorPosi_rightArm.name[4] = "cube_4_right";
  motorPosi_rightArm.name[5] = "wrist_right";

  motorPosi_leftArm.name[0] = "shoulder_left";
  motorPosi_leftArm.name[1] = "cube_1_left";
  motorPosi_leftArm.name[2] = "cube_2_left";
  motorPosi_leftArm.name[3] = "cube_3_left";
  motorPosi_leftArm.name[4] = "cube_4_left";
  motorPosi_leftArm.name[5] = "wrist_left";

  for (int i = 0; i < motorPosi_rightArm.position.size(); i++)
  {
    motorPosi_rightArm.position[i] = 0;
    motorPosi_leftArm.position[i] = 0;
  }

  pubHandler_posi_motor_rightArm = nodeHandler.advertise<sensor_msgs::JointState>("/right_arm/motor_position", 10);
  pubHandler_posi_motor_leftArm = nodeHandler.advertise<sensor_msgs::JointState>("/left_arm/motor_position", 10);
}

void ArmInvRout::invRoutCallback_rightArm(const sensor_msgs::JointState::ConstPtr &msg)
{
  motorPosi_rightArm.position[0] = msg->position[0];
  motorPosi_rightArm.position[5] = msg->position[4];

  motorPosi_rightArm.position[1] = msg->position[1] * ETA;                       // j1 pos
  motorPosi_rightArm.position[2] = msg->position[2] * ETA;                       // j2 neg
  motorPosi_rightArm.position[3] = (-msg->position[1] - msg->position[2]) * ETA; // j1 neg j2 pos
  motorPosi_rightArm.position[4] = msg->position[3] * ETA;                       // j3 pos

  motorPosi_rightArm.header.stamp = ros::Time::now();
  pubHandler_posi_motor_rightArm.publish(motorPosi_rightArm);
}
void ArmInvRout::invRoutCallback_leftArm(const sensor_msgs::JointState::ConstPtr &msg)
{
  motorPosi_leftArm.position[0] = msg->position[0];
  motorPosi_leftArm.position[5] = msg->position[4];

  motorPosi_leftArm.position[1] = msg->position[1] * ETA;
  motorPosi_leftArm.position[2] = msg->position[2] * ETA;
  motorPosi_leftArm.position[3] = (-msg->position[1] - msg->position[2]) * ETA;
  motorPosi_leftArm.position[4] = msg->position[3] * ETA;

  motorPosi_leftArm.header.stamp = ros::Time::now();
  pubHandler_posi_motor_leftArm.publish(motorPosi_leftArm);
}