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
 * \file      JoyBridge_Class.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "JoyBridge_Class.h"

JoyBridge::JoyBridge()
{
  std::string topic_index, topic_track, topic_middle, cmd_pose_ref, topic_state_joy, cmd_closure_hand;
  std::string topic_joy_x, topic_joy_y, topic_button1, topic_button2, cmd_joy_x, cmd_joy_y, cmd_button1, cmd_button2;
  std::vector<double> Vec_robotbase2shoulder;

  nodeHandler.getParam("name_param", arm_name);
  nodeHandler.getParam("topic_index_param", topic_index);
  nodeHandler.getParam("topic_track_param", topic_track);
  nodeHandler.getParam("topic_middle_param", topic_middle);
  nodeHandler.getParam("cmd_pose_ref", cmd_pose_ref);
  nodeHandler.getParam("length_arm_robot_param", length_arm_robot);
  nodeHandler.getParam("Vec_robotbase2shoulder_param", Vec_robotbase2shoulder);
  nodeHandler.getParam("topic_state_joy_param", topic_state_joy);
  nodeHandler.getParam("cmd_closure_hand", cmd_closure_hand);
  nodeHandler.getParam("topic_joy_x", topic_joy_x);
  nodeHandler.getParam("topic_joy_y", topic_joy_y);
  nodeHandler.getParam("topic_button1", topic_button1);
  nodeHandler.getParam("topic_button2", topic_button2);

  nodeHandler.getParam("cmd_joy_x", cmd_joy_x);
  nodeHandler.getParam("cmd_joy_y", cmd_joy_y);
  nodeHandler.getParam("cmd_button1", cmd_button1);
  nodeHandler.getParam("cmd_button2", cmd_button2);

  buf_button1 = false;
  state_joy.data = 0;
  flag_calibrate_init = false;
  memset(msg_index, 0, sizeof(msg_index));
  memset(msg_middle, 0, sizeof(msg_middle));
  closure_hand.data = 0.0;
  joy_x_cmd.data = 0;
  joy_y_cmd.data = 0;

  position_joy_cur << 0, 0, 0;
  position_joy_buf << 0, 0, 0;

  quat_joy_cur.w() = 1.0;
  quat_joy_cur.vec() << 0, 0, 0;
  quat_robothand_buf.w() = 1.0;
  quat_robothand_buf.vec() << 0, 0, 0;
  Trans_oculusbase2shoulder = Eigen::Matrix4d::Identity();
  ;
  Trans_robotbase2shoulder = Eigen::Matrix4d::Identity();
  Trans_robotbase2shoulder.block<3, 3>(0, 0) << Vec_robotbase2shoulder[0], Vec_robotbase2shoulder[1], Vec_robotbase2shoulder[2], Vec_robotbase2shoulder[3], Vec_robotbase2shoulder[4], Vec_robotbase2shoulder[5], Vec_robotbase2shoulder[6], Vec_robotbase2shoulder[7], Vec_robotbase2shoulder[8];
  std::cout << "Trans_robotbase2shoulder: " << Trans_robotbase2shoulder << std::endl;
  // INDEX TRIGGER
  subHandler_index = nodeHandler.subscribe(topic_index, 1, &JoyBridge::indexCallback, this);
  // HAND TRACKING
  subHandler_track = nodeHandler.subscribe(topic_track, 1, &JoyBridge::trackCallback, this);
  // MIDDLE TRIGGER
  subHandler_middle = nodeHandler.subscribe(topic_middle, 1, &JoyBridge::middleCallback, this);
  subHandler_joy_x = nodeHandler.subscribe(topic_joy_x, 1, &JoyBridge::joyXCallback, this);
  subHandler_joy_y = nodeHandler.subscribe(topic_joy_y, 1, &JoyBridge::joyYCallback, this);
  subHandler_button1 = nodeHandler.subscribe(topic_button1, 1, &JoyBridge::button1Callback, this);
  subHandler_button2 = nodeHandler.subscribe(topic_button2, 1, &JoyBridge::button2Callback, this);
  subHandler_state_joy = nodeHandler.subscribe(topic_state_joy, 1, &JoyBridge::joyStateCallback, this);

  pubHandler_tracking = nodeHandler.advertise<geometry_msgs::Pose>(cmd_pose_ref, 10);
  pubHandler_state_joy = nodeHandler.advertise<std_msgs::Int16>(topic_state_joy, 1);
  pubHandler_indexcmd = nodeHandler.advertise<std_msgs::Float64>(cmd_closure_hand, 1);
  pubHandler_joy_x = nodeHandler.advertise<std_msgs::Float64>(cmd_joy_x, 1);
  pubHandler_joy_y = nodeHandler.advertise<std_msgs::Float64>(cmd_joy_y, 1);
  pubHandler_button1 = nodeHandler.advertise<std_msgs::Bool>(cmd_button1, 1);

  pubHandler_state_joy.publish(state_joy);
}
JoyBridge::~JoyBridge()
{
}

void JoyBridge::joyStateCallback(const std_msgs::Int16::ConstPtr &msg)
{
  state_joy.data = msg->data;
}

void JoyBridge::button1Callback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true && buf_button1 == false)
  {

    if (0 == arm_name.compare("left_arm"))
    {
      std_msgs::Bool arm_cmd;
      arm_cmd.data = true;
      if (state_joy.data != 0)
        pubHandler_button1.publish(arm_cmd);
    }
    else
    {
    }
  }
  buf_button1 = msg->data;
}

void JoyBridge::button2Callback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true)
  {
    if (0 == arm_name.compare("left_arm"))
    {
    }
    else
    {
    }
    std_msgs::Int16 state;
    state.data = 0;
    pubHandler_state_joy.publish(state);
  }
}
void JoyBridge::joyXCallback(const std_msgs::Float64::ConstPtr &msg)
{
  joy_x_cmd.data = msg->data;
}
void JoyBridge::joyYCallback(const std_msgs::Float64::ConstPtr &msg)
{
  joy_y_cmd.data = msg->data;
}

void JoyBridge::indexCallback(const std_msgs::Float64::ConstPtr &msg)
{
  msg_index[0] = msg->data;
}

void JoyBridge::trackCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  position_joy_cur << msg->position.x, msg->position.y, msg->position.z;
  quat_joy_cur.w() = msg->orientation.w;
  quat_joy_cur.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;
  cntTimeout = 0;
}

void JoyBridge::middleCallback(const std_msgs::Float64::ConstPtr &msg)
{
  if (0 == arm_name.compare("left_arm"))
  {
  }
  else
  {
  }
}

void JoyBridge::run()
{

  cntTimeout++;
  if (cntTimeout > NODEFREQ_JOYBRIDGE * 5)
  {
    cntTimeout = NODEFREQ_JOYBRIDGE * 5;
  }

  switch (state_joy.data)
  {
  case 0:
    /*need calibrate*/
    if ((msg_index[0] < 0.5) && (msg_index[1] >= 0.5))
    {
      /*start calibrate*/
      if (false == flag_calibrate_init)
      {
        /*calibration initialize*/
        position_joy_buf = position_joy_cur;
        flag_calibrate_init = !flag_calibrate_init;
        std::cout << "START CALIBRATION: " << arm_name << std::endl;
      }
      else
      {
        /*second step of calibration*/
        Eigen::Vector3d diff = position_joy_buf - position_joy_cur;
        length_arm_user = sqrt(pow(diff.norm(), 2) / 2.0);
        std::cout << "user arm length: " << length_arm_user << std::endl;

        Eigen::Matrix4d Trans_joy2shoulder = Eigen::Matrix4d::Identity();
        Trans_joy2shoulder.block<3, 1>(0, 3) << 0.0, 0.0, length_arm_user;
        std::cout << "Trans_joy2shoulder: " << std::endl
                  << Trans_joy2shoulder << std::endl;

        Eigen::Matrix4d Trans_oculusbase2joy = Eigen::Matrix4d::Identity();
        Trans_oculusbase2joy.block<3, 1>(0, 3) = position_joy_cur;
        Trans_oculusbase2shoulder = Trans_oculusbase2joy * Trans_joy2shoulder;
        std::cout << "Trans_oculusbase2shoulder: " << std::endl
                  << Trans_oculusbase2shoulder << std::endl;

        std::cout << "END CALIBRATION: " << arm_name << std::endl;
        flag_calibrate_init = !flag_calibrate_init;

        state_joy.data = 1;
        pubHandler_state_joy.publish(state_joy);
      }
    }
    msg_index[1] = msg_index[0];
    break;
  case 1:
    // std::cout <<"START CONTROL: " << arm_name <<std::endl;
    // if((msg_middle[0]<0.5) && (msg_middle[1]>=0.5));
    // msg_middle[1]=msg_middle[0];
    /*track teleoperation*/
    Eigen::Matrix4d Trans_oculusbase2joy = Eigen::Matrix4d::Identity();
    Trans_oculusbase2joy.block<3, 3>(0, 0) = quat_joy_cur.toRotationMatrix();
    Trans_oculusbase2joy.block<3, 1>(0, 3) = position_joy_cur;
    // std::cout << "Trans_oculusbase2joy: " << Trans_oculusbase2joy << std::endl;

    Eigen::Matrix4d Trans_shoulder2oculusbase = Eigen::Matrix4d::Identity();
    bool invertible;
    Trans_oculusbase2shoulder.computeInverseWithCheck(Trans_shoulder2oculusbase, invertible);
    // std::cout << "Trans_shoulder2oculusbase: " << Trans_shoulder2oculusbase << std::endl;

    Eigen::Matrix4d Trans_shoulder2joy = Trans_shoulder2oculusbase * Trans_oculusbase2joy;
    // std::cout << "Trans_shoulder2joy: " << Trans_shoulder2joy << std::endl;

    /*transform to robot scale*/
    // std::cout << "Trans_shoulder2joy: " << Trans_shoulder2joy << std::endl;
    // std::cout << "length_arm_robot: " << length_arm_robot << std::endl;
    //  std::cout << "length_arm_user: " << length_arm_user << std::endl;
    Trans_shoulder2joy.block<3, 1>(0, 3) = Trans_shoulder2joy.block<3, 1>(0, 3) * length_arm_robot / length_arm_user;
    // std::cout << "robot Trans_shoulder2joy: " << Trans_shoulder2joy << std::endl;
    /*replace the base of the space*/
    Eigen::Matrix4d Trans_robotbase2robothand = Trans_robotbase2shoulder * Trans_shoulder2joy * Trans_robotbase2shoulder.transpose();

    //*std::cout << "Trans_shoulder2joy: " << Trans_shoulder2joy << std::endl;
    //*std::cout << "Trans_robotbase2robothand: " << Trans_robotbase2robothand << std::endl;

    Eigen::Vector3d position_robothand_ref = Trans_robotbase2robothand.block<3, 1>(0, 3);
    Eigen::Quaterniond quat_robothand_ref(Trans_robotbase2robothand.block<3, 3>(0, 0));
    quat_robothand_ref.normalize();

    // rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
    double sign_check = quat_robothand_ref.w() * quat_robothand_buf.w() + quat_robothand_ref.x() * quat_robothand_buf.x() + quat_robothand_ref.y() * quat_robothand_buf.y() + quat_robothand_ref.z() * quat_robothand_buf.z();
    if (sign_check < 0.0)
    {
      quat_robothand_ref.w() = quat_robothand_ref.w() * (-1);
      quat_robothand_ref.vec() = quat_robothand_ref.vec() * (-1);
    }
    quat_robothand_buf = quat_robothand_ref;
    // std::cout << "position_robothand_ref: " << position_robothand_ref << std::endl;
    // std::cout << "quat_robothand_ref: " << quat_robothand_ref.w() << quat_robothand_ref.x() << quat_robothand_ref.y() << quat_robothand_ref.z() << std::endl;

    geometry_msgs::Pose pose_ref;
    pose_ref.position.x = position_robothand_ref(0);
    pose_ref.position.y = position_robothand_ref(1);
    pose_ref.position.z = position_robothand_ref(2);
    pose_ref.orientation.w = quat_robothand_ref.w();
    pose_ref.orientation.x = quat_robothand_ref.x();
    pose_ref.orientation.y = quat_robothand_ref.y();
    pose_ref.orientation.z = quat_robothand_ref.z();

    closure_hand.data = msg_index[0];
    pubHandler_tracking.publish(pose_ref);
    pubHandler_joy_x.publish(joy_x_cmd);
    pubHandler_joy_y.publish(joy_y_cmd);
    pubHandler_indexcmd.publish(closure_hand);

    break;
  }
}