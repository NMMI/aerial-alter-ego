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
 * \file      ArmInvKine_Class.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <boost/scoped_ptr.hpp>

#include "utils/pseudo_inversion_switched.h"
#include "utils/skew_symmetric.h"

#define NODEFREQ_ARMINVKINE ((double)200.0)

#define LENGTH_J0 ((double)0.07)
#define LENGTH_J1 ((double)0.11)
#define LENGTH_J2 ((double)0.14)
#define LENGTH_J3 ((double)0.16)
#define LENGTH_WRIST_TO_HAND ((double)0.06)

#define WEIGHT_INIT ((double)0.3)
#define WEIGHT_SINGULAR ((double)0.064)

#define DEG2RAD(deg) ((double)deg * M_PI / 180.0)
#define SINGULAR_DETECT_ANGLE (DEG2RAD(20.0))
#define K_POSI_INIT (4.5)
#define K_ORIENT_INIT (5.5)

class ArmInvKine
{
public:
  ArmInvKine();

  void run();
  double dt;

private:
  ros::NodeHandle nodeHandler;
  ros::Subscriber subHandler_pose_ref_rightArm, subHandler_pose_ref_leftArm, subHandler_state_joy_rightArm, subHandler_state_joy_leftArm;
  ros::Publisher pubHandler_state_joint_rightArm, pubHandler_state_joint_leftArm;
  KDL::Chain chain_arm;
  boost::scoped_ptr<KDL::ChainFkSolverPos> joint2pos_solver_ptr;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> joint2jac_solver_ptr;

  KDL::JntArray jointPosiKDL_rightArm, jointPosiKDL_leftArm, jointPosiKDL_buf_rightArm, jointPosiKDL_buf_leftArm;
  KDL::Frame frameKDL_hand2base_rightArm, frameKDL_hand2base_leftArm;
  KDL::Jacobian JacobianKDL_rightArm, JacobianKDL_leftArm;
  Eigen::VectorXd jointVel_rightArm, jointVel_leftArm;
  std::vector<double> jointPosi_upThresh, jointPosi_downThresh;

  Eigen::MatrixXd Weight_rightArm, Weight_leftArm;
  Eigen::Quaterniond tipQuat_ref_rightArm, tipQuat_ref_leftArm, tipQuat_buf_rightArm, tipQuat_buf_leftArm;
  Eigen::Vector3d tipPosi_ref_rightArm, tipPosi_ref_leftArm;
  Eigen::MatrixXd k_rightArm, k_leftArm;

  std_msgs::Int16 state_joy_rightArm, state_joy_leftArm;

  double k_posi, k_orient, jointVelLimit, jointVelLimit_reset;

  void joyStateCallback_rightArm(const std_msgs::Int16::ConstPtr &msg);
  void joyStateCallback_leftArm(const std_msgs::Int16::ConstPtr &msg);

  void poseCallback_rightArm(const geometry_msgs::Pose::ConstPtr &msg);
  void poseCallback_leftArm(const geometry_msgs::Pose::ConstPtr &msg);
  void rightArmInvKine();
  void leftArmInvKine();
  inline void reinit_rightArm(void);
  inline void reinit_leftArm(void);
};

int signOfDouble(double d);