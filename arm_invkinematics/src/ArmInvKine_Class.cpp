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
 * \file      ArmInvKine_Class.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "ArmInvKine_Class.h"

ArmInvKine::ArmInvKine()
{

  chain_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, -M_PI_2, 0.0, 0.0)));
  chain_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(LENGTH_J1, 0.0, 0.0, -M_PI_2)));
  chain_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(LENGTH_J2, 0.0, 0.0, 0.0)));
  chain_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, M_PI_2, 0.0, M_PI_2)));
  chain_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0.0, LENGTH_J3 + LENGTH_WRIST_TO_HAND, 0.0)));

  joint2pos_solver_ptr.reset(new KDL::ChainFkSolverPos_recursive(chain_arm));
  joint2jac_solver_ptr.reset(new KDL::ChainJntToJacSolver(chain_arm));

  jointPosiKDL_rightArm.resize(chain_arm.getNrOfJoints());
  jointPosiKDL_buf_rightArm.resize(chain_arm.getNrOfJoints());
  JacobianKDL_rightArm.resize(chain_arm.getNrOfJoints());
  jointPosiKDL_leftArm.resize(chain_arm.getNrOfJoints());
  jointPosiKDL_buf_leftArm.resize(chain_arm.getNrOfJoints());
  JacobianKDL_leftArm.resize(chain_arm.getNrOfJoints());

  KDL::SetToZero(jointPosiKDL_buf_rightArm);
  KDL::SetToZero(jointPosiKDL_buf_leftArm);

  Weight_rightArm.resize(chain_arm.getNrOfJoints(), chain_arm.getNrOfJoints());
  Weight_rightArm = Eigen::MatrixXd::Identity(chain_arm.getNrOfJoints(), chain_arm.getNrOfJoints());
  Weight_rightArm(0, 0) = WEIGHT_INIT;

  Weight_leftArm.resize(chain_arm.getNrOfJoints(), chain_arm.getNrOfJoints());
  Weight_leftArm = Eigen::MatrixXd::Identity(chain_arm.getNrOfJoints(), chain_arm.getNrOfJoints());
  Weight_leftArm(0, 0) = WEIGHT_INIT;

  k_posi = K_POSI_INIT;
  k_orient = K_ORIENT_INIT;

  k_rightArm = Eigen::Matrix<double, 6, 6>::Zero();
  k_rightArm.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * k_posi;
  k_rightArm.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * k_orient;

  k_leftArm = Eigen::Matrix<double, 6, 6>::Zero();
  k_leftArm.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * k_posi;
  k_leftArm.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * k_orient;

  jointVel_rightArm.resize(chain_arm.getNrOfJoints());
  jointVel_leftArm.resize(chain_arm.getNrOfJoints());

  jointPosi_upThresh.resize(chain_arm.getNrOfJoints());
  jointPosi_downThresh.resize(chain_arm.getNrOfJoints());

  nodeHandler.getParam("/jointPosi_upThresh_param", jointPosi_upThresh);
  nodeHandler.getParam("/jointPosi_downThresh_param", jointPosi_downThresh);
  nodeHandler.getParam("/jointVelLimit_param", jointVelLimit);
  nodeHandler.getParam("/jointVelLimit_reset_param", jointVelLimit_reset);

  reinit_rightArm();
  reinit_leftArm();

  subHandler_pose_ref_rightArm = nodeHandler.subscribe("/right_arm/pose_ref_endeffctor", 10, &ArmInvKine::poseCallback_rightArm, this);
  subHandler_pose_ref_leftArm = nodeHandler.subscribe("/left_arm/pose_ref_endeffctor", 10, &ArmInvKine::poseCallback_leftArm, this);
  subHandler_state_joy_rightArm = nodeHandler.subscribe("/right_arm/state_joy", 1, &ArmInvKine::joyStateCallback_rightArm, this);
  subHandler_state_joy_leftArm = nodeHandler.subscribe("/left_arm/state_joy", 1, &ArmInvKine::joyStateCallback_leftArm, this);
  pubHandler_state_joint_rightArm = nodeHandler.advertise<sensor_msgs::JointState>("/right_arm/state_joint", 10);
  pubHandler_state_joint_leftArm = nodeHandler.advertise<sensor_msgs::JointState>("/left_arm/state_joint", 10);
}

void ArmInvKine::joyStateCallback_rightArm(const std_msgs::Int16::ConstPtr &msg)
{
  state_joy_rightArm.data = msg->data;
}
void ArmInvKine::joyStateCallback_leftArm(const std_msgs::Int16::ConstPtr &msg)
{
  state_joy_leftArm.data = msg->data;
}
void ArmInvKine::poseCallback_rightArm(const geometry_msgs::Pose::ConstPtr &msg)
{
  tipPosi_ref_rightArm(0) = msg->position.x;
  tipPosi_ref_rightArm(1) = msg->position.y;
  tipPosi_ref_rightArm(2) = msg->position.z;

  tipQuat_ref_rightArm.w() = msg->orientation.w;
  tipQuat_ref_rightArm.x() = msg->orientation.x;
  tipQuat_ref_rightArm.y() = msg->orientation.y;
  tipQuat_ref_rightArm.z() = msg->orientation.z;
}
void ArmInvKine::poseCallback_leftArm(const geometry_msgs::Pose::ConstPtr &msg)
{
  tipPosi_ref_leftArm(0) = msg->position.x;
  tipPosi_ref_leftArm(1) = msg->position.y;
  tipPosi_ref_leftArm(2) = msg->position.z;

  tipQuat_ref_leftArm.w() = msg->orientation.w;
  tipQuat_ref_leftArm.x() = msg->orientation.x;
  tipQuat_ref_leftArm.y() = msg->orientation.y;
  tipQuat_ref_leftArm.z() = msg->orientation.z;
}

void ArmInvKine::rightArmInvKine()
{
  sensor_msgs::JointState jointState_pub;
  jointState_pub.name.resize(chain_arm.getNrOfJoints());
  jointState_pub.position.resize(chain_arm.getNrOfJoints());

  if (0 == state_joy_rightArm.data)
  {
    reinit_rightArm();
  }
  /*---------------------------get direct kinematic transmition matrix----------------------------------*/
  joint2pos_solver_ptr->JntToCart(jointPosiKDL_rightArm, frameKDL_hand2base_rightArm);

  /*---------------------------use weight matrix to arrange movements at singularity----------------------------------*/
  if (double sum = (jointPosiKDL_rightArm(1) + jointPosiKDL_rightArm(2) + jointPosiKDL_rightArm(3)) / (SINGULAR_DETECT_ANGLE) < 1)
  {
    Weight_rightArm(0, 0) = WEIGHT_SINGULAR + (WEIGHT_INIT - WEIGHT_SINGULAR) * sum;
  }
  else
  {
    Weight_rightArm(0, 0) = WEIGHT_INIT;
  }
  /*---------------------------calculate tip position and orientation----------------------------------*/
  Eigen::Vector3d tipPosi_fdb_rightArm = Eigen::Vector3d::Zero();
  Eigen::Matrix3d tipRota_fdb_rightArm = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond tipQuat_fdb_rightArm;
  for (int i = 0; i < 3; i++)
  {
    tipPosi_fdb_rightArm(i) = frameKDL_hand2base_rightArm.p(i);
  }
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tipRota_fdb_rightArm(i, j) = frameKDL_hand2base_rightArm.M(i, j);
    }
  }
  tipQuat_fdb_rightArm = tipRota_fdb_rightArm;
  tipQuat_fdb_rightArm.normalize();
  // avoid sign_flip
  double sign_check = tipQuat_fdb_rightArm.w() * tipQuat_buf_rightArm.w() + tipQuat_fdb_rightArm.x() * tipQuat_buf_rightArm.x() + tipQuat_fdb_rightArm.y() * tipQuat_buf_rightArm.y() + tipQuat_fdb_rightArm.z() * tipQuat_buf_rightArm.z();
  if (sign_check < 0.0)
  {
    tipQuat_fdb_rightArm.w() = tipQuat_fdb_rightArm.w() * (-1.0);
    tipQuat_fdb_rightArm.vec() = tipQuat_fdb_rightArm.vec() * (-1.0);
  }
  tipQuat_buf_rightArm = tipQuat_fdb_rightArm;
  /*---------------------------get Jacobian matrix----------------------------------*/
  joint2jac_solver_ptr->JntToJac(jointPosiKDL_rightArm, JacobianKDL_rightArm);
  Eigen::MatrixXd Jacobian(6, chain_arm.getNrOfJoints());
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < chain_arm.getNrOfJoints(); j++)
    {
      Jacobian(i, j) = JacobianKDL_rightArm(i, j); // Jac_R_5;J_R_5
    }
  }
  /*---------------------------get error----------------------------------*/
  tipQuat_ref_rightArm.normalize();
  Eigen::Vector3d tipQuat_ref_vec(tipQuat_ref_rightArm.x(), tipQuat_ref_rightArm.y(), tipQuat_ref_rightArm.z());
  Eigen::Matrix3d skew_ref_vec;
  skew_symmetric(tipQuat_ref_vec, skew_ref_vec);
  Eigen::Vector3d tipPosi_error = tipPosi_ref_rightArm - tipPosi_fdb_rightArm;
  Eigen::Vector3d tipQuat_error = tipQuat_fdb_rightArm.w() * tipQuat_ref_rightArm.vec() - tipQuat_ref_rightArm.w() * tipQuat_fdb_rightArm.vec() - skew_ref_vec * tipQuat_fdb_rightArm.vec();
  Eigen::VectorXd tipPose_error;
  tipPose_error.resize(6);
  tipPose_error << tipPosi_error, tipQuat_error;
  /*-------------------------------------------------------------------------*/
  if (tipPose_error.norm() > 0.001)
  {
    Eigen::MatrixXd Jaco_inter;
    Jaco_inter.resize(6, chain_arm.getNrOfJoints());
    Jaco_inter = Jacobian * Weight_rightArm;
    Eigen::MatrixXd Jaco_pinv_inter;
    Jaco_pinv_inter.resize(chain_arm.getNrOfJoints(), 6);
    pseudo_inverse_switched(Jaco_inter, Jaco_pinv_inter, true);
    Eigen::MatrixXd Jacobian_pinv;
    Jacobian_pinv.resize(chain_arm.getNrOfJoints(), 6);
    Jacobian_pinv = Weight_rightArm * Jaco_pinv_inter;
    jointVel_rightArm = Jacobian_pinv * (k_rightArm * tipPose_error);

    for (int i = 0; i < chain_arm.getNrOfJoints(); i++)
    {
      jointPosiKDL_rightArm(i) += jointVel_rightArm(i) * dt;
      if (jointPosiKDL_rightArm(i) > jointPosi_upThresh[i])
      {
        jointPosiKDL_rightArm(i) = jointPosi_upThresh[i];
      }
      else if (jointPosiKDL_rightArm(i) < jointPosi_downThresh[i])
      {
        jointPosiKDL_rightArm(i) = jointPosi_downThresh[i];
      }
    }
  }

  /*-----------------------------------joint velocity limit----------------------------------*/
  double maxStep = ((0 == state_joy_rightArm.data) ? jointVelLimit_reset : jointVelLimit) / NODEFREQ_ARMINVKINE;
  for (int i = 0; i < chain_arm.getNrOfJoints(); i++)
  {
    if (fabs(jointPosiKDL_rightArm(i) - jointPosiKDL_buf_rightArm(i)) > maxStep)
    {
      jointPosiKDL_rightArm(i) = jointPosiKDL_buf_rightArm(i) + signOfDouble(jointPosiKDL_rightArm(i) - jointPosiKDL_buf_rightArm(i)) * maxStep;
    }
    jointPosiKDL_buf_rightArm(i) = jointPosiKDL_rightArm(i);

    std::string name;
    name = "Joint_" + std::to_string(i) + "_right";
    jointState_pub.name[i] = name;
    jointState_pub.position[i] = jointPosiKDL_rightArm(i);
  }
  jointState_pub.header.stamp = ros::Time::now();
  pubHandler_state_joint_rightArm.publish(jointState_pub);
}

void ArmInvKine::leftArmInvKine()
{
  sensor_msgs::JointState jointState_pub;
  jointState_pub.name.resize(chain_arm.getNrOfJoints());
  jointState_pub.position.resize(chain_arm.getNrOfJoints());
  if (0 == state_joy_leftArm.data)
  {
    reinit_leftArm();
  }
  /*---------------------------get direct kinematic transmition matrix----------------------------------*/
  joint2pos_solver_ptr->JntToCart(jointPosiKDL_leftArm, frameKDL_hand2base_leftArm);

  /*---------------------------use weight matrix to arrange movements at singularity----------------------------------*/
  if (double sum = (jointPosiKDL_leftArm(1) + jointPosiKDL_leftArm(2) + jointPosiKDL_leftArm(3)) / (SINGULAR_DETECT_ANGLE) < 1)
  {
    Weight_leftArm(0, 0) = WEIGHT_SINGULAR + (WEIGHT_INIT - WEIGHT_SINGULAR) * sum;
  }
  else
  {
    Weight_leftArm(0, 0) = WEIGHT_INIT;
  }
  /*---------------------------calculate tip position and orientation----------------------------------*/
  Eigen::Vector3d tipPosi_fdb_leftArm = Eigen::Vector3d::Zero();
  Eigen::Matrix3d tipRota_fdb_leftArm = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond tipQuat_fdb_leftArm;
  for (int i = 0; i < 3; i++)
  {
    tipPosi_fdb_leftArm(i) = frameKDL_hand2base_leftArm.p(i);
  }
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      tipRota_fdb_leftArm(i, j) = frameKDL_hand2base_leftArm.M(i, j);
    }
  }
  tipQuat_fdb_leftArm = tipRota_fdb_leftArm;
  tipQuat_fdb_leftArm.normalize();

  // avoid sign_flip
  double sign_check = tipQuat_fdb_leftArm.w() * tipQuat_buf_leftArm.w() + tipQuat_fdb_leftArm.x() * tipQuat_buf_leftArm.x() + tipQuat_fdb_leftArm.y() * tipQuat_buf_leftArm.y() + tipQuat_fdb_leftArm.z() * tipQuat_buf_leftArm.z();
  if (sign_check < 0.0)
  {
    tipQuat_fdb_leftArm.w() = tipQuat_fdb_leftArm.w() * (-1.0);
    tipQuat_fdb_leftArm.vec() = tipQuat_fdb_leftArm.vec() * (-1.0);
  }
  tipQuat_buf_leftArm = tipQuat_fdb_leftArm;
  /*---------------------------get Jacobian matrix----------------------------------*/
  joint2jac_solver_ptr->JntToJac(jointPosiKDL_leftArm, JacobianKDL_leftArm);
  // std::cout<<"JacobianKDL_leftArm:" << JacobianKDL_leftArm.data  << std::endl;
  Eigen::MatrixXd Jacobian(6, chain_arm.getNrOfJoints());
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < chain_arm.getNrOfJoints(); j++)
    {
      Jacobian(i, j) = JacobianKDL_leftArm(i, j);
    }
  }
  // std::cout<<"Jacobian:" << Jacobian  << std::endl;
  /*---------------------------get error----------------------------------*/
  tipQuat_ref_leftArm.normalize();
  Eigen::Vector3d tipQuat_ref_vec(tipQuat_ref_leftArm.x(), tipQuat_ref_leftArm.y(), tipQuat_ref_leftArm.z());
  Eigen::Matrix3d skew_ref_vec;
  skew_symmetric(tipQuat_ref_vec, skew_ref_vec);
  Eigen::Vector3d tipPosi_error = tipPosi_ref_leftArm - tipPosi_fdb_leftArm;
  Eigen::Vector3d tipQuat_error = tipQuat_fdb_leftArm.w() * tipQuat_ref_leftArm.vec() - tipQuat_ref_leftArm.w() * tipQuat_fdb_leftArm.vec() - skew_ref_vec * tipQuat_fdb_leftArm.vec();
  Eigen::VectorXd tipPose_error;
  tipPose_error.resize(6);
  tipPose_error << tipPosi_error, tipQuat_error;
  // std::cout<<"tipPose_error:" << tipPose_error  << std::endl;
  /*-------------------------------------------------------------------------*/
  if (tipPose_error.norm() > 0.001)
  {
    Eigen::MatrixXd Jaco_inter;
    Jaco_inter.resize(6, chain_arm.getNrOfJoints());
    Jaco_inter = Jacobian * Weight_leftArm;
    //		std::cout<<"Jaco_inter:" << Jaco_inter  << std::endl;

    Eigen::MatrixXd Jaco_pinv_inter;
    Jaco_pinv_inter.resize(chain_arm.getNrOfJoints(), 6);
    pseudo_inverse_switched(Jaco_inter, Jaco_pinv_inter, true);
    //		std::cout<<"Jaco_pinv_inter:" << Jaco_pinv_inter  << std::endl;

    Eigen::MatrixXd Jacobian_pinv;
    Jacobian_pinv.resize(chain_arm.getNrOfJoints(), 6);
    Jacobian_pinv = Weight_leftArm * Jaco_pinv_inter;
    //		std::cout<<"I1:" << Jacobian*Jacobian_pinv << std::endl;
    //		std::cout<<"I2:" << Jacobian_pinv*Jacobian << std::endl;

    //		std::cout<<"Weight_leftArm:" << Weight_leftArm  << std::endl;
    //		std::cout<<"Jaco_pinv_inter:" << Jaco_pinv_inter  << std::endl;
    //		std::cout<<"Jacobian_pinv:" << Jacobian_pinv  << std::endl;

    jointVel_leftArm = Jacobian_pinv * (k_leftArm * tipPose_error);

    for (int i = 0; i < chain_arm.getNrOfJoints(); i++)
    {
      jointPosiKDL_leftArm(i) += jointVel_leftArm(i) * dt;
      if (jointPosiKDL_leftArm(i) > jointPosi_upThresh[i])
      {
        jointPosiKDL_leftArm(i) = jointPosi_upThresh[i];
      }
      else if (jointPosiKDL_leftArm(i) < jointPosi_downThresh[i])
      {
        jointPosiKDL_leftArm(i) = jointPosi_downThresh[i];
      }
    }
    //		std::cout<< "jointPosiKDL_leftArm:" << jointPosiKDL_leftArm.data <<std::endl;
  }

  /*-----------------------------------joint velocity limit----------------------------------*/
  double maxStep = ((0 == state_joy_leftArm.data) ? jointVelLimit_reset : jointVelLimit) / NODEFREQ_ARMINVKINE;
  for (int i = 0; i < chain_arm.getNrOfJoints(); i++)
  {
    if (fabs(jointPosiKDL_leftArm(i) - jointPosiKDL_buf_leftArm(i)) > maxStep)
    {
      jointPosiKDL_leftArm(i) = jointPosiKDL_buf_leftArm(i) + signOfDouble(jointPosiKDL_leftArm(i) - jointPosiKDL_buf_leftArm(i)) * maxStep;
    }
    jointPosiKDL_buf_leftArm(i) = jointPosiKDL_leftArm(i);

    std::string name;
    name = "Joint_" + std::to_string(i) + "_left";
    jointState_pub.name[i] = name;
    jointState_pub.position[i] = jointPosiKDL_leftArm(i);
  }
  jointState_pub.header.stamp = ros::Time::now();
  pubHandler_state_joint_leftArm.publish(jointState_pub);
}

void ArmInvKine::run()
{
  rightArmInvKine();
  leftArmInvKine();
}

inline void ArmInvKine::reinit_rightArm(void)
{
  tipQuat_buf_rightArm = Eigen::Quaterniond::Identity();
  tipQuat_ref_rightArm = Eigen::Quaterniond::Identity();
  tipPosi_ref_rightArm << 0.0, 0.0, (LENGTH_J1 + LENGTH_J2 + LENGTH_J3 + LENGTH_WRIST_TO_HAND);
  KDL::SetToZero(jointPosiKDL_rightArm);
  KDL::SetToZero(JacobianKDL_rightArm);
}
inline void ArmInvKine::reinit_leftArm(void)
{
  tipQuat_buf_leftArm = Eigen::Quaterniond::Identity();
  tipQuat_ref_leftArm = Eigen::Quaterniond::Identity();
  tipPosi_ref_leftArm << 0.0, 0.0, (LENGTH_J1 + LENGTH_J2 + LENGTH_J3 + LENGTH_WRIST_TO_HAND);
  KDL::SetToZero(jointPosiKDL_leftArm);
  KDL::SetToZero(JacobianKDL_leftArm);
}

int signOfDouble(double d)
{
  return (d < 0) ? -1 : 1;
}