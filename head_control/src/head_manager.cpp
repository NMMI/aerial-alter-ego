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
 * \file      head_manager.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "ros/ros.h"
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <vector>
#include <string>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include <trajectory_msgs/JointTrajectory.h>

#include <sensor_msgs/JointState.h>

using namespace KDL;
using namespace std;

#define MAX_STIFF 0.3

Frame ref_frame;
Eigen::VectorXd ref_twist(6);
Eigen::VectorXd ref_stiffness(6);
int arm_side;
std::string ns;
Eigen::Matrix4d T_b_0;
Eigen::Matrix3d R_o_b;
ros::Time cmd_time, cmd_time_old;

/*---------------------------------------------------------------------*
 * POSTURE CALLBACK                                                     *
 *                                                                      *
 *----------------------------------------------------------------------*/
void posture__Callback(const geometry_msgs::Pose::ConstPtr &msg)
{
  Eigen::Quaterniond ref_quat;
  static Eigen::Quaterniond old_quat;
  double sign_check;
  Eigen::Vector3d r_p;
  Eigen::Matrix3d r_M;
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // r_p << msg->position.x, msg->position.y, msg->position.z;
  // r_p = r_p*0.38;
  ref_quat.x() = msg->orientation.x;
  ref_quat.y() = msg->orientation.y;
  ref_quat.z() = msg->orientation.z;
  ref_quat.w() = msg->orientation.w;

  sign_check = ref_quat.w() * old_quat.w() + ref_quat.x() * old_quat.x() + ref_quat.y() * old_quat.y() + ref_quat.z() * old_quat.z();
  if (sign_check < 0.0)
  {
    ref_quat.w() = -ref_quat.w();
    ref_quat.vec() = -ref_quat.vec();
  }
  old_quat = ref_quat;

  ref_quat = (R_o_b * ref_quat * R_o_b.transpose()) * T_b_0.block<3, 3>(0, 0).transpose();
  r_p = R_o_b * r_p + T_b_0.block<3, 1>(0, 3);
  r_M = ref_quat;

  ref_frame = Frame(Rotation(r_M(0, 0), r_M(0, 1), r_M(0, 2), r_M(1, 0), r_M(1, 1), r_M(1, 2), r_M(2, 0), r_M(2, 1), r_M(2, 2)), Vector(r_p(0), r_p(1), r_p(2)));

  transform.setOrigin(tf::Vector3(r_p(0), r_p(1), r_p(2)));
  transform.setRotation(tf::Quaternion(ref_quat.x(), ref_quat.y(), ref_quat.z(), ref_quat.w()));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root_link", ns + "_ref_head"));

  cmd_time_old = cmd_time;
  cmd_time = ros::Time::now();

  // cout << " " << endl;
  // cout << ref_frame.M << endl;
  // cout << " " << endl;
  // cout << ref_frame.p << endl;
}

/*---------------------------------------------------------------------*
 * STIFFNESS CALLBACK                                                   *
 *                                                                      *
 *----------------------------------------------------------------------*/
void stiffness__Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
  for (int i = 0; i < 2; i++)
    ref_stiffness(i) = msg->data[i];
}

/*---------------------------------------------------------------------*
 * MAIN                                                                 *
 *                                                                      *
 *----------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  // ------------------------------------------------------------------------------------- Init Var
  // --- node param ---
  std::string chain_topic;
  std::string pose_ref_topic;
  std::string stiff_ref_topic;
  std::string phantom_arm_topic;
  std::string ref_head_topic;
  int run_freq;
  ros::Subscriber sub_posture;
  ros::Subscriber sub_stiffness;
  ros::Publisher pub_cart_ref;
  ros::Publisher pub_phantom;
  ros::Publisher pub_head_ref;
  geometry_msgs::Pose cart_ref_msg;
  sensor_msgs::JointState phantom_msg;
  char buffer[50];

  sensor_msgs::JointState head_ref_msg;

  // --- kinematics ---
  std::vector<double> DH;
  std::vector<double> DH_Xtr;
  std::vector<double> DH_Xrot;
  std::vector<double> DH_Ztr;
  std::vector<double> DH_Zrot;
  std::vector<double> T_t2s;
  std::vector<double> T_o2t;
  std::vector<double> R_o2b;
  std::vector<double> q_min;
  std::vector<double> q_max;
  std::vector<int> qbmove_tf_ids;
  int softhand_tf_id;

  KDL::Chain chain;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;

  Eigen::VectorXd qMax_left(2);
  Eigen::VectorXd qMin_left(2);
  Eigen::VectorXd qMax_right(2);
  Eigen::VectorXd qMin_right(2);

  KDL::JntArray q;
  KDL::Jacobian JA_kdl;
  KDL::Frame act_frame;
  KDL::Twist err_twist;
  Eigen::VectorXd err_post(6);
  Eigen::VectorXd x_post(6);
  Eigen::MatrixXd K_v(6, 6);
  K_v << 50 * Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd JA(6, 2);
  Eigen::MatrixXd JA_pinv(2, 6);
  Eigen::MatrixXd K_d(6, 6);
  K_d << 0.1 * Eigen::MatrixXd::Identity(6, 6);
  double k_0;
  k_0 = 2;

  Eigen::VectorXd eq_dot(2);
  Eigen::VectorXd eq(2);
  Eigen::VectorXd eq_f(2);

  double arm_l;
  Eigen::Quaterniond act_quat;

  unsigned long int msg_seq(0);
  double stiffn;

  ros::Duration max_cmd_time = ros::Duration(1);
  ros::Duration filt_time = ros::Duration(5);
  ros::Duration max_cmd_latency = ros::Duration(1);
  ros::Time start_f_time;
  int act_bp(1);
  double alpha(1);

  // ------------------------------------------------------------------------------------- Init node
  ros::init(argc, argv, "head_manager");
  ros::NodeHandle n;
  ns = ros::this_node::getNamespace();
  ns = ns.substr(1, ns.length() - 1);

  // --- Rviz ---
  tf::TransformBroadcaster ik_br;
  tf::Transform ik_tf;

  // ------------------------------------------------------------------------------------- Check/save args
  n.getParam("/head/DH_Xtr", DH_Xtr);
  n.getParam("/head/DH_Xrot", DH_Xrot);
  n.getParam("/head/DH_Ztr", DH_Ztr);
  n.getParam("/head/DH_Zrot", DH_Zrot);
  n.getParam("/head/T_t2s", T_t2s);
  n.getParam("/head/T_o2t", T_o2t);
  n.getParam("/head/R_o2b", R_o2b);
  n.getParam("/head/q_min", q_min);
  n.getParam("/head/q_max", q_max);
  n.getParam("/head/qbmove_tf_ids", qbmove_tf_ids);
  n.getParam("/head/pose_ref_topic", pose_ref_topic);
  n.getParam("/head/stiff_ref_topic", stiff_ref_topic);
  n.getParam("/head/phantom_arm_topic", phantom_arm_topic);
  n.getParam("/head/ref_head_topic", ref_head_topic);
  n.getParam("/head/active_back_pos", act_bp);

  run_freq = 50;
  n.getParam("/head/head_frequency", run_freq); // Override if configured
  ros::Rate loop_rate(run_freq);

  // ------------------------------------------------------------------------------------- Kinematics

  chain.addSegment(Segment(Joint(Joint::None), Frame(Rotation(T_t2s[0], T_t2s[4], T_t2s[8], T_t2s[1], T_t2s[5], T_t2s[9], T_t2s[2], T_t2s[6], T_t2s[10]),
                                                     Vector(T_t2s[3], T_t2s[7], T_t2s[11]))));
  chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[0], DH_Xrot[0], DH_Ztr[0], DH_Zrot[0])));
  chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH(DH_Xtr[1], DH_Xrot[1], DH_Ztr[1], DH_Zrot[1])));

  T_b_0 << T_o2t[0], T_o2t[1], T_o2t[2], T_o2t[3],
      T_o2t[4], T_o2t[5], T_o2t[6], T_o2t[7],
      T_o2t[8], T_o2t[9], T_o2t[10], T_o2t[11],
      T_o2t[12], T_o2t[13], T_o2t[14], T_o2t[15];

  R_o_b << R_o2b[0], R_o2b[1], R_o2b[2],
      R_o2b[3], R_o2b[4], R_o2b[5],
      R_o2b[6], R_o2b[7], R_o2b[8];

  jnt_to_pose_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(chain));

  q.resize(chain.getNrOfJoints());
  JA_kdl.resize(chain.getNrOfJoints());
  KDL::SetToZero(q);

  jnt_to_pose_solver->JntToCart(q, ref_frame);
  jnt_to_jac_solver->JntToJac(q, JA_kdl);
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 2; j++)
      if (fabs(JA_kdl(i, j)) > 0.000001)
        JA(i, j) = JA_kdl(i, j);
      else
        JA(i, j) = 0;
  }

  JA_pinv = JA.transpose() * ((JA * JA.transpose() + K_d).inverse());

  ref_twist << 0, 0, 0, 0, 0, 0;

  eq << 0, 0;
  eq_f << 0, 0;

  // ------------------------------------------------------------------------------------- Subscribe to topics
  sub_posture = n.subscribe(pose_ref_topic, 1, posture__Callback);
  sub_stiffness = n.subscribe(stiff_ref_topic, 1, stiffness__Callback);

  // ------------------------------------------------------------------------------------- Published topics
  // ros::Publisher    	pub_inv_kin		= n.advertise<qb_interface::cubeEq_Preset>(target_chain + "_eq_pre", 1000);
  // pub_cart_ref	= n.advertise<geometry_msgs::Pose>(chain_topic, 1);
  pub_head_ref = n.advertise<sensor_msgs::JointState>(ref_head_topic, 1);
  pub_phantom = n.advertise<sensor_msgs::JointState>(phantom_arm_topic, 1);

  stiffn = MAX_STIFF * 0.9;

  cmd_time = ros::Time::now();
  cmd_time_old = ros::Time::now();

  // ------------------------------------------------------------------------------------- MAIN LOOP
  while (ros::ok())
  {
    // --- Inverse Kinematics ---
    jnt_to_pose_solver->JntToCart(q, act_frame);
    jnt_to_jac_solver->JntToJac(q, JA_kdl);
    for (int i = 0; i < 6; i++)
    {
      for (int j = 0; j < 2; j++)
        if (fabs(JA_kdl(i, j)) > 0.000001)
          JA(i, j) = JA_kdl(i, j);
        else
          JA(i, j) = 0;
    }

    err_twist = KDL::diff(act_frame, ref_frame);

    err_post << err_twist[0], err_twist[1], err_twist[2], err_twist[3], err_twist[4], err_twist[5];
    err_post = K_v * err_post;

    JA_pinv = JA.transpose() * ((JA * JA.transpose() + K_d).inverse());

    eq_dot = JA_pinv * err_post;

    eq += eq_dot / run_freq;

    // Back position
    if (act_bp == 1 && (ros::Time::now() - cmd_time > max_cmd_time))
    {
      eq << 0, 0;
      start_f_time = ros::Time::now();
      // cout << "1  " << eq_f.transpose() << endl;
      alpha = 1;
    }

    // Check latency between msgs
    // if (cmd_time-cmd_time_old > max_cmd_latency){
    // 	start_f_time = ros::Time::now();
    // 	// cout << "2  " << cmd_time << endl;
    // 	// cout << "2  " << cmd_time_old << endl;
    // }

    // Filtering position
    if (ros::Time::now() - start_f_time < filt_time)
    {
      alpha -= 1 / (filt_time.toSec() * run_freq);
      if (alpha < 0)
        alpha = 0;
      eq_f = alpha * eq_f + (1 - alpha) * eq;
      // cout << "3  " << eq_f.transpose() << endl;
    }
    else
      eq_f = eq;

    for (int i = 0; i < 2; i++)
    {
      if (eq_f(i) > q_max[i])
      {
        eq_f(i) = q_max[i];
      }
      if (eq_f(i) < q_min[i])
      {
        eq_f(i) = q_min[i];
      }

      q(i) = eq_f(i);
      eq(i) = eq_f(i);
    }
    act_frame.M.GetQuaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w());

    // --- publish all messages ---
    // Rviz TF:
    ik_tf.setOrigin(tf::Vector3(act_frame.p[0], act_frame.p[1], act_frame.p[2]));
    ik_tf.setRotation(tf::Quaternion(act_quat.x(), act_quat.y(), act_quat.z(), act_quat.w()));
    ik_br.sendTransform(tf::StampedTransform(ik_tf, ros::Time::now(), "root_link", ns + "_ego_ik"));

    head_ref_msg.name.resize(chain.getNrOfJoints());
    head_ref_msg.position.resize(chain.getNrOfJoints());

    head_ref_msg.name[0] = "neck_0";
    head_ref_msg.name[1] = "neck_1";
    head_ref_msg.position[0] = q(0);
    head_ref_msg.position[1] = q(1);

    // Rviz model:
    phantom_msg.name.resize(chain.getNrOfJoints());
    phantom_msg.position.resize(chain.getNrOfJoints());
    for (int i = 0; i < 2; i++)
    {
      sprintf(buffer, "phantom_cube%d_shaft_joint", qbmove_tf_ids[i]);
      phantom_msg.name[i] = buffer;
      phantom_msg.position[i] = q(i);
    }

    pub_head_ref.publish(head_ref_msg);
    pub_phantom.publish(phantom_msg);

    // // --- cycle ---
    ros::spinOnce();
    loop_rate.sleep();
  }
}