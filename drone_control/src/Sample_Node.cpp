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
 * \file      Sample_Node.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
// ----------------------------------------------------------------------------

#include "Sample_Node.h"
#include "../../Onboard-SDK-ROS/dji_sdk/include/dji_sdk/dji_sdk.h"
#include <cmath>

#define SAMPLE_FRQ 50

const float deg2rad = C_PI / 180.0;
const float rad2deg = 180.0 / C_PI;

int testcmd1 = 0;
ros::Publisher ctrlGenericPub;
ros::Time time_start;
std::ofstream myFile;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_arm_service;
bool drone_arm_status = false;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

// os::Publisher ctrlPosYawPub;
// ros::Publisher ctrlBrakePub;
ManualCtrlCmd droneCtrlCmd;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode = 255;

// sensor_msgs::NavSatFix current_gps;
// geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

// Mission square_mission;
geometry_msgs::QuaternionStamped attitude_recorder;
geometry_msgs::Vector3Stamped angularRate_recorder;
geometry_msgs::Vector3Stamped velocity_recorder;
geometry_msgs::Vector3Stamped acceleration_recorder;
sensor_msgs::NavSatFix gps_position_recorder;
sensor_msgs::Joy rc_recorder;
std_msgs::UInt8 control_device_recorder;
std_msgs::Int16 motor_speed_recorder;
bool takeoff_flag = false;

void thrustCmdCallback(const std_msgs::Float64::ConstPtr &msg)
{
  droneCtrlCmd.thrust = msg->data;
}
void yawCmdCallback(const std_msgs::Float64::ConstPtr &msg)
{
  droneCtrlCmd.yawRate = -msg->data;
}
void pitchCmdCallback(const std_msgs::Float64::ConstPtr &msg)
{
  droneCtrlCmd.pitch = msg->data;
}
void rollCmdCallback(const std_msgs::Float64::ConstPtr &msg)
{
  droneCtrlCmd.roll = msg->data;
}

void fileCreatCallback(const std_msgs::String::ConstPtr &msg)
{
  std::string filepath = msg->data + ".csv";
  ROS_INFO("%s", filepath.c_str());
  myFile.open(filepath);
  if (myFile.is_open())
  {
    std::vector<std::string> header;
    header = {"time", "cmd_thrust", "cmd_roll", "cmd_pitch", "cmd_yawrate",
              "p_x", "p_y", "p_z",
              "v_x", "v_y", "v_z",
              "a_x", "a_y", "a_z",
              "q_w", "q_x", "q_y", "q_z",
              "angRate_x", "angRate_y", "angRate_z"};
    for (int j = 0; j < header.size(); ++j)
    {
      myFile << header.at(j);
      if (j != header.size() - 1)
        myFile << ","; // No comma at end of line
    }
    myFile << "\n";
  }
  else
  {
    ROS_ERROR("File not open!");
  }

  time_start = ros::Time::now();
  testcmd1 = 2;
}

void attitude_recorder_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_attitude);
  attitude_recorder.header = msg->header;
  attitude_recorder.quaternion = msg->quaternion;
}
void angularRate_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_angularRate);
  angularRate_recorder.header = msg->header;
  angularRate_recorder.vector = msg->vector;
}
void velocity_recorder_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_velocity);
  velocity_recorder.header = msg->header;
  velocity_recorder.vector = msg->vector;
}
void acceleration_recorder_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_acceleration);
  acceleration_recorder.header = msg->header;
  acceleration_recorder.vector = msg->vector;
}
void gps_position_recorder_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_gps_position);
  gps_position_recorder.header = msg->header;
  gps_position_recorder.latitude = msg->latitude;
  gps_position_recorder.longitude = msg->longitude;
  gps_position_recorder.altitude = msg->altitude;
  gps_position_recorder.position_covariance = msg->position_covariance;
  gps_position_recorder.position_covariance_type = msg->position_covariance_type;
  gps_position_recorder.status = msg->status;
}
void rc_recorder_callback(const sensor_msgs::Joy::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_rc);
  rc_recorder.header = msg->header;
  rc_recorder.buttons = msg->buttons;
  rc_recorder.axes = msg->axes;
}
void control_device_recorder_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_control_device);
  control_device_recorder.data = msg->data;
}
void motor_speed_recorder_callback(const std_msgs::Int16::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(mutex_motor_speed);
  motor_speed_recorder.data = msg->data;
}

void test_callback_1(const std_msgs::Int16::ConstPtr &msg)
{
  testcmd1 = msg->data;
}

void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> dataset)
{
  // Make a CSV file with one or more columns of integer values
  // Each column of data is represented by the pair <column name, column data>
  //   as std::pair<std::string, std::vector<int>>
  // The dataset is represented as a vector of these columns
  // Note that all columns should be the same size

  // Create an output filestream object
  std::ofstream myFile(filename);

  // Send column names to the stream
  for (int j = 0; j < dataset.size(); ++j)
  {
    myFile << dataset.at(j).first;
    if (j != dataset.size() - 1)
      myFile << ","; // No comma at end of line
  }
  myFile << "\n";

  // Send data to the stream
  for (int i = 0; i < dataset.at(0).second.size(); ++i)
  {
    for (int j = 0; j < dataset.size(); ++j)
    {
      myFile << dataset.at(j).second.at(i);
      if (j != dataset.size() - 1)
        myFile << ","; // No comma at end of line
    }
    myFile << "\n";
  }

  // Close the file
  myFile.close();
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  current_local_pos = msg->point;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_droneControl");
  ros::NodeHandle nh;

  unsigned int time_count = 0;

  droneCtrlCmd.pitch = 0;
  droneCtrlCmd.roll = 0;
  droneCtrlCmd.yawRate = 0;
  droneCtrlCmd.thrust = 0;

  unsigned int sdk_cmd_count = 0;
  unsigned int datasave_count = 0;
  // Subscribe to messages from dji_sdk_node
  // ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  // ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);

  ros::Subscriber thrustCmdSub = nh.subscribe("/drone_control/thrust", 1, &thrustCmdCallback);
  ros::Subscriber yawCmdSub = nh.subscribe("/drone_control/yawRate", 1, &yawCmdCallback);
  ros::Subscriber pitchCmdSub = nh.subscribe("/drone_control/pitch", 1, &pitchCmdCallback);
  ros::Subscriber rollCmdSub = nh.subscribe("/drone_control/roll", 1, &rollCmdCallback);

  ros::Subscriber fileNameSub = nh.subscribe("/filename_sample", 1, &fileCreatCallback);
  //  ros::Subscriber droneArmSub = nh.subscribe("/drone_control/drone_arm_cmd", 1, &drone_arm_callback);

  ros::Subscriber attitude_subscriber = nh.subscribe("dji_sdk/attitude", 10, &attitude_recorder_callback);
  ros::Subscriber angularRate_subscriber = nh.subscribe("dji_sdk/angular_velocity_fused", 10, &angularRate_callback);
  ros::Subscriber velocity_subscriber = nh.subscribe("dji_sdk/velocity", 10, &velocity_recorder_callback);
  ros::Subscriber acceleration_subscriber = nh.subscribe("dji_sdk/acceleration_ground_fused", 10, &acceleration_recorder_callback);
  ros::Subscriber gps_position_subscriber = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_recorder_callback);
  ros::Subscriber rc_subscriber = nh.subscribe("dji_sdk/rc", 10, &rc_recorder_callback);
  ros::Subscriber control_device_subscriber = nh.subscribe("dji_sdk/control_device", 10, &control_device_recorder_callback);
  ros::Subscriber motor_speed_subscriber = nh.subscribe("dji_sdk/motor_speed", 10, &motor_speed_recorder_callback);

  ros::Subscriber test_subscriber_1 = nh.subscribe("/testcmd1", 1, &test_callback_1);

  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // Publish the control signal
  // ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  // ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  ctrlGenericPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1);
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_arm_service = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");

  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  if (is_M100())
  {
    ROS_INFO("Is this M100?");
    // takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = monitoredTakeoff();
  }

  if (takeoff_result)
  {
    takeoff_flag = true;
    //   square_mission.reset();
    //   square_mission.start_gps_location = current_gps;
    //   square_mission.start_local_position = current_local_pos;
    //   ROS_INFO("local position x:%f, y:%f, z:%f",current_local_pos.x,current_local_pos.y,current_local_pos.z);
    //   square_mission.setTarget(0, 20, 3, 60);
    //   square_mission.state = 1;
    //   ROS_INFO("##### Start route %d ....", square_mission.state);
  }

  ros::Rate r(NODEFREQ_ARMINVKINE);

  while (ros::ok())
  {
    sdk_cmd_count++;

    if (sdk_cmd_count < (unsigned int)(NODEFREQ_ARMINVKINE / SAMPLE_FRQ))
    {
    }
    else
    {
      sdk_cmd_count = 0;
      if (takeoff_flag == true)
      {

        // ROS_INFO("cmd time:%f", (ros::Time::now() - time_start).toSec());
        // droneCtrlCmd.thrust = 30; //29.2009801
        // droneCtrlCmd.yawRate = 0;
        // droneCtrlCmd.pitch = 0;
        // droneCtrlCmd.roll = 0;
        if (testcmd1 > 0)
        {
          time_count++;
          if (time_count < 2 * SAMPLE_FRQ)
          {
            droneCtrlCmd.thrust = 29.2009801;
            droneCtrlCmd.yawRate = 0;
            droneCtrlCmd.pitch = 0;
            droneCtrlCmd.roll = 0;
          }
          else
          {
            droneCtrlCmd.thrust = 29.2009801;
            droneCtrlCmd.yawRate = 10 * sin(M_PI * 2 * 2 * time_count / SAMPLE_FRQ) / 180.0 * M_PI;
            droneCtrlCmd.pitch = 0;
            droneCtrlCmd.roll = 0;
          }

          droneMovementCtrl(droneCtrlCmd);
        }
        // ROS_INFO("cmd sent:%f", (ros::Time::now() - time_start).toSec());
        //  Write the vector to CSV
        if (testcmd1 > 1)
        {

          if (myFile.is_open())
          {
            std::lock_guard<std::mutex> lock1(mutex_attitude);
            std::lock_guard<std::mutex> lock2(mutex_angularRate);
            std::lock_guard<std::mutex> lock3(mutex_velocity);
            std::lock_guard<std::mutex> lock4(mutex_acceleration);
            std::lock_guard<std::mutex> lock5(mutex_motor_speed);

            std::vector<double> vals;
            vals = {(attitude_recorder.header.stamp - time_start).toSec(), droneCtrlCmd.thrust, droneCtrlCmd.roll, droneCtrlCmd.pitch, droneCtrlCmd.yawRate,
                    current_local_pos.x, current_local_pos.y, current_local_pos.z,
                    velocity_recorder.vector.x, velocity_recorder.vector.y, velocity_recorder.vector.z,
                    acceleration_recorder.vector.x, acceleration_recorder.vector.y, acceleration_recorder.vector.z,
                    attitude_recorder.quaternion.w, attitude_recorder.quaternion.x, attitude_recorder.quaternion.y, attitude_recorder.quaternion.z,
                    angularRate_recorder.vector.x, angularRate_recorder.vector.y, angularRate_recorder.vector.z};
            for (int j = 0; j < vals.size(); ++j)
            {
              myFile << vals.at(j);
              if (j != vals.size() - 1)
                myFile << ","; // No comma at end of line
            }
            myFile << "\n";
            datasave_count++;
            // ROS_INFO("save finished:%f", (ros::Time::now() - time_start).toSec());
          }
          else
          {
            ROS_ERROR("File not open!");
          }

          if (datasave_count >= 10 * SAMPLE_FRQ)
          {
            ROS_INFO("test finished");
            myFile.close();
            datasave_count = 0;
            testcmd1 = 0;
            time_count = 0;
          }
        }
      }
    }
    // std::cout<<"h_space" << h_space << std::endl;
    ros::spinOnce();
    r.sleep();

  } // end while()
  if (myFile.is_open())
  {
    myFile.close();
  }
  return 0;
}

// void drone_arm_callback(const std_msgs::Bool::ConstPtr &msg)
// {

//   dji_sdk::DroneArmControl armCtrl;
//   armCtrl.request.arm = (false == drone_arm_status) ? dji_sdk::DroneArmControl::Request::ARM_COMMAND : dji_sdk::DroneArmControl::Request::DISARM_COMMAND;
//   drone_arm_service.call(armCtrl);

//   if (!armCtrl.response.result)
//   {
//     if (false == drone_arm_status)
//     {
//       ROS_ERROR("drone arm failed!");
//     }
//     else
//     {
//       ROS_ERROR("drone disarm failed!");
//     }
//   }
//   else
//   {
//     if (false == drone_arm_status)
//     {
//       ROS_INFO("drone arm succeeded! take care of the propellers!");
//     }
//     else
//     {
//       ROS_INFO("drone disarm succeeded!");
//     }
//     drone_arm_status = !drone_arm_status;
//   }
// }

void droneMovementCtrl(ManualCtrlCmd cmd)
{

  sensor_msgs::Joy controlMaual;
  uint8_t flag = (DJISDK::VERTICAL_THRUST |
                  DJISDK::HORIZONTAL_ANGLE |
                  DJISDK::YAW_RATE |
                  DJISDK::HORIZONTAL_BODY |
                  DJISDK::STABLE_ENABLE);
  controlMaual.axes.push_back(cmd.roll);
  controlMaual.axes.push_back(cmd.pitch);
  controlMaual.axes.push_back(cmd.thrust);
  controlMaual.axes.push_back(cmd.yawRate);
  controlMaual.axes.push_back(flag);

  // ROS_INFO("roll: %f pitch: %f thrust: %f yaw: %f", cmd.roll, cmd.pitch, cmd.thrust, cmd.yawRate);
  ctrlGenericPub.publish(controlMaual);

  return;
}

// // Helper Functions

// /*! Very simple calculation of local NED offset between two pairs of GPS
// /coordinates. Accurate when distances are small.
// !*/
// void
// localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
//                          sensor_msgs::NavSatFix& target,
//                          sensor_msgs::NavSatFix& origin)
// {
//   double deltaLon = target.longitude - origin.longitude;
//   double deltaLat = target.latitude - origin.latitude;

//   deltaNed.y = deltaLat * deg2rad * C_EARTH;
//   deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
//   deltaNed.z = target.altitude - origin.altitude;
// }

// geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
// {
//   geometry_msgs::Vector3 ans;

//   tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
//   R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
//   return ans;
// }

// void Mission::step()
// {
//   static int info_counter = 0;
//   geometry_msgs::Vector3     localOffset;

//   float speedFactor         = 2;
//   float yawThresholdInDeg   = 2;

//   float xCmd, yCmd, zCmd;

//   localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);

//   double xOffsetRemaining = target_offset_x - localOffset.x;
//   double yOffsetRemaining = target_offset_y - localOffset.y;
//   double zOffsetRemaining = target_offset_z - localOffset.z;

//   double yawDesiredRad     = deg2rad * target_yaw;
//   double yawThresholdInRad = deg2rad * yawThresholdInDeg;
//   double yawInRad          = toEulerAngle(current_atti).z;

//   info_counter++;
//   if(info_counter > 25)
//   {
//     info_counter = 0;
//     ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x,localOffset.y, localOffset.z,yawInRad);
//     ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining,yOffsetRemaining, zOffsetRemaining,yawInRad - yawDesiredRad);
//   }
//   if (abs(xOffsetRemaining) >= speedFactor)
//     xCmd = (xOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
//   else
//     xCmd = xOffsetRemaining;

//   if (abs(yOffsetRemaining) >= speedFactor)
//     yCmd = (yOffsetRemaining>0) ? speedFactor : -1 * speedFactor;
//   else
//     yCmd = yOffsetRemaining;

//   zCmd = start_local_position.z + target_offset_z;

//   /*!
//    * @brief: if we already started breaking, keep break for 50 sample (1sec)
//    *         and call it done, else we send normal command
//    */

//   if (break_counter > 50)
//   {
//     ROS_INFO("##### Route %d finished....", state);
//     finished = true;
//     return;
//   }
//   else if(break_counter > 0)
//   {
//     sensor_msgs::Joy controlVelYawRate;
//     uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
//                 DJISDK::HORIZONTAL_VELOCITY |
//                 DJISDK::YAW_RATE            |
//                 DJISDK::HORIZONTAL_GROUND   |
//                 DJISDK::STABLE_ENABLE);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(0);
//     controlVelYawRate.axes.push_back(flag);

//     ctrlBrakePub.publish(controlVelYawRate);
//     break_counter++;
//     return;
//   }
//   else //break_counter = 0, not in break stage
//   {
//     sensor_msgs::Joy controlPosYaw;

//     controlPosYaw.axes.push_back(xCmd);
//     controlPosYaw.axes.push_back(yCmd);
//     controlPosYaw.axes.push_back(zCmd);
//     controlPosYaw.axes.push_back(yawDesiredRad);
//     ctrlPosYawPub.publish(controlPosYaw);
//   }

//   if (std::abs(xOffsetRemaining) < 0.5 &&
//       std::abs(yOffsetRemaining) < 0.5 &&
//       std::abs(zOffsetRemaining) < 0.5 &&
//       std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
//   {
//     //! 1. We are within bounds; start incrementing our in-bound counter
//     inbound_counter ++;
//   }
//   else
//   {
//     if (inbound_counter != 0)
//     {
//       //! 2. Start incrementing an out-of-bounds counter
//       outbound_counter ++;
//     }
//   }

//   //! 3. Reset withinBoundsCounter if necessary
//   if (outbound_counter > 10)
//   {
//     ROS_INFO("##### Route %d: out of bounds, reset....", state);
//     inbound_counter  = 0;
//     outbound_counter = 0;
//   }

//   if (inbound_counter > 50)
//   {
//     ROS_INFO("##### Route %d start break....", state);
//     break_counter = 1;
//   }

// }

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if (!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(authority);

  if (!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }
  ROS_INFO("obtain control successful!");
  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if (query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

// void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
// {
//   current_atti = msg->quaternion;
// }

void flight_status_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr &msg)
{
  display_mode = msg->data;
}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (ros::Time::now() - start_time > ros::Duration(5))
  {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }

  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF && display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if (ros::Time::now() - start_time > ros::Duration(20))
  {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ((display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20))
  {
    ros::Duration(0.01).sleep();
    // ROS_INFO("altittude:%f", current_local_pos.z);
    //  if (current_local_pos.z > 0.2)
    //  {
    //    if (!takeoff_land(13u))
    //    {
    //      ROS_ERROR("Fail to exit takeoff!");
    //      return false;
    //    }
    //    else
    //    {
    //      ROS_INFO("Successful exit takeoff!");
    //      break;
    //    }
    //  }
    ros::spinOnce();
  }

  if (display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    ROS_INFO("display_mode = %d", display_mode);
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
