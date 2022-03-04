/*
 * Copyright (C) 2017 Walk-Man
 * Copyright (c) 2022, NMMI
 * Author: Alessandro Settimi, Gianluca Lentini, Danilo Caporale
 * email: ale.settimi@gmail.com, danilo.caporale@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <udp_interface/udp_client.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#define NODEFREQ_VIDEOSTREAM ((double)15.0)

#define IP "169.254.250.197"
#define PORT 2302

#define ZEDWIDTH (int)1280
#define ZEDHEIGHT (int)720

#define RESIZE_FRAME_HEIGHT 394 //...562
#define RESIZE_FRAME_WIDTH 700  //...1000
#define PACK_SIZE 8100          // udp pack size; note that OSX limits < 8100 bytes
#define ENCODE_QUALITY 80

class VideoStream
{
public:
  VideoStream();
  void run();

private:
  udp_client UDPsender;

  ros::NodeHandle nodeHandler;

  ros::Subscriber subHandler_video_right, subHandler_video_left;

  cv::Mat frame;
  cv::Mat left;
  cv::Mat right;
  cv::Mat send;
  std::vector<int> compression_params;
  std::vector<uchar> encoded;

  std::mutex mutex_video_right, mutex_video_left;
  bool flag_imageReady_right, flag_imageReady_left;
  sensor_msgs::ImageConstPtr image_raw_right, image_raw_left;
  void getVideoCallback_right(const sensor_msgs::Image &msg);
  void getVideoCallback_left(const sensor_msgs::Image &msg);
};