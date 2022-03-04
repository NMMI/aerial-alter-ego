/*
 * Copyright (C) 2017 Walk-Man
 * Copyright (c) 2022, NMMI
 * 
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

/*
 * Modification NMMI 2022:
 * seperated into two files, VideoStream_Class and VideoStream_Node. Improve the image transfer stability
 */

#include "VideoStream_Class.h"

VideoStream::VideoStream() : UDPsender(IP, PORT)
{
  subHandler_video_right = nodeHandler.subscribe("/camera/right/image_raw", 1, &VideoStream::getVideoCallback_right, this);
  subHandler_video_left = nodeHandler.subscribe("/camera/left/image_raw", 1, &VideoStream::getVideoCallback_left, this);
  flag_imageReady_right = false;
  flag_imageReady_left = false;

  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(ENCODE_QUALITY);

  frame = cv::Mat(ZEDHEIGHT, ZEDWIDTH + ZEDWIDTH, CV_8UC3);
  left = cv::Mat(frame, cv::Rect(0, 0, ZEDWIDTH, ZEDHEIGHT));
  right = cv::Mat(frame, cv::Rect(ZEDWIDTH, 0, ZEDWIDTH, ZEDHEIGHT));
}
void VideoStream::run()
{

  std::lock_guard<std::mutex> lock1(mutex_video_right);
  std::lock_guard<std::mutex> lock2(mutex_video_left);

  if (flag_imageReady_right && flag_imageReady_left)
  {

    cv_bridge::toCvCopy(image_raw_right, sensor_msgs::image_encodings::BGR8)->image.copyTo(right);
    cv_bridge::toCvCopy(image_raw_left, sensor_msgs::image_encodings::BGR8)->image.copyTo(left);
    flag_imageReady_right = false;
    flag_imageReady_left = false;

    // cv::resize(frame, send, cv::Size(RESIZE_FRAME_WIDTH * 2, RESIZE_FRAME_HEIGHT));
    cv::imencode(".jpg", frame, encoded, compression_params);

    int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;
    int ibuf[1] = {total_pack};

    if (UDPsender.send((char *)(&ibuf[0]), sizeof(int)) == -1)
    {
      std::cout << "sendto() failed" << std::endl;
      return;
    }
    usleep(5000);
    // send image
    for (int i = 0; i < total_pack - 1; i++)
    {
      if (UDPsender.send((char *)(&encoded[i * PACK_SIZE]), PACK_SIZE) == -1)
      {
        std::cout << "sendto() failed" << std::endl;
        return;
      }
    }
    if (UDPsender.send((char *)(&encoded[(total_pack - 1) * PACK_SIZE]), encoded.size() % PACK_SIZE) == -1)
    {
      std::cout << "sendto(last) failed" << std::endl;
      return;
    }
  }
}

void VideoStream::getVideoCallback_right(const sensor_msgs::Image &msg)
{

  std::lock_guard<std::mutex> lock(mutex_video_right);

  flag_imageReady_right = true;
  image_raw_right.reset(new sensor_msgs::Image(msg));
}
void VideoStream::getVideoCallback_left(const sensor_msgs::Image &msg)
{

  std::lock_guard<std::mutex> lock(mutex_video_left);

  flag_imageReady_left = true;
  image_raw_left.reset(new sensor_msgs::Image(msg));
}