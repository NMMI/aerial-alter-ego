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

#include "VideoStream_Class.h"

int main(int argc, char *argv[])
{
  if (!ros::isInitialized())
    ros::init(argc, argv, "node_videoStream");

  VideoStream obj;
  ros::AsyncSpinner spinner(3);
  ros::Rate r(NODEFREQ_VIDEOSTREAM);
  spinner.start();

  while (ros::ok())
  {
    obj.run();
    r.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
