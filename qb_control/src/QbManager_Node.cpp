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
 * \file      QbManager_Node.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/rate.h>

#include "QbManager_Class.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_qbManager");

    QbManager Obj;
    ros::Rate r(NODEFREQ_ARMINVKINE);
    ros::Time time_buf, time_cur;
    time_buf=ros::Time::now();
    
    Obj.initialize();
    
    while(ros::ok())
    {
        time_cur = ros::Time::now();
        Obj.dt= (time_cur-time_buf).toSec();
        time_buf=time_cur;

        Obj.run();
        // std::cout<<"h_space" << h_space << std::endl;
        ros::spinOnce();
        r.sleep();
            
    }// end while()

    Obj.terminate();

    return 0;
}