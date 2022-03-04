#ifndef _ROS_lwr_controllers_CartesianImpedancePoint_h
#define _ROS_lwr_controllers_CartesianImpedancePoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "lwr_controllers/Stiffness.h"
#include "geometry_msgs/Wrench.h"

namespace lwr_controllers
{

  class CartesianImpedancePoint : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _x_FRI_type;
      _x_FRI_type x_FRI;
      typedef lwr_controllers::Stiffness _k_FRI_type;
      _k_FRI_type k_FRI;
      typedef lwr_controllers::Stiffness _d_FRI_type;
      _d_FRI_type d_FRI;
      typedef geometry_msgs::Wrench _f_FRI_type;
      _f_FRI_type f_FRI;

    CartesianImpedancePoint():
      header(),
      x_FRI(),
      k_FRI(),
      d_FRI(),
      f_FRI()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->x_FRI.serialize(outbuffer + offset);
      offset += this->k_FRI.serialize(outbuffer + offset);
      offset += this->d_FRI.serialize(outbuffer + offset);
      offset += this->f_FRI.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->x_FRI.deserialize(inbuffer + offset);
      offset += this->k_FRI.deserialize(inbuffer + offset);
      offset += this->d_FRI.deserialize(inbuffer + offset);
      offset += this->f_FRI.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "lwr_controllers/CartesianImpedancePoint"; };
    const char * getMD5(){ return "71b8c728a99ddb73ba603776d4733a5f"; };

  };

}
#endif