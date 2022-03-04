#ifndef _ROS_lwr_controllers_Stiffness_h
#define _ROS_lwr_controllers_Stiffness_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lwr_controllers
{

  class Stiffness : public ros::Msg
  {
    public:
      typedef double _x_type;
      _x_type x;
      typedef double _y_type;
      _y_type y;
      typedef double _z_type;
      _z_type z;
      typedef double _rx_type;
      _rx_type rx;
      typedef double _ry_type;
      _ry_type ry;
      typedef double _rz_type;
      _rz_type rz;

    Stiffness():
      x(0),
      y(0),
      z(0),
      rx(0),
      ry(0),
      rz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_z.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_z.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_z.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_z.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->z);
      union {
        double real;
        uint64_t base;
      } u_rx;
      u_rx.real = this->rx;
      *(outbuffer + offset + 0) = (u_rx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rx);
      union {
        double real;
        uint64_t base;
      } u_ry;
      u_ry.real = this->ry;
      *(outbuffer + offset + 0) = (u_ry.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ry.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ry.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ry.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ry.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ry.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ry.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ry.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ry);
      union {
        double real;
        uint64_t base;
      } u_rz;
      u_rz.real = this->rz;
      *(outbuffer + offset + 0) = (u_rz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rz.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rz.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rz.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rz.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rz.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_z.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        double real;
        uint64_t base;
      } u_rx;
      u_rx.base = 0;
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rx = u_rx.real;
      offset += sizeof(this->rx);
      union {
        double real;
        uint64_t base;
      } u_ry;
      u_ry.base = 0;
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ry.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ry = u_ry.real;
      offset += sizeof(this->ry);
      union {
        double real;
        uint64_t base;
      } u_rz;
      u_rz.base = 0;
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rz.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rz = u_rz.real;
      offset += sizeof(this->rz);
     return offset;
    }

    const char * getType(){ return "lwr_controllers/Stiffness"; };
    const char * getMD5(){ return "88d995302418aa13534aa14fc9aeed94"; };

  };

}
#endif