#ifndef _ROS_ros_myo_EmgArray_h
#define _ROS_ros_myo_EmgArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_myo
{

  class EmgArray : public ros::Msg
  {
    public:
      int16_t data[8];

    EmgArray():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "ros_myo/EmgArray"; };
    const char * getMD5(){ return "566f8e2295101ede678e362013a694af"; };

  };

}
#endif