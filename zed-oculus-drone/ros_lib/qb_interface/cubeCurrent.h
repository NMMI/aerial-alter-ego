#ifndef _ROS_qb_interface_cubeCurrent_h
#define _ROS_qb_interface_cubeCurrent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_interface
{

  class cubeCurrent : public ros::Msg
  {
    public:
      uint32_t current_m1_length;
      typedef int32_t _current_m1_type;
      _current_m1_type st_current_m1;
      _current_m1_type * current_m1;
      uint32_t current_m2_length;
      typedef int32_t _current_m2_type;
      _current_m2_type st_current_m2;
      _current_m2_type * current_m2;

    cubeCurrent():
      current_m1_length(0), current_m1(NULL),
      current_m2_length(0), current_m2(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->current_m1_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_m1_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_m1_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_m1_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_m1_length);
      for( uint32_t i = 0; i < current_m1_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_current_m1i;
      u_current_m1i.real = this->current_m1[i];
      *(outbuffer + offset + 0) = (u_current_m1i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_m1i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_m1i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_m1i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_m1[i]);
      }
      *(outbuffer + offset + 0) = (this->current_m2_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_m2_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_m2_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_m2_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_m2_length);
      for( uint32_t i = 0; i < current_m2_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_current_m2i;
      u_current_m2i.real = this->current_m2[i];
      *(outbuffer + offset + 0) = (u_current_m2i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_m2i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_m2i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_m2i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_m2[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t current_m1_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      current_m1_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      current_m1_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      current_m1_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->current_m1_length);
      if(current_m1_lengthT > current_m1_length)
        this->current_m1 = (int32_t*)realloc(this->current_m1, current_m1_lengthT * sizeof(int32_t));
      current_m1_length = current_m1_lengthT;
      for( uint32_t i = 0; i < current_m1_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_current_m1;
      u_st_current_m1.base = 0;
      u_st_current_m1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_current_m1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_current_m1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_current_m1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_current_m1 = u_st_current_m1.real;
      offset += sizeof(this->st_current_m1);
        memcpy( &(this->current_m1[i]), &(this->st_current_m1), sizeof(int32_t));
      }
      uint32_t current_m2_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      current_m2_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      current_m2_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      current_m2_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->current_m2_length);
      if(current_m2_lengthT > current_m2_length)
        this->current_m2 = (int32_t*)realloc(this->current_m2, current_m2_lengthT * sizeof(int32_t));
      current_m2_length = current_m2_lengthT;
      for( uint32_t i = 0; i < current_m2_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_current_m2;
      u_st_current_m2.base = 0;
      u_st_current_m2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_current_m2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_current_m2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_current_m2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_current_m2 = u_st_current_m2.real;
      offset += sizeof(this->st_current_m2);
        memcpy( &(this->current_m2[i]), &(this->st_current_m2), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "qb_interface/cubeCurrent"; };
    const char * getMD5(){ return "2eef53e05b04751a24e5c83cddf11cef"; };

  };

}
#endif