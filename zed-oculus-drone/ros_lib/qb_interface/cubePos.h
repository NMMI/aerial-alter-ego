#ifndef _ROS_qb_interface_cubePos_h
#define _ROS_qb_interface_cubePos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_interface
{

  class cubePos : public ros::Msg
  {
    public:
      uint32_t p_1_length;
      typedef float _p_1_type;
      _p_1_type st_p_1;
      _p_1_type * p_1;
      uint32_t p_2_length;
      typedef float _p_2_type;
      _p_2_type st_p_2;
      _p_2_type * p_2;
      uint32_t p_L_length;
      typedef float _p_L_type;
      _p_L_type st_p_L;
      _p_L_type * p_L;

    cubePos():
      p_1_length(0), p_1(NULL),
      p_2_length(0), p_2(NULL),
      p_L_length(0), p_L(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->p_1_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_1_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->p_1_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->p_1_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_1_length);
      for( uint32_t i = 0; i < p_1_length; i++){
      union {
        float real;
        uint32_t base;
      } u_p_1i;
      u_p_1i.real = this->p_1[i];
      *(outbuffer + offset + 0) = (u_p_1i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_1i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_1i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_1i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_1[i]);
      }
      *(outbuffer + offset + 0) = (this->p_2_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_2_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->p_2_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->p_2_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_2_length);
      for( uint32_t i = 0; i < p_2_length; i++){
      union {
        float real;
        uint32_t base;
      } u_p_2i;
      u_p_2i.real = this->p_2[i];
      *(outbuffer + offset + 0) = (u_p_2i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_2i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_2i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_2i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_2[i]);
      }
      *(outbuffer + offset + 0) = (this->p_L_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->p_L_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->p_L_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->p_L_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_L_length);
      for( uint32_t i = 0; i < p_L_length; i++){
      union {
        float real;
        uint32_t base;
      } u_p_Li;
      u_p_Li.real = this->p_L[i];
      *(outbuffer + offset + 0) = (u_p_Li.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_Li.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_Li.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_Li.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p_L[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t p_1_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      p_1_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      p_1_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      p_1_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->p_1_length);
      if(p_1_lengthT > p_1_length)
        this->p_1 = (float*)realloc(this->p_1, p_1_lengthT * sizeof(float));
      p_1_length = p_1_lengthT;
      for( uint32_t i = 0; i < p_1_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_p_1;
      u_st_p_1.base = 0;
      u_st_p_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_p_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_p_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_p_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_p_1 = u_st_p_1.real;
      offset += sizeof(this->st_p_1);
        memcpy( &(this->p_1[i]), &(this->st_p_1), sizeof(float));
      }
      uint32_t p_2_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      p_2_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      p_2_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      p_2_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->p_2_length);
      if(p_2_lengthT > p_2_length)
        this->p_2 = (float*)realloc(this->p_2, p_2_lengthT * sizeof(float));
      p_2_length = p_2_lengthT;
      for( uint32_t i = 0; i < p_2_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_p_2;
      u_st_p_2.base = 0;
      u_st_p_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_p_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_p_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_p_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_p_2 = u_st_p_2.real;
      offset += sizeof(this->st_p_2);
        memcpy( &(this->p_2[i]), &(this->st_p_2), sizeof(float));
      }
      uint32_t p_L_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      p_L_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      p_L_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      p_L_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->p_L_length);
      if(p_L_lengthT > p_L_length)
        this->p_L = (float*)realloc(this->p_L, p_L_lengthT * sizeof(float));
      p_L_length = p_L_lengthT;
      for( uint32_t i = 0; i < p_L_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_p_L;
      u_st_p_L.base = 0;
      u_st_p_L.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_p_L.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_p_L.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_p_L.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_p_L = u_st_p_L.real;
      offset += sizeof(this->st_p_L);
        memcpy( &(this->p_L[i]), &(this->st_p_L), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "qb_interface/cubePos"; };
    const char * getMD5(){ return "a02b665022ec76721e512ea8351a945c"; };

  };

}
#endif