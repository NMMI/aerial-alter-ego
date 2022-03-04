#ifndef _ROS_qb_interface_handPos_h
#define _ROS_qb_interface_handPos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_interface
{

  class handPos : public ros::Msg
  {
    public:
      uint32_t closure_length;
      typedef float _closure_type;
      _closure_type st_closure;
      _closure_type * closure;

    handPos():
      closure_length(0), closure(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->closure_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->closure_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->closure_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->closure_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->closure_length);
      for( uint32_t i = 0; i < closure_length; i++){
      union {
        float real;
        uint32_t base;
      } u_closurei;
      u_closurei.real = this->closure[i];
      *(outbuffer + offset + 0) = (u_closurei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_closurei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_closurei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_closurei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->closure[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t closure_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      closure_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      closure_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      closure_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->closure_length);
      if(closure_lengthT > closure_length)
        this->closure = (float*)realloc(this->closure, closure_lengthT * sizeof(float));
      closure_length = closure_lengthT;
      for( uint32_t i = 0; i < closure_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_closure;
      u_st_closure.base = 0;
      u_st_closure.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_closure.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_closure.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_closure.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_closure = u_st_closure.real;
      offset += sizeof(this->st_closure);
        memcpy( &(this->closure[i]), &(this->st_closure), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "qb_interface/handPos"; };
    const char * getMD5(){ return "3d3fcdda7faa52ddfc5831b96497f9ca"; };

  };

}
#endif