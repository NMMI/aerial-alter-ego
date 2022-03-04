#ifndef _ROS_qb_interface_handCurrent_h
#define _ROS_qb_interface_handCurrent_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_interface
{

  class handCurrent : public ros::Msg
  {
    public:
      uint32_t current_length;
      typedef int32_t _current_type;
      _current_type st_current;
      _current_type * current;

    handCurrent():
      current_length(0), current(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->current_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_length);
      for( uint32_t i = 0; i < current_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_currenti;
      u_currenti.real = this->current[i];
      *(outbuffer + offset + 0) = (u_currenti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currenti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currenti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currenti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t current_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      current_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      current_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      current_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->current_length);
      if(current_lengthT > current_length)
        this->current = (int32_t*)realloc(this->current, current_lengthT * sizeof(int32_t));
      current_length = current_lengthT;
      for( uint32_t i = 0; i < current_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_current;
      u_st_current.base = 0;
      u_st_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_current = u_st_current.real;
      offset += sizeof(this->st_current);
        memcpy( &(this->current[i]), &(this->st_current), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "qb_interface/handCurrent"; };
    const char * getMD5(){ return "d976b449f93ed52a0958824b07438fab"; };

  };

}
#endif