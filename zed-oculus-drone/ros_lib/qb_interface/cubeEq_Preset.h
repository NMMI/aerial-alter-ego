#ifndef _ROS_qb_interface_cubeEq_Preset_h
#define _ROS_qb_interface_cubeEq_Preset_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace qb_interface
{

  class cubeEq_Preset : public ros::Msg
  {
    public:
      uint32_t eq_length;
      typedef float _eq_type;
      _eq_type st_eq;
      _eq_type * eq;
      uint32_t preset_length;
      typedef float _preset_type;
      _preset_type st_preset;
      _preset_type * preset;

    cubeEq_Preset():
      eq_length(0), eq(NULL),
      preset_length(0), preset(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->eq_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->eq_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->eq_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->eq_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eq_length);
      for( uint32_t i = 0; i < eq_length; i++){
      union {
        float real;
        uint32_t base;
      } u_eqi;
      u_eqi.real = this->eq[i];
      *(outbuffer + offset + 0) = (u_eqi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_eqi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_eqi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_eqi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->eq[i]);
      }
      *(outbuffer + offset + 0) = (this->preset_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->preset_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->preset_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->preset_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->preset_length);
      for( uint32_t i = 0; i < preset_length; i++){
      union {
        float real;
        uint32_t base;
      } u_preseti;
      u_preseti.real = this->preset[i];
      *(outbuffer + offset + 0) = (u_preseti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_preseti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_preseti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_preseti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->preset[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t eq_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      eq_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      eq_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      eq_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->eq_length);
      if(eq_lengthT > eq_length)
        this->eq = (float*)realloc(this->eq, eq_lengthT * sizeof(float));
      eq_length = eq_lengthT;
      for( uint32_t i = 0; i < eq_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_eq;
      u_st_eq.base = 0;
      u_st_eq.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_eq.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_eq.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_eq.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_eq = u_st_eq.real;
      offset += sizeof(this->st_eq);
        memcpy( &(this->eq[i]), &(this->st_eq), sizeof(float));
      }
      uint32_t preset_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      preset_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      preset_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      preset_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->preset_length);
      if(preset_lengthT > preset_length)
        this->preset = (float*)realloc(this->preset, preset_lengthT * sizeof(float));
      preset_length = preset_lengthT;
      for( uint32_t i = 0; i < preset_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_preset;
      u_st_preset.base = 0;
      u_st_preset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_preset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_preset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_preset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_preset = u_st_preset.real;
      offset += sizeof(this->st_preset);
        memcpy( &(this->preset[i]), &(this->st_preset), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "qb_interface/cubeEq_Preset"; };
    const char * getMD5(){ return "70c9b6b7a00b0977bdadaa889ac3c537"; };

  };

}
#endif