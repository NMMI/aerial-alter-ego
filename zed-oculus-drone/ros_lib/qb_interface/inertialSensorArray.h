#ifndef _ROS_qb_interface_inertialSensorArray_h
#define _ROS_qb_interface_inertialSensorArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "qb_interface/inertialSensor.h"

namespace qb_interface
{

  class inertialSensorArray : public ros::Msg
  {
    public:
      uint32_t m_length;
      typedef qb_interface::inertialSensor _m_type;
      _m_type st_m;
      _m_type * m;

    inertialSensorArray():
      m_length(0), m(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->m_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->m_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->m_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->m_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m_length);
      for( uint32_t i = 0; i < m_length; i++){
      offset += this->m[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t m_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->m_length);
      if(m_lengthT > m_length)
        this->m = (qb_interface::inertialSensor*)realloc(this->m, m_lengthT * sizeof(qb_interface::inertialSensor));
      m_length = m_lengthT;
      for( uint32_t i = 0; i < m_length; i++){
      offset += this->st_m.deserialize(inbuffer + offset);
        memcpy( &(this->m[i]), &(this->st_m), sizeof(qb_interface::inertialSensor));
      }
     return offset;
    }

    const char * getType(){ return "qb_interface/inertialSensorArray"; };
    const char * getMD5(){ return "1b19aedfe4fe063f050add3e99a0693c"; };

  };

}
#endif