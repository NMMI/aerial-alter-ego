#ifndef _ROS_ros_myo_MyoArm_h
#define _ROS_ros_myo_MyoArm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_myo
{

  class MyoArm : public ros::Msg
  {
    public:
      typedef uint8_t _arm_type;
      _arm_type arm;
      typedef uint8_t _xdir_type;
      _xdir_type xdir;

    MyoArm():
      arm(0),
      xdir(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->arm >> (8 * 0)) & 0xFF;
      offset += sizeof(this->arm);
      *(outbuffer + offset + 0) = (this->xdir >> (8 * 0)) & 0xFF;
      offset += sizeof(this->xdir);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->arm =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->arm);
      this->xdir =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->xdir);
     return offset;
    }

    const char * getType(){ return "ros_myo/MyoArm"; };
    const char * getMD5(){ return "01e586809ffd60cf3f07c41869248647"; };

  };

}
#endif