#ifndef _ROS_ros_myo_Vib2_h
#define _ROS_ros_myo_Vib2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_myo
{

  class Vib2 : public ros::Msg
  {
    public:
      typedef uint8_t _data1_type;
      _data1_type data1;
      typedef uint8_t _data2_type;
      _data2_type data2;
      typedef uint8_t _data3_type;
      _data3_type data3;
      typedef uint8_t _data4_type;
      _data4_type data4;
      typedef uint8_t _data5_type;
      _data5_type data5;
      typedef uint8_t _data6_type;
      _data6_type data6;
      typedef uint8_t _data7_type;
      _data7_type data7;
      typedef uint8_t _data8_type;
      _data8_type data8;
      typedef uint8_t _data9_type;
      _data9_type data9;
      typedef uint8_t _data10_type;
      _data10_type data10;
      typedef uint8_t _data11_type;
      _data11_type data11;
      typedef uint8_t _data12_type;
      _data12_type data12;
      typedef uint8_t _data13_type;
      _data13_type data13;
      typedef uint8_t _data14_type;
      _data14_type data14;
      typedef uint8_t _data15_type;
      _data15_type data15;
      typedef uint8_t _data16_type;
      _data16_type data16;
      typedef uint8_t _data17_type;
      _data17_type data17;
      typedef uint8_t _data18_type;
      _data18_type data18;

    Vib2():
      data1(0),
      data2(0),
      data3(0),
      data4(0),
      data5(0),
      data6(0),
      data7(0),
      data8(0),
      data9(0),
      data10(0),
      data11(0),
      data12(0),
      data13(0),
      data14(0),
      data15(0),
      data16(0),
      data17(0),
      data18(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data1);
      *(outbuffer + offset + 0) = (this->data2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data2);
      *(outbuffer + offset + 0) = (this->data3 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data3);
      *(outbuffer + offset + 0) = (this->data4 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data4);
      *(outbuffer + offset + 0) = (this->data5 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data5);
      *(outbuffer + offset + 0) = (this->data6 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data6);
      *(outbuffer + offset + 0) = (this->data7 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data7);
      *(outbuffer + offset + 0) = (this->data8 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data8);
      *(outbuffer + offset + 0) = (this->data9 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data9);
      *(outbuffer + offset + 0) = (this->data10 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data10);
      *(outbuffer + offset + 0) = (this->data11 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data11);
      *(outbuffer + offset + 0) = (this->data12 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data12);
      *(outbuffer + offset + 0) = (this->data13 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data13);
      *(outbuffer + offset + 0) = (this->data14 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data14);
      *(outbuffer + offset + 0) = (this->data15 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data15);
      *(outbuffer + offset + 0) = (this->data16 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data16);
      *(outbuffer + offset + 0) = (this->data17 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data17);
      *(outbuffer + offset + 0) = (this->data18 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data18);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->data1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data1);
      this->data2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data2);
      this->data3 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data3);
      this->data4 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data4);
      this->data5 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data5);
      this->data6 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data6);
      this->data7 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data7);
      this->data8 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data8);
      this->data9 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data9);
      this->data10 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data10);
      this->data11 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data11);
      this->data12 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data12);
      this->data13 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data13);
      this->data14 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data14);
      this->data15 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data15);
      this->data16 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data16);
      this->data17 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data17);
      this->data18 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data18);
     return offset;
    }

    const char * getType(){ return "ros_myo/Vib2"; };
    const char * getMD5(){ return "3b7d03162055fcd9984c64511791eded"; };

  };

}
#endif