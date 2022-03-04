#ifndef _ROS_lwr_controllers_ArmState_h
#define _ROS_lwr_controllers_ArmState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Wrench.h"

namespace lwr_controllers
{

  class ArmState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;
      uint32_t est_ext_torques_length;
      typedef float _est_ext_torques_type;
      _est_ext_torques_type st_est_ext_torques;
      _est_ext_torques_type * est_ext_torques;
      typedef geometry_msgs::Wrench _est_ee_wrench_type;
      _est_ee_wrench_type est_ee_wrench;
      typedef geometry_msgs::Wrench _est_ee_wrench_base_type;
      _est_ee_wrench_base_type est_ee_wrench_base;

    ArmState():
      header(),
      joint_name_length(0), joint_name(NULL),
      est_ext_torques_length(0), est_ext_torques(NULL),
      est_ee_wrench(),
      est_ee_wrench_base()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_name_length);
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_joint_namei = strlen(this->joint_name[i]);
      varToArr(outbuffer + offset, length_joint_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name[i], length_joint_namei);
      offset += length_joint_namei;
      }
      *(outbuffer + offset + 0) = (this->est_ext_torques_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->est_ext_torques_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->est_ext_torques_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->est_ext_torques_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->est_ext_torques_length);
      for( uint32_t i = 0; i < est_ext_torques_length; i++){
      union {
        float real;
        uint32_t base;
      } u_est_ext_torquesi;
      u_est_ext_torquesi.real = this->est_ext_torques[i];
      *(outbuffer + offset + 0) = (u_est_ext_torquesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_est_ext_torquesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_est_ext_torquesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_est_ext_torquesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->est_ext_torques[i]);
      }
      offset += this->est_ee_wrench.serialize(outbuffer + offset);
      offset += this->est_ee_wrench_base.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t joint_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_name_length);
      if(joint_name_lengthT > joint_name_length)
        this->joint_name = (char**)realloc(this->joint_name, joint_name_lengthT * sizeof(char*));
      joint_name_length = joint_name_lengthT;
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_st_joint_name;
      arrToVar(length_st_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_name-1]=0;
      this->st_joint_name = (char *)(inbuffer + offset-1);
      offset += length_st_joint_name;
        memcpy( &(this->joint_name[i]), &(this->st_joint_name), sizeof(char*));
      }
      uint32_t est_ext_torques_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      est_ext_torques_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      est_ext_torques_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      est_ext_torques_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->est_ext_torques_length);
      if(est_ext_torques_lengthT > est_ext_torques_length)
        this->est_ext_torques = (float*)realloc(this->est_ext_torques, est_ext_torques_lengthT * sizeof(float));
      est_ext_torques_length = est_ext_torques_lengthT;
      for( uint32_t i = 0; i < est_ext_torques_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_est_ext_torques;
      u_st_est_ext_torques.base = 0;
      u_st_est_ext_torques.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_est_ext_torques.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_est_ext_torques.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_est_ext_torques.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_est_ext_torques = u_st_est_ext_torques.real;
      offset += sizeof(this->st_est_ext_torques);
        memcpy( &(this->est_ext_torques[i]), &(this->st_est_ext_torques), sizeof(float));
      }
      offset += this->est_ee_wrench.deserialize(inbuffer + offset);
      offset += this->est_ee_wrench_base.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "lwr_controllers/ArmState"; };
    const char * getMD5(){ return "1ec4005eded14deb32698c63aa479d03"; };

  };

}
#endif