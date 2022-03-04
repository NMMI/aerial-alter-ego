#ifndef _ROS_lwr_controllers_MultiPriorityTask_h
#define _ROS_lwr_controllers_MultiPriorityTask_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lwr_controllers
{

  class MultiPriorityTask : public ros::Msg
  {
    public:
      uint32_t links_length;
      typedef int32_t _links_type;
      _links_type st_links;
      _links_type * links;
      uint32_t tasks_length;
      typedef double _tasks_type;
      _tasks_type st_tasks;
      _tasks_type * tasks;

    MultiPriorityTask():
      links_length(0), links(NULL),
      tasks_length(0), tasks(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->links_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->links_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->links_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->links_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->links_length);
      for( uint32_t i = 0; i < links_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_linksi;
      u_linksi.real = this->links[i];
      *(outbuffer + offset + 0) = (u_linksi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linksi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linksi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linksi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->links[i]);
      }
      *(outbuffer + offset + 0) = (this->tasks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tasks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tasks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tasks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tasks_length);
      for( uint32_t i = 0; i < tasks_length; i++){
      union {
        double real;
        uint64_t base;
      } u_tasksi;
      u_tasksi.real = this->tasks[i];
      *(outbuffer + offset + 0) = (u_tasksi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tasksi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tasksi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tasksi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tasksi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tasksi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tasksi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tasksi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tasks[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t links_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      links_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      links_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      links_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->links_length);
      if(links_lengthT > links_length)
        this->links = (int32_t*)realloc(this->links, links_lengthT * sizeof(int32_t));
      links_length = links_lengthT;
      for( uint32_t i = 0; i < links_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_links;
      u_st_links.base = 0;
      u_st_links.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_links.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_links.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_links.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_links = u_st_links.real;
      offset += sizeof(this->st_links);
        memcpy( &(this->links[i]), &(this->st_links), sizeof(int32_t));
      }
      uint32_t tasks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tasks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tasks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tasks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tasks_length);
      if(tasks_lengthT > tasks_length)
        this->tasks = (double*)realloc(this->tasks, tasks_lengthT * sizeof(double));
      tasks_length = tasks_lengthT;
      for( uint32_t i = 0; i < tasks_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_tasks;
      u_st_tasks.base = 0;
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_tasks.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_tasks = u_st_tasks.real;
      offset += sizeof(this->st_tasks);
        memcpy( &(this->tasks[i]), &(this->st_tasks), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "lwr_controllers/MultiPriorityTask"; };
    const char * getMD5(){ return "42e011fbf13e33d6ed6e65ac7a0bdf63"; };

  };

}
#endif