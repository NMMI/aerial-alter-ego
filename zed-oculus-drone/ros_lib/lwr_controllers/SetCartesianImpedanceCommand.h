#ifndef _ROS_SERVICE_SetCartesianImpedanceCommand_h
#define _ROS_SERVICE_SetCartesianImpedanceCommand_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "lwr_controllers/CartesianImpedancePoint.h"

namespace lwr_controllers
{

static const char SETCARTESIANIMPEDANCECOMMAND[] = "lwr_controllers/SetCartesianImpedanceCommand";

  class SetCartesianImpedanceCommandRequest : public ros::Msg
  {
    public:
      typedef lwr_controllers::CartesianImpedancePoint _command_type;
      _command_type command;

    SetCartesianImpedanceCommandRequest():
      command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETCARTESIANIMPEDANCECOMMAND; };
    const char * getMD5(){ return "cc36264319f44d204c2ace0744aa21ee"; };

  };

  class SetCartesianImpedanceCommandResponse : public ros::Msg
  {
    public:

    SetCartesianImpedanceCommandResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETCARTESIANIMPEDANCECOMMAND; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetCartesianImpedanceCommand {
    public:
    typedef SetCartesianImpedanceCommandRequest Request;
    typedef SetCartesianImpedanceCommandResponse Response;
  };

}
#endif
