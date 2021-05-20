#ifndef _ROS_pf_pgv100_pgv_dir_msg_h
#define _ROS_pf_pgv100_pgv_dir_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pf_pgv100
{

  class pgv_dir_msg : public ros::Msg
  {
    public:
      typedef uint8_t _dir_command_type;
      _dir_command_type dir_command;

    pgv_dir_msg():
      dir_command(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dir_command >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dir_command);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->dir_command =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dir_command);
     return offset;
    }

    const char * getType(){ return "pf_pgv100/pgv_dir_msg"; };
    const char * getMD5(){ return "be420e78d04d79ee4d8fd23e20966af0"; };

  };

}
#endif
