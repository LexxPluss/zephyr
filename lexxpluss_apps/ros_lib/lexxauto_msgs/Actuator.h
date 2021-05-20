#ifndef _ROS_SERVICE_Actuator_h
#define _ROS_SERVICE_Actuator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

static const char ACTUATOR[] = "lexxauto_msgs/Actuator";

  class ActuatorRequest : public ros::Msg
  {
    public:
      typedef bool _raise_type;
      _raise_type raise;
      typedef bool _lower_type;
      _lower_type lower;

    ActuatorRequest():
      raise(0),
      lower(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_raise;
      u_raise.real = this->raise;
      *(outbuffer + offset + 0) = (u_raise.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->raise);
      union {
        bool real;
        uint8_t base;
      } u_lower;
      u_lower.real = this->lower;
      *(outbuffer + offset + 0) = (u_lower.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lower);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_raise;
      u_raise.base = 0;
      u_raise.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->raise = u_raise.real;
      offset += sizeof(this->raise);
      union {
        bool real;
        uint8_t base;
      } u_lower;
      u_lower.base = 0;
      u_lower.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lower = u_lower.real;
      offset += sizeof(this->lower);
     return offset;
    }

    const char * getType(){ return ACTUATOR; };
    const char * getMD5(){ return "e1bc6bc1451d88b69d73fc6277062a34"; };

  };

  class ActuatorResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    ActuatorResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return ACTUATOR; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class Actuator {
    public:
    typedef ActuatorRequest Request;
    typedef ActuatorResponse Response;
  };

}
#endif
