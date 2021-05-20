#ifndef _ROS_SERVICE_Sounds_h
#define _ROS_SERVICE_Sounds_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

static const char SOUNDS[] = "lexxauto_msgs/Sounds";

  class SoundsRequest : public ros::Msg
  {
    public:
      typedef const char* _pattern_type;
      _pattern_type pattern;

    SoundsRequest():
      pattern("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_pattern = strlen(this->pattern);
      varToArr(outbuffer + offset, length_pattern);
      offset += 4;
      memcpy(outbuffer + offset, this->pattern, length_pattern);
      offset += length_pattern;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_pattern;
      arrToVar(length_pattern, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_pattern; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_pattern-1]=0;
      this->pattern = (char *)(inbuffer + offset-1);
      offset += length_pattern;
     return offset;
    }

    const char * getType(){ return SOUNDS; };
    const char * getMD5(){ return "130674745f136c4f7050dd52353c9bf6"; };

  };

  class SoundsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SoundsResponse():
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

    const char * getType(){ return SOUNDS; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class Sounds {
    public:
    typedef SoundsRequest Request;
    typedef SoundsResponse Response;
  };

}
#endif
