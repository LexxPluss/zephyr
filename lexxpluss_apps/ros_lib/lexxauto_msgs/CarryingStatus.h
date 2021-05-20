#ifndef _ROS_lexxauto_msgs_CarryingStatus_h
#define _ROS_lexxauto_msgs_CarryingStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

  class CarryingStatus : public ros::Msg
  {
    public:
      typedef const char* _carrying_object_type;
      _carrying_object_type carrying_object;
      typedef bool _front_blind_type;
      _front_blind_type front_blind;
      typedef bool _back_blind_type;
      _back_blind_type back_blind;
      typedef bool _side_blind_type;
      _side_blind_type side_blind;

    CarryingStatus():
      carrying_object(""),
      front_blind(0),
      back_blind(0),
      side_blind(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_carrying_object = strlen(this->carrying_object);
      varToArr(outbuffer + offset, length_carrying_object);
      offset += 4;
      memcpy(outbuffer + offset, this->carrying_object, length_carrying_object);
      offset += length_carrying_object;
      union {
        bool real;
        uint8_t base;
      } u_front_blind;
      u_front_blind.real = this->front_blind;
      *(outbuffer + offset + 0) = (u_front_blind.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->front_blind);
      union {
        bool real;
        uint8_t base;
      } u_back_blind;
      u_back_blind.real = this->back_blind;
      *(outbuffer + offset + 0) = (u_back_blind.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->back_blind);
      union {
        bool real;
        uint8_t base;
      } u_side_blind;
      u_side_blind.real = this->side_blind;
      *(outbuffer + offset + 0) = (u_side_blind.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->side_blind);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_carrying_object;
      arrToVar(length_carrying_object, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_carrying_object; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_carrying_object-1]=0;
      this->carrying_object = (char *)(inbuffer + offset-1);
      offset += length_carrying_object;
      union {
        bool real;
        uint8_t base;
      } u_front_blind;
      u_front_blind.base = 0;
      u_front_blind.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->front_blind = u_front_blind.real;
      offset += sizeof(this->front_blind);
      union {
        bool real;
        uint8_t base;
      } u_back_blind;
      u_back_blind.base = 0;
      u_back_blind.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->back_blind = u_back_blind.real;
      offset += sizeof(this->back_blind);
      union {
        bool real;
        uint8_t base;
      } u_side_blind;
      u_side_blind.base = 0;
      u_side_blind.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->side_blind = u_side_blind.real;
      offset += sizeof(this->side_blind);
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/CarryingStatus"; };
    const char * getMD5(){ return "5a3ee7b1b017602e3da500953dbe602a"; };

  };

}
#endif
