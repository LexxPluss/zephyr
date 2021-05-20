#ifndef _ROS_SERVICE_MakeOrder_h
#define _ROS_SERVICE_MakeOrder_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "lexxauto_msgs/Mode.h"

namespace lexxauto_msgs
{

static const char MAKEORDER[] = "lexxauto_msgs/MakeOrder";

  class MakeOrderRequest : public ros::Msg
  {
    public:
      typedef int32_t _modes_count_type;
      _modes_count_type modes_count;
      uint32_t modes_length;
      typedef lexxauto_msgs::Mode _modes_type;
      _modes_type st_modes;
      _modes_type * modes;

    MakeOrderRequest():
      modes_count(0),
      modes_length(0), modes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_modes_count;
      u_modes_count.real = this->modes_count;
      *(outbuffer + offset + 0) = (u_modes_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_modes_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_modes_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_modes_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->modes_count);
      *(outbuffer + offset + 0) = (this->modes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->modes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->modes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->modes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->modes_length);
      for( uint32_t i = 0; i < modes_length; i++){
      offset += this->modes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_modes_count;
      u_modes_count.base = 0;
      u_modes_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_modes_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_modes_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_modes_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->modes_count = u_modes_count.real;
      offset += sizeof(this->modes_count);
      uint32_t modes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->modes_length);
      if(modes_lengthT > modes_length)
        this->modes = (lexxauto_msgs::Mode*)realloc(this->modes, modes_lengthT * sizeof(lexxauto_msgs::Mode));
      modes_length = modes_lengthT;
      for( uint32_t i = 0; i < modes_length; i++){
      offset += this->st_modes.deserialize(inbuffer + offset);
        memcpy( &(this->modes[i]), &(this->st_modes), sizeof(lexxauto_msgs::Mode));
      }
     return offset;
    }

    const char * getType(){ return MAKEORDER; };
    const char * getMD5(){ return "51fca2d5abe487fccd839a838189b9ba"; };

  };

  class MakeOrderResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    MakeOrderResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return MAKEORDER; };
    const char * getMD5(){ return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class MakeOrder {
    public:
    typedef MakeOrderRequest Request;
    typedef MakeOrderResponse Response;
  };

}
#endif
