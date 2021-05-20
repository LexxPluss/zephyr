#ifndef _ROS_lexxauto_msgs_Mode_h
#define _ROS_lexxauto_msgs_Mode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "lexxauto_msgs/Dict.h"

namespace lexxauto_msgs
{

  class Mode : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef int32_t _params_count_type;
      _params_count_type params_count;
      uint32_t params_length;
      typedef lexxauto_msgs::Dict _params_type;
      _params_type st_params;
      _params_type * params;

    Mode():
      name(""),
      params_count(0),
      params_length(0), params(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      union {
        int32_t real;
        uint32_t base;
      } u_params_count;
      u_params_count.real = this->params_count;
      *(outbuffer + offset + 0) = (u_params_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_params_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_params_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_params_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params_count);
      *(outbuffer + offset + 0) = (this->params_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->params_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->params_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->params_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->params_length);
      for( uint32_t i = 0; i < params_length; i++){
      offset += this->params[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      union {
        int32_t real;
        uint32_t base;
      } u_params_count;
      u_params_count.base = 0;
      u_params_count.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_params_count.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_params_count.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_params_count.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->params_count = u_params_count.real;
      offset += sizeof(this->params_count);
      uint32_t params_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      params_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->params_length);
      if(params_lengthT > params_length)
        this->params = (lexxauto_msgs::Dict*)realloc(this->params, params_lengthT * sizeof(lexxauto_msgs::Dict));
      params_length = params_lengthT;
      for( uint32_t i = 0; i < params_length; i++){
      offset += this->st_params.deserialize(inbuffer + offset);
        memcpy( &(this->params[i]), &(this->st_params), sizeof(lexxauto_msgs::Dict));
      }
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/Mode"; };
    const char * getMD5(){ return "921c334871eab97e9422ba7ba93a127b"; };

  };

}
#endif
