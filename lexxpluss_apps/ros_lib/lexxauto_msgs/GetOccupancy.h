#ifndef _ROS_SERVICE_GetOccupancy_h
#define _ROS_SERVICE_GetOccupancy_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

static const char GETOCCUPANCY[] = "lexxauto_msgs/GetOccupancy";

  class GetOccupancyRequest : public ros::Msg
  {
    public:
      typedef const char* _area_name_type;
      _area_name_type area_name;

    GetOccupancyRequest():
      area_name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_area_name = strlen(this->area_name);
      varToArr(outbuffer + offset, length_area_name);
      offset += 4;
      memcpy(outbuffer + offset, this->area_name, length_area_name);
      offset += length_area_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_area_name;
      arrToVar(length_area_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_area_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_area_name-1]=0;
      this->area_name = (char *)(inbuffer + offset-1);
      offset += length_area_name;
     return offset;
    }

    const char * getType(){ return GETOCCUPANCY; };
    const char * getMD5(){ return "a2d7ccf8e5de29f456d5e1df798f949b"; };

  };

  class GetOccupancyResponse : public ros::Msg
  {
    public:
      typedef const char* _answer_type;
      _answer_type answer;

    GetOccupancyResponse():
      answer("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_answer = strlen(this->answer);
      varToArr(outbuffer + offset, length_answer);
      offset += 4;
      memcpy(outbuffer + offset, this->answer, length_answer);
      offset += length_answer;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_answer;
      arrToVar(length_answer, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_answer; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_answer-1]=0;
      this->answer = (char *)(inbuffer + offset-1);
      offset += length_answer;
     return offset;
    }

    const char * getType(){ return GETOCCUPANCY; };
    const char * getMD5(){ return "d7e708f879c94bb931716d8f4f130f30"; };

  };

  class GetOccupancy {
    public:
    typedef GetOccupancyRequest Request;
    typedef GetOccupancyResponse Response;
  };

}
#endif
