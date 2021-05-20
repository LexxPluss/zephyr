#ifndef _ROS_lexxauto_msgs_safety_status_h
#define _ROS_lexxauto_msgs_safety_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

  class safety_status : public ros::Msg
  {
    public:
      typedef const char* _status_type;
      _status_type status;
      typedef const char* _front_type;
      _front_type front;
      typedef const char* _back_type;
      _back_type back;
      typedef const char* _side_type;
      _side_type side;

    safety_status():
      status(""),
      front(""),
      back(""),
      side("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_status = strlen(this->status);
      varToArr(outbuffer + offset, length_status);
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      uint32_t length_front = strlen(this->front);
      varToArr(outbuffer + offset, length_front);
      offset += 4;
      memcpy(outbuffer + offset, this->front, length_front);
      offset += length_front;
      uint32_t length_back = strlen(this->back);
      varToArr(outbuffer + offset, length_back);
      offset += 4;
      memcpy(outbuffer + offset, this->back, length_back);
      offset += length_back;
      uint32_t length_side = strlen(this->side);
      varToArr(outbuffer + offset, length_side);
      offset += 4;
      memcpy(outbuffer + offset, this->side, length_side);
      offset += length_side;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_status;
      arrToVar(length_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
      uint32_t length_front;
      arrToVar(length_front, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_front; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_front-1]=0;
      this->front = (char *)(inbuffer + offset-1);
      offset += length_front;
      uint32_t length_back;
      arrToVar(length_back, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_back; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_back-1]=0;
      this->back = (char *)(inbuffer + offset-1);
      offset += length_back;
      uint32_t length_side;
      arrToVar(length_side, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_side; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_side-1]=0;
      this->side = (char *)(inbuffer + offset-1);
      offset += length_side;
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/safety_status"; };
    const char * getMD5(){ return "3fe529ebcb63769b544ff1769a76eaf5"; };

  };

}
#endif
