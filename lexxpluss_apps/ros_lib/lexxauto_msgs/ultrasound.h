#ifndef _ROS_lexxauto_msgs_ultrasound_h
#define _ROS_lexxauto_msgs_ultrasound_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

  class ultrasound : public ros::Msg
  {
    public:
      typedef uint16_t _sensor0_type;
      _sensor0_type sensor0;
      typedef uint16_t _sensor1_type;
      _sensor1_type sensor1;
      typedef uint16_t _sensor2_type;
      _sensor2_type sensor2;
      typedef uint16_t _sensor3_type;
      _sensor3_type sensor3;

    ultrasound():
      sensor0(0),
      sensor1(0),
      sensor2(0),
      sensor3(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->sensor0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor0 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sensor0);
      *(outbuffer + offset + 0) = (this->sensor1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor1 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sensor1);
      *(outbuffer + offset + 0) = (this->sensor2 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor2 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sensor2);
      *(outbuffer + offset + 0) = (this->sensor3 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor3 >> (8 * 1)) & 0xFF;
      offset += sizeof(this->sensor3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->sensor0 =  ((uint16_t) (*(inbuffer + offset)));
      this->sensor0 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sensor0);
      this->sensor1 =  ((uint16_t) (*(inbuffer + offset)));
      this->sensor1 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sensor1);
      this->sensor2 =  ((uint16_t) (*(inbuffer + offset)));
      this->sensor2 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sensor2);
      this->sensor3 =  ((uint16_t) (*(inbuffer + offset)));
      this->sensor3 |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->sensor3);
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/ultrasound"; };
    const char * getMD5(){ return "41857dc070d1cca2686860bd82b722e5"; };

  };

}
#endif
