#ifndef _ROS_lexxauto_msgs_execution_scenario_h
#define _ROS_lexxauto_msgs_execution_scenario_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

  class execution_scenario : public ros::Msg
  {
    public:
      typedef bool _execution_scenario_type;
      _execution_scenario_type execution_scenario;
      typedef const char* _agv_scenario_mode_type;
      _agv_scenario_mode_type agv_scenario_mode;

    execution_scenario():
      execution_scenario(0),
      agv_scenario_mode("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_execution_scenario;
      u_execution_scenario.real = this->execution_scenario;
      *(outbuffer + offset + 0) = (u_execution_scenario.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->execution_scenario);
      uint32_t length_agv_scenario_mode = strlen(this->agv_scenario_mode);
      varToArr(outbuffer + offset, length_agv_scenario_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->agv_scenario_mode, length_agv_scenario_mode);
      offset += length_agv_scenario_mode;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_execution_scenario;
      u_execution_scenario.base = 0;
      u_execution_scenario.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->execution_scenario = u_execution_scenario.real;
      offset += sizeof(this->execution_scenario);
      uint32_t length_agv_scenario_mode;
      arrToVar(length_agv_scenario_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_agv_scenario_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_agv_scenario_mode-1]=0;
      this->agv_scenario_mode = (char *)(inbuffer + offset-1);
      offset += length_agv_scenario_mode;
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/execution_scenario"; };
    const char * getMD5(){ return "b9fb4051f560772e93fcbe7b7c396666"; };

  };

}
#endif
