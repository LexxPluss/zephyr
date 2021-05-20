#ifndef _ROS_pf_pgv100_pgv_scan_data_h
#define _ROS_pf_pgv100_pgv_scan_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pf_pgv100
{

  class pgv_scan_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _angle_type;
      _angle_type angle;
      typedef float _x_pos_type;
      _x_pos_type x_pos;
      typedef float _y_pos_type;
      _y_pos_type y_pos;
      typedef const char* _direction_type;
      _direction_type direction;
      typedef uint8_t _color_lane_count_type;
      _color_lane_count_type color_lane_count;
      typedef uint8_t _no_color_lane_type;
      _no_color_lane_type no_color_lane;
      typedef uint8_t _no_pos_type;
      _no_pos_type no_pos;
      typedef uint8_t _tag_detected_type;
      _tag_detected_type tag_detected;
      typedef bool _control_code1_detected_type;
      _control_code1_detected_type control_code1_detected;
      typedef bool _control_code2_detected_type;
      _control_code2_detected_type control_code2_detected;

    pgv_scan_data():
      header(),
      angle(0),
      x_pos(0),
      y_pos(0),
      direction(""),
      color_lane_count(0),
      no_color_lane(0),
      no_pos(0),
      tag_detected(0),
      control_code1_detected(0),
      control_code2_detected(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        float real;
        uint32_t base;
      } u_x_pos;
      u_x_pos.real = this->x_pos;
      *(outbuffer + offset + 0) = (u_x_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_pos);
      union {
        float real;
        uint32_t base;
      } u_y_pos;
      u_y_pos.real = this->y_pos;
      *(outbuffer + offset + 0) = (u_y_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_pos);
      uint32_t length_direction = strlen(this->direction);
      varToArr(outbuffer + offset, length_direction);
      offset += 4;
      memcpy(outbuffer + offset, this->direction, length_direction);
      offset += length_direction;
      *(outbuffer + offset + 0) = (this->color_lane_count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->color_lane_count);
      *(outbuffer + offset + 0) = (this->no_color_lane >> (8 * 0)) & 0xFF;
      offset += sizeof(this->no_color_lane);
      *(outbuffer + offset + 0) = (this->no_pos >> (8 * 0)) & 0xFF;
      offset += sizeof(this->no_pos);
      *(outbuffer + offset + 0) = (this->tag_detected >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tag_detected);
      union {
        bool real;
        uint8_t base;
      } u_control_code1_detected;
      u_control_code1_detected.real = this->control_code1_detected;
      *(outbuffer + offset + 0) = (u_control_code1_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->control_code1_detected);
      union {
        bool real;
        uint8_t base;
      } u_control_code2_detected;
      u_control_code2_detected.real = this->control_code2_detected;
      *(outbuffer + offset + 0) = (u_control_code2_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->control_code2_detected);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        float real;
        uint32_t base;
      } u_x_pos;
      u_x_pos.base = 0;
      u_x_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_pos = u_x_pos.real;
      offset += sizeof(this->x_pos);
      union {
        float real;
        uint32_t base;
      } u_y_pos;
      u_y_pos.base = 0;
      u_y_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_pos = u_y_pos.real;
      offset += sizeof(this->y_pos);
      uint32_t length_direction;
      arrToVar(length_direction, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_direction; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_direction-1]=0;
      this->direction = (char *)(inbuffer + offset-1);
      offset += length_direction;
      this->color_lane_count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->color_lane_count);
      this->no_color_lane =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->no_color_lane);
      this->no_pos =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->no_pos);
      this->tag_detected =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->tag_detected);
      union {
        bool real;
        uint8_t base;
      } u_control_code1_detected;
      u_control_code1_detected.base = 0;
      u_control_code1_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->control_code1_detected = u_control_code1_detected.real;
      offset += sizeof(this->control_code1_detected);
      union {
        bool real;
        uint8_t base;
      } u_control_code2_detected;
      u_control_code2_detected.base = 0;
      u_control_code2_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->control_code2_detected = u_control_code2_detected.real;
      offset += sizeof(this->control_code2_detected);
     return offset;
    }

    const char * getType(){ return "pf_pgv100/pgv_scan_data"; };
    const char * getMD5(){ return "563f3fb578e3c9a6f8c9584e082c3bbe"; };

  };

}
#endif
