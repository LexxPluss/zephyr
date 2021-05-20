#ifndef _ROS_lexxauto_msgs_goal_pose_list_message_h
#define _ROS_lexxauto_msgs_goal_pose_list_message_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

  class goal_pose_list_message : public ros::Msg
  {
    public:
      uint32_t area_name_length;
      typedef char* _area_name_type;
      _area_name_type st_area_name;
      _area_name_type * area_name;
      uint32_t x_length;
      typedef float _x_type;
      _x_type st_x;
      _x_type * x;
      uint32_t y_length;
      typedef float _y_type;
      _y_type st_y;
      _y_type * y;
      uint32_t z_length;
      typedef float _z_type;
      _z_type st_z;
      _z_type * z;
      uint32_t w_length;
      typedef float _w_type;
      _w_type st_w;
      _w_type * w;

    goal_pose_list_message():
      area_name_length(0), area_name(NULL),
      x_length(0), x(NULL),
      y_length(0), y(NULL),
      z_length(0), z(NULL),
      w_length(0), w(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->area_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->area_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->area_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->area_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->area_name_length);
      for( uint32_t i = 0; i < area_name_length; i++){
      uint32_t length_area_namei = strlen(this->area_name[i]);
      varToArr(outbuffer + offset, length_area_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->area_name[i], length_area_namei);
      offset += length_area_namei;
      }
      *(outbuffer + offset + 0) = (this->x_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_length);
      for( uint32_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_xi;
      u_xi.real = this->x[i];
      *(outbuffer + offset + 0) = (u_xi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x[i]);
      }
      *(outbuffer + offset + 0) = (this->y_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_length);
      for( uint32_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_yi;
      u_yi.real = this->y[i];
      *(outbuffer + offset + 0) = (u_yi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y[i]);
      }
      *(outbuffer + offset + 0) = (this->z_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->z_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->z_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->z_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z_length);
      for( uint32_t i = 0; i < z_length; i++){
      union {
        float real;
        uint32_t base;
      } u_zi;
      u_zi.real = this->z[i];
      *(outbuffer + offset + 0) = (u_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z[i]);
      }
      *(outbuffer + offset + 0) = (this->w_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->w_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->w_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->w_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w_length);
      for( uint32_t i = 0; i < w_length; i++){
      union {
        float real;
        uint32_t base;
      } u_wi;
      u_wi.real = this->w[i];
      *(outbuffer + offset + 0) = (u_wi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->w[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t area_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      area_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      area_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      area_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->area_name_length);
      if(area_name_lengthT > area_name_length)
        this->area_name = (char**)realloc(this->area_name, area_name_lengthT * sizeof(char*));
      area_name_length = area_name_lengthT;
      for( uint32_t i = 0; i < area_name_length; i++){
      uint32_t length_st_area_name;
      arrToVar(length_st_area_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_area_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_area_name-1]=0;
      this->st_area_name = (char *)(inbuffer + offset-1);
      offset += length_st_area_name;
        memcpy( &(this->area_name[i]), &(this->st_area_name), sizeof(char*));
      }
      uint32_t x_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      x_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->x_length);
      if(x_lengthT > x_length)
        this->x = (float*)realloc(this->x, x_lengthT * sizeof(float));
      x_length = x_lengthT;
      for( uint32_t i = 0; i < x_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_x;
      u_st_x.base = 0;
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_x = u_st_x.real;
      offset += sizeof(this->st_x);
        memcpy( &(this->x[i]), &(this->st_x), sizeof(float));
      }
      uint32_t y_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      y_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->y_length);
      if(y_lengthT > y_length)
        this->y = (float*)realloc(this->y, y_lengthT * sizeof(float));
      y_length = y_lengthT;
      for( uint32_t i = 0; i < y_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_y;
      u_st_y.base = 0;
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_y = u_st_y.real;
      offset += sizeof(this->st_y);
        memcpy( &(this->y[i]), &(this->st_y), sizeof(float));
      }
      uint32_t z_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      z_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->z_length);
      if(z_lengthT > z_length)
        this->z = (float*)realloc(this->z, z_lengthT * sizeof(float));
      z_length = z_lengthT;
      for( uint32_t i = 0; i < z_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_z;
      u_st_z.base = 0;
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_z = u_st_z.real;
      offset += sizeof(this->st_z);
        memcpy( &(this->z[i]), &(this->st_z), sizeof(float));
      }
      uint32_t w_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      w_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      w_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      w_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->w_length);
      if(w_lengthT > w_length)
        this->w = (float*)realloc(this->w, w_lengthT * sizeof(float));
      w_length = w_lengthT;
      for( uint32_t i = 0; i < w_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_w;
      u_st_w.base = 0;
      u_st_w.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_w.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_w.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_w.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_w = u_st_w.real;
      offset += sizeof(this->st_w);
        memcpy( &(this->w[i]), &(this->st_w), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/goal_pose_list_message"; };
    const char * getMD5(){ return "a5475a7ae2fd9ac126709fe22bf98032"; };

  };

}
#endif
