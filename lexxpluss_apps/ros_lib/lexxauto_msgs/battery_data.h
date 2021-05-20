#ifndef _ROS_lexxauto_msgs_battery_data_h
#define _ROS_lexxauto_msgs_battery_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lexxauto_msgs
{

  class battery_data : public ros::Msg
  {
    public:
      typedef float _total_voltage_type;
      _total_voltage_type total_voltage;
      typedef float _current_type;
      _current_type current;
      typedef float _residual_capacity_type;
      _residual_capacity_type residual_capacity;
      typedef float _nominal_capacity_type;
      _nominal_capacity_type nominal_capacity;
      typedef int32_t _cycle_life_type;
      _cycle_life_type cycle_life;
      typedef int32_t _product_date_year_type;
      _product_date_year_type product_date_year;
      typedef int32_t _product_date_month_type;
      _product_date_month_type product_date_month;
      typedef int32_t _product_date_day_type;
      _product_date_day_type product_date_day;
      typedef uint16_t _balance_status_type;
      _balance_status_type balance_status;
      typedef uint16_t _balance_status_high_type;
      _balance_status_high_type balance_status_high;
      typedef uint16_t _protection_status_type;
      _protection_status_type protection_status;
      typedef float _version_type;
      _version_type version;
      typedef int32_t _rsoc_type;
      _rsoc_type rsoc;
      typedef uint8_t _control_status_type;
      _control_status_type control_status;
      typedef int32_t _num_cell_type;
      _num_cell_type num_cell;
      typedef int32_t _num_ntc_type;
      _num_ntc_type num_ntc;
      uint32_t temp_length;
      typedef float _temp_type;
      _temp_type st_temp;
      _temp_type * temp;

    battery_data():
      total_voltage(0),
      current(0),
      residual_capacity(0),
      nominal_capacity(0),
      cycle_life(0),
      product_date_year(0),
      product_date_month(0),
      product_date_day(0),
      balance_status(0),
      balance_status_high(0),
      protection_status(0),
      version(0),
      rsoc(0),
      control_status(0),
      num_cell(0),
      num_ntc(0),
      temp_length(0), temp(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->total_voltage);
      offset += serializeAvrFloat64(outbuffer + offset, this->current);
      offset += serializeAvrFloat64(outbuffer + offset, this->residual_capacity);
      offset += serializeAvrFloat64(outbuffer + offset, this->nominal_capacity);
      union {
        int32_t real;
        uint32_t base;
      } u_cycle_life;
      u_cycle_life.real = this->cycle_life;
      *(outbuffer + offset + 0) = (u_cycle_life.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cycle_life.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cycle_life.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cycle_life.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cycle_life);
      union {
        int32_t real;
        uint32_t base;
      } u_product_date_year;
      u_product_date_year.real = this->product_date_year;
      *(outbuffer + offset + 0) = (u_product_date_year.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_product_date_year.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_product_date_year.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_product_date_year.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->product_date_year);
      union {
        int32_t real;
        uint32_t base;
      } u_product_date_month;
      u_product_date_month.real = this->product_date_month;
      *(outbuffer + offset + 0) = (u_product_date_month.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_product_date_month.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_product_date_month.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_product_date_month.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->product_date_month);
      union {
        int32_t real;
        uint32_t base;
      } u_product_date_day;
      u_product_date_day.real = this->product_date_day;
      *(outbuffer + offset + 0) = (u_product_date_day.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_product_date_day.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_product_date_day.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_product_date_day.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->product_date_day);
      *(outbuffer + offset + 0) = (this->balance_status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->balance_status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->balance_status);
      *(outbuffer + offset + 0) = (this->balance_status_high >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->balance_status_high >> (8 * 1)) & 0xFF;
      offset += sizeof(this->balance_status_high);
      *(outbuffer + offset + 0) = (this->protection_status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->protection_status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->protection_status);
      offset += serializeAvrFloat64(outbuffer + offset, this->version);
      union {
        int32_t real;
        uint32_t base;
      } u_rsoc;
      u_rsoc.real = this->rsoc;
      *(outbuffer + offset + 0) = (u_rsoc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rsoc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rsoc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rsoc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rsoc);
      *(outbuffer + offset + 0) = (this->control_status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->control_status);
      union {
        int32_t real;
        uint32_t base;
      } u_num_cell;
      u_num_cell.real = this->num_cell;
      *(outbuffer + offset + 0) = (u_num_cell.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_cell.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_cell.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_cell.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_cell);
      union {
        int32_t real;
        uint32_t base;
      } u_num_ntc;
      u_num_ntc.real = this->num_ntc;
      *(outbuffer + offset + 0) = (u_num_ntc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_ntc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_ntc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_ntc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_ntc);
      *(outbuffer + offset + 0) = (this->temp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->temp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->temp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->temp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_length);
      for( uint32_t i = 0; i < temp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->temp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->total_voltage));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->residual_capacity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nominal_capacity));
      union {
        int32_t real;
        uint32_t base;
      } u_cycle_life;
      u_cycle_life.base = 0;
      u_cycle_life.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cycle_life.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cycle_life.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cycle_life.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cycle_life = u_cycle_life.real;
      offset += sizeof(this->cycle_life);
      union {
        int32_t real;
        uint32_t base;
      } u_product_date_year;
      u_product_date_year.base = 0;
      u_product_date_year.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_product_date_year.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_product_date_year.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_product_date_year.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->product_date_year = u_product_date_year.real;
      offset += sizeof(this->product_date_year);
      union {
        int32_t real;
        uint32_t base;
      } u_product_date_month;
      u_product_date_month.base = 0;
      u_product_date_month.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_product_date_month.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_product_date_month.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_product_date_month.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->product_date_month = u_product_date_month.real;
      offset += sizeof(this->product_date_month);
      union {
        int32_t real;
        uint32_t base;
      } u_product_date_day;
      u_product_date_day.base = 0;
      u_product_date_day.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_product_date_day.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_product_date_day.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_product_date_day.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->product_date_day = u_product_date_day.real;
      offset += sizeof(this->product_date_day);
      this->balance_status =  ((uint16_t) (*(inbuffer + offset)));
      this->balance_status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->balance_status);
      this->balance_status_high =  ((uint16_t) (*(inbuffer + offset)));
      this->balance_status_high |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->balance_status_high);
      this->protection_status =  ((uint16_t) (*(inbuffer + offset)));
      this->protection_status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->protection_status);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->version));
      union {
        int32_t real;
        uint32_t base;
      } u_rsoc;
      u_rsoc.base = 0;
      u_rsoc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rsoc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rsoc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rsoc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rsoc = u_rsoc.real;
      offset += sizeof(this->rsoc);
      this->control_status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->control_status);
      union {
        int32_t real;
        uint32_t base;
      } u_num_cell;
      u_num_cell.base = 0;
      u_num_cell.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_cell.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_cell.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_cell.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_cell = u_num_cell.real;
      offset += sizeof(this->num_cell);
      union {
        int32_t real;
        uint32_t base;
      } u_num_ntc;
      u_num_ntc.base = 0;
      u_num_ntc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_ntc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_ntc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_ntc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_ntc = u_num_ntc.real;
      offset += sizeof(this->num_ntc);
      uint32_t temp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      temp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      temp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      temp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->temp_length);
      if(temp_lengthT > temp_length)
        this->temp = (float*)realloc(this->temp, temp_lengthT * sizeof(float));
      temp_length = temp_lengthT;
      for( uint32_t i = 0; i < temp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_temp));
        memcpy( &(this->temp[i]), &(this->st_temp), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "lexxauto_msgs/battery_data"; };
    const char * getMD5(){ return "c9e4e5ad7e388e0998125c69215128aa"; };

  };

}
#endif
