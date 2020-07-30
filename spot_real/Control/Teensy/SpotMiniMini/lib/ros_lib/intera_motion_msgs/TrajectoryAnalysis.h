#ifndef _ROS_intera_motion_msgs_TrajectoryAnalysis_h
#define _ROS_intera_motion_msgs_TrajectoryAnalysis_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_motion_msgs
{

  class TrajectoryAnalysis : public ros::Msg
  {
    public:
      typedef double _planned_duration_type;
      _planned_duration_type planned_duration;
      typedef double _measured_duration_type;
      _measured_duration_type measured_duration;
      uint32_t min_angle_command_length;
      typedef double _min_angle_command_type;
      _min_angle_command_type st_min_angle_command;
      _min_angle_command_type * min_angle_command;
      uint32_t max_angle_command_length;
      typedef double _max_angle_command_type;
      _max_angle_command_type st_max_angle_command;
      _max_angle_command_type * max_angle_command;
      uint32_t peak_speed_command_length;
      typedef double _peak_speed_command_type;
      _peak_speed_command_type st_peak_speed_command;
      _peak_speed_command_type * peak_speed_command;
      uint32_t peak_accel_command_length;
      typedef double _peak_accel_command_type;
      _peak_accel_command_type st_peak_accel_command;
      _peak_accel_command_type * peak_accel_command;
      uint32_t peak_jerk_command_length;
      typedef double _peak_jerk_command_type;
      _peak_jerk_command_type st_peak_jerk_command;
      _peak_jerk_command_type * peak_jerk_command;
      typedef double _min_time_rate_type;
      _min_time_rate_type min_time_rate;
      typedef double _max_time_rate_type;
      _max_time_rate_type max_time_rate;
      uint32_t max_position_error_length;
      typedef double _max_position_error_type;
      _max_position_error_type st_max_position_error;
      _max_position_error_type * max_position_error;
      uint32_t max_velocity_error_length;
      typedef double _max_velocity_error_type;
      _max_velocity_error_type st_max_velocity_error;
      _max_velocity_error_type * max_velocity_error;

    TrajectoryAnalysis():
      planned_duration(0),
      measured_duration(0),
      min_angle_command_length(0), min_angle_command(NULL),
      max_angle_command_length(0), max_angle_command(NULL),
      peak_speed_command_length(0), peak_speed_command(NULL),
      peak_accel_command_length(0), peak_accel_command(NULL),
      peak_jerk_command_length(0), peak_jerk_command(NULL),
      min_time_rate(0),
      max_time_rate(0),
      max_position_error_length(0), max_position_error(NULL),
      max_velocity_error_length(0), max_velocity_error(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_planned_duration;
      u_planned_duration.real = this->planned_duration;
      *(outbuffer + offset + 0) = (u_planned_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_planned_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_planned_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_planned_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_planned_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_planned_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_planned_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_planned_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->planned_duration);
      union {
        double real;
        uint64_t base;
      } u_measured_duration;
      u_measured_duration.real = this->measured_duration;
      *(outbuffer + offset + 0) = (u_measured_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_measured_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_measured_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_measured_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_measured_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_measured_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_measured_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_measured_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->measured_duration);
      *(outbuffer + offset + 0) = (this->min_angle_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->min_angle_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->min_angle_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->min_angle_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_angle_command_length);
      for( uint32_t i = 0; i < min_angle_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_min_angle_commandi;
      u_min_angle_commandi.real = this->min_angle_command[i];
      *(outbuffer + offset + 0) = (u_min_angle_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_angle_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_angle_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_angle_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_angle_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_angle_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_angle_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_angle_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_angle_command[i]);
      }
      *(outbuffer + offset + 0) = (this->max_angle_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_angle_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_angle_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_angle_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_angle_command_length);
      for( uint32_t i = 0; i < max_angle_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_max_angle_commandi;
      u_max_angle_commandi.real = this->max_angle_command[i];
      *(outbuffer + offset + 0) = (u_max_angle_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_angle_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_angle_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_angle_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_angle_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_angle_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_angle_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_angle_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_angle_command[i]);
      }
      *(outbuffer + offset + 0) = (this->peak_speed_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->peak_speed_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->peak_speed_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->peak_speed_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->peak_speed_command_length);
      for( uint32_t i = 0; i < peak_speed_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_peak_speed_commandi;
      u_peak_speed_commandi.real = this->peak_speed_command[i];
      *(outbuffer + offset + 0) = (u_peak_speed_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_peak_speed_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_peak_speed_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_peak_speed_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_peak_speed_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_peak_speed_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_peak_speed_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_peak_speed_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->peak_speed_command[i]);
      }
      *(outbuffer + offset + 0) = (this->peak_accel_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->peak_accel_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->peak_accel_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->peak_accel_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->peak_accel_command_length);
      for( uint32_t i = 0; i < peak_accel_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_peak_accel_commandi;
      u_peak_accel_commandi.real = this->peak_accel_command[i];
      *(outbuffer + offset + 0) = (u_peak_accel_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_peak_accel_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_peak_accel_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_peak_accel_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_peak_accel_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_peak_accel_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_peak_accel_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_peak_accel_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->peak_accel_command[i]);
      }
      *(outbuffer + offset + 0) = (this->peak_jerk_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->peak_jerk_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->peak_jerk_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->peak_jerk_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->peak_jerk_command_length);
      for( uint32_t i = 0; i < peak_jerk_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_peak_jerk_commandi;
      u_peak_jerk_commandi.real = this->peak_jerk_command[i];
      *(outbuffer + offset + 0) = (u_peak_jerk_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_peak_jerk_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_peak_jerk_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_peak_jerk_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_peak_jerk_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_peak_jerk_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_peak_jerk_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_peak_jerk_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->peak_jerk_command[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_min_time_rate;
      u_min_time_rate.real = this->min_time_rate;
      *(outbuffer + offset + 0) = (u_min_time_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_time_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_time_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_time_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_time_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_time_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_time_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_time_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_time_rate);
      union {
        double real;
        uint64_t base;
      } u_max_time_rate;
      u_max_time_rate.real = this->max_time_rate;
      *(outbuffer + offset + 0) = (u_max_time_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_time_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_time_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_time_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_time_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_time_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_time_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_time_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_time_rate);
      *(outbuffer + offset + 0) = (this->max_position_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_position_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_position_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_position_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_position_error_length);
      for( uint32_t i = 0; i < max_position_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_max_position_errori;
      u_max_position_errori.real = this->max_position_error[i];
      *(outbuffer + offset + 0) = (u_max_position_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_position_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_position_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_position_errori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_position_errori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_position_errori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_position_errori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_position_errori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_position_error[i]);
      }
      *(outbuffer + offset + 0) = (this->max_velocity_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_velocity_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_velocity_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_velocity_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_velocity_error_length);
      for( uint32_t i = 0; i < max_velocity_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_max_velocity_errori;
      u_max_velocity_errori.real = this->max_velocity_error[i];
      *(outbuffer + offset + 0) = (u_max_velocity_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_velocity_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_velocity_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_velocity_errori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_velocity_errori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_velocity_errori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_velocity_errori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_velocity_errori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_velocity_error[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_planned_duration;
      u_planned_duration.base = 0;
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_planned_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->planned_duration = u_planned_duration.real;
      offset += sizeof(this->planned_duration);
      union {
        double real;
        uint64_t base;
      } u_measured_duration;
      u_measured_duration.base = 0;
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_measured_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->measured_duration = u_measured_duration.real;
      offset += sizeof(this->measured_duration);
      uint32_t min_angle_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      min_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      min_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      min_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->min_angle_command_length);
      if(min_angle_command_lengthT > min_angle_command_length)
        this->min_angle_command = (double*)realloc(this->min_angle_command, min_angle_command_lengthT * sizeof(double));
      min_angle_command_length = min_angle_command_lengthT;
      for( uint32_t i = 0; i < min_angle_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_min_angle_command;
      u_st_min_angle_command.base = 0;
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_min_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_min_angle_command = u_st_min_angle_command.real;
      offset += sizeof(this->st_min_angle_command);
        memcpy( &(this->min_angle_command[i]), &(this->st_min_angle_command), sizeof(double));
      }
      uint32_t max_angle_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_angle_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_angle_command_length);
      if(max_angle_command_lengthT > max_angle_command_length)
        this->max_angle_command = (double*)realloc(this->max_angle_command, max_angle_command_lengthT * sizeof(double));
      max_angle_command_length = max_angle_command_lengthT;
      for( uint32_t i = 0; i < max_angle_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_max_angle_command;
      u_st_max_angle_command.base = 0;
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_max_angle_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_max_angle_command = u_st_max_angle_command.real;
      offset += sizeof(this->st_max_angle_command);
        memcpy( &(this->max_angle_command[i]), &(this->st_max_angle_command), sizeof(double));
      }
      uint32_t peak_speed_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      peak_speed_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      peak_speed_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      peak_speed_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->peak_speed_command_length);
      if(peak_speed_command_lengthT > peak_speed_command_length)
        this->peak_speed_command = (double*)realloc(this->peak_speed_command, peak_speed_command_lengthT * sizeof(double));
      peak_speed_command_length = peak_speed_command_lengthT;
      for( uint32_t i = 0; i < peak_speed_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_peak_speed_command;
      u_st_peak_speed_command.base = 0;
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_peak_speed_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_peak_speed_command = u_st_peak_speed_command.real;
      offset += sizeof(this->st_peak_speed_command);
        memcpy( &(this->peak_speed_command[i]), &(this->st_peak_speed_command), sizeof(double));
      }
      uint32_t peak_accel_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      peak_accel_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      peak_accel_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      peak_accel_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->peak_accel_command_length);
      if(peak_accel_command_lengthT > peak_accel_command_length)
        this->peak_accel_command = (double*)realloc(this->peak_accel_command, peak_accel_command_lengthT * sizeof(double));
      peak_accel_command_length = peak_accel_command_lengthT;
      for( uint32_t i = 0; i < peak_accel_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_peak_accel_command;
      u_st_peak_accel_command.base = 0;
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_peak_accel_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_peak_accel_command = u_st_peak_accel_command.real;
      offset += sizeof(this->st_peak_accel_command);
        memcpy( &(this->peak_accel_command[i]), &(this->st_peak_accel_command), sizeof(double));
      }
      uint32_t peak_jerk_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      peak_jerk_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      peak_jerk_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      peak_jerk_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->peak_jerk_command_length);
      if(peak_jerk_command_lengthT > peak_jerk_command_length)
        this->peak_jerk_command = (double*)realloc(this->peak_jerk_command, peak_jerk_command_lengthT * sizeof(double));
      peak_jerk_command_length = peak_jerk_command_lengthT;
      for( uint32_t i = 0; i < peak_jerk_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_peak_jerk_command;
      u_st_peak_jerk_command.base = 0;
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_peak_jerk_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_peak_jerk_command = u_st_peak_jerk_command.real;
      offset += sizeof(this->st_peak_jerk_command);
        memcpy( &(this->peak_jerk_command[i]), &(this->st_peak_jerk_command), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_min_time_rate;
      u_min_time_rate.base = 0;
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min_time_rate = u_min_time_rate.real;
      offset += sizeof(this->min_time_rate);
      union {
        double real;
        uint64_t base;
      } u_max_time_rate;
      u_max_time_rate.base = 0;
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_time_rate = u_max_time_rate.real;
      offset += sizeof(this->max_time_rate);
      uint32_t max_position_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_position_error_length);
      if(max_position_error_lengthT > max_position_error_length)
        this->max_position_error = (double*)realloc(this->max_position_error, max_position_error_lengthT * sizeof(double));
      max_position_error_length = max_position_error_lengthT;
      for( uint32_t i = 0; i < max_position_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_max_position_error;
      u_st_max_position_error.base = 0;
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_max_position_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_max_position_error = u_st_max_position_error.real;
      offset += sizeof(this->st_max_position_error);
        memcpy( &(this->max_position_error[i]), &(this->st_max_position_error), sizeof(double));
      }
      uint32_t max_velocity_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_velocity_error_length);
      if(max_velocity_error_lengthT > max_velocity_error_length)
        this->max_velocity_error = (double*)realloc(this->max_velocity_error, max_velocity_error_lengthT * sizeof(double));
      max_velocity_error_length = max_velocity_error_lengthT;
      for( uint32_t i = 0; i < max_velocity_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_max_velocity_error;
      u_st_max_velocity_error.base = 0;
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_max_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_max_velocity_error = u_st_max_velocity_error.real;
      offset += sizeof(this->st_max_velocity_error);
        memcpy( &(this->max_velocity_error[i]), &(this->st_max_velocity_error), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/TrajectoryAnalysis"; };
    const char * getMD5(){ return "f30ec541413b4eb2cecc0d0af7d30ad4"; };

  };

}
#endif
