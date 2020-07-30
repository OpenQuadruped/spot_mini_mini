#ifndef _ROS_intera_motion_msgs_WaypointOptions_h
#define _ROS_intera_motion_msgs_WaypointOptions_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_motion_msgs
{

  class WaypointOptions : public ros::Msg
  {
    public:
      typedef const char* _label_type;
      _label_type label;
      typedef double _max_joint_speed_ratio_type;
      _max_joint_speed_ratio_type max_joint_speed_ratio;
      uint32_t joint_tolerances_length;
      typedef double _joint_tolerances_type;
      _joint_tolerances_type st_joint_tolerances;
      _joint_tolerances_type * joint_tolerances;
      uint32_t max_joint_accel_length;
      typedef double _max_joint_accel_type;
      _max_joint_accel_type st_max_joint_accel;
      _max_joint_accel_type * max_joint_accel;
      typedef double _max_linear_speed_type;
      _max_linear_speed_type max_linear_speed;
      typedef double _max_linear_accel_type;
      _max_linear_accel_type max_linear_accel;
      typedef double _max_rotational_speed_type;
      _max_rotational_speed_type max_rotational_speed;
      typedef double _max_rotational_accel_type;
      _max_rotational_accel_type max_rotational_accel;
      typedef double _corner_distance_type;
      _corner_distance_type corner_distance;

    WaypointOptions():
      label(""),
      max_joint_speed_ratio(0),
      joint_tolerances_length(0), joint_tolerances(NULL),
      max_joint_accel_length(0), max_joint_accel(NULL),
      max_linear_speed(0),
      max_linear_accel(0),
      max_rotational_speed(0),
      max_rotational_accel(0),
      corner_distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      union {
        double real;
        uint64_t base;
      } u_max_joint_speed_ratio;
      u_max_joint_speed_ratio.real = this->max_joint_speed_ratio;
      *(outbuffer + offset + 0) = (u_max_joint_speed_ratio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_joint_speed_ratio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_joint_speed_ratio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_joint_speed_ratio.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_joint_speed_ratio.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_joint_speed_ratio.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_joint_speed_ratio.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_joint_speed_ratio.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_joint_speed_ratio);
      *(outbuffer + offset + 0) = (this->joint_tolerances_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_tolerances_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_tolerances_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_tolerances_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_tolerances_length);
      for( uint32_t i = 0; i < joint_tolerances_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_tolerancesi;
      u_joint_tolerancesi.real = this->joint_tolerances[i];
      *(outbuffer + offset + 0) = (u_joint_tolerancesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_tolerancesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_tolerancesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_tolerancesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_tolerancesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_tolerancesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_tolerancesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_tolerancesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_tolerances[i]);
      }
      *(outbuffer + offset + 0) = (this->max_joint_accel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_joint_accel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_joint_accel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_joint_accel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_joint_accel_length);
      for( uint32_t i = 0; i < max_joint_accel_length; i++){
      union {
        double real;
        uint64_t base;
      } u_max_joint_acceli;
      u_max_joint_acceli.real = this->max_joint_accel[i];
      *(outbuffer + offset + 0) = (u_max_joint_acceli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_joint_acceli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_joint_acceli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_joint_acceli.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_joint_acceli.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_joint_acceli.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_joint_acceli.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_joint_acceli.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_joint_accel[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_max_linear_speed;
      u_max_linear_speed.real = this->max_linear_speed;
      *(outbuffer + offset + 0) = (u_max_linear_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_linear_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_linear_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_linear_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_linear_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_linear_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_linear_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_linear_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_linear_speed);
      union {
        double real;
        uint64_t base;
      } u_max_linear_accel;
      u_max_linear_accel.real = this->max_linear_accel;
      *(outbuffer + offset + 0) = (u_max_linear_accel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_linear_accel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_linear_accel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_linear_accel.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_linear_accel.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_linear_accel.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_linear_accel.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_linear_accel.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_linear_accel);
      union {
        double real;
        uint64_t base;
      } u_max_rotational_speed;
      u_max_rotational_speed.real = this->max_rotational_speed;
      *(outbuffer + offset + 0) = (u_max_rotational_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_rotational_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_rotational_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_rotational_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_rotational_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_rotational_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_rotational_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_rotational_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_rotational_speed);
      union {
        double real;
        uint64_t base;
      } u_max_rotational_accel;
      u_max_rotational_accel.real = this->max_rotational_accel;
      *(outbuffer + offset + 0) = (u_max_rotational_accel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_rotational_accel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_rotational_accel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_rotational_accel.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_rotational_accel.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_rotational_accel.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_rotational_accel.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_rotational_accel.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_rotational_accel);
      union {
        double real;
        uint64_t base;
      } u_corner_distance;
      u_corner_distance.real = this->corner_distance;
      *(outbuffer + offset + 0) = (u_corner_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_corner_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_corner_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_corner_distance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_corner_distance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_corner_distance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_corner_distance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_corner_distance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->corner_distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      union {
        double real;
        uint64_t base;
      } u_max_joint_speed_ratio;
      u_max_joint_speed_ratio.base = 0;
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_joint_speed_ratio.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_joint_speed_ratio = u_max_joint_speed_ratio.real;
      offset += sizeof(this->max_joint_speed_ratio);
      uint32_t joint_tolerances_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_tolerances_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_tolerances_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_tolerances_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_tolerances_length);
      if(joint_tolerances_lengthT > joint_tolerances_length)
        this->joint_tolerances = (double*)realloc(this->joint_tolerances, joint_tolerances_lengthT * sizeof(double));
      joint_tolerances_length = joint_tolerances_lengthT;
      for( uint32_t i = 0; i < joint_tolerances_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_tolerances;
      u_st_joint_tolerances.base = 0;
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_tolerances.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_tolerances = u_st_joint_tolerances.real;
      offset += sizeof(this->st_joint_tolerances);
        memcpy( &(this->joint_tolerances[i]), &(this->st_joint_tolerances), sizeof(double));
      }
      uint32_t max_joint_accel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_joint_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_joint_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_joint_accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_joint_accel_length);
      if(max_joint_accel_lengthT > max_joint_accel_length)
        this->max_joint_accel = (double*)realloc(this->max_joint_accel, max_joint_accel_lengthT * sizeof(double));
      max_joint_accel_length = max_joint_accel_lengthT;
      for( uint32_t i = 0; i < max_joint_accel_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_max_joint_accel;
      u_st_max_joint_accel.base = 0;
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_max_joint_accel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_max_joint_accel = u_st_max_joint_accel.real;
      offset += sizeof(this->st_max_joint_accel);
        memcpy( &(this->max_joint_accel[i]), &(this->st_max_joint_accel), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_max_linear_speed;
      u_max_linear_speed.base = 0;
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_linear_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_linear_speed = u_max_linear_speed.real;
      offset += sizeof(this->max_linear_speed);
      union {
        double real;
        uint64_t base;
      } u_max_linear_accel;
      u_max_linear_accel.base = 0;
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_linear_accel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_linear_accel = u_max_linear_accel.real;
      offset += sizeof(this->max_linear_accel);
      union {
        double real;
        uint64_t base;
      } u_max_rotational_speed;
      u_max_rotational_speed.base = 0;
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_rotational_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_rotational_speed = u_max_rotational_speed.real;
      offset += sizeof(this->max_rotational_speed);
      union {
        double real;
        uint64_t base;
      } u_max_rotational_accel;
      u_max_rotational_accel.base = 0;
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_rotational_accel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_rotational_accel = u_max_rotational_accel.real;
      offset += sizeof(this->max_rotational_accel);
      union {
        double real;
        uint64_t base;
      } u_corner_distance;
      u_corner_distance.base = 0;
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_corner_distance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->corner_distance = u_corner_distance.real;
      offset += sizeof(this->corner_distance);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/WaypointOptions"; };
    const char * getMD5(){ return "1b4687d4e536269b06e629169723339f"; };

  };

}
#endif
