#ifndef _ROS_baxter_core_msgs_SEAJointState_h
#define _ROS_baxter_core_msgs_SEAJointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace baxter_core_msgs
{

  class SEAJointState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t commanded_position_length;
      typedef double _commanded_position_type;
      _commanded_position_type st_commanded_position;
      _commanded_position_type * commanded_position;
      uint32_t commanded_velocity_length;
      typedef double _commanded_velocity_type;
      _commanded_velocity_type st_commanded_velocity;
      _commanded_velocity_type * commanded_velocity;
      uint32_t commanded_acceleration_length;
      typedef double _commanded_acceleration_type;
      _commanded_acceleration_type st_commanded_acceleration;
      _commanded_acceleration_type * commanded_acceleration;
      uint32_t commanded_effort_length;
      typedef double _commanded_effort_type;
      _commanded_effort_type st_commanded_effort;
      _commanded_effort_type * commanded_effort;
      uint32_t actual_position_length;
      typedef double _actual_position_type;
      _actual_position_type st_actual_position;
      _actual_position_type * actual_position;
      uint32_t actual_velocity_length;
      typedef double _actual_velocity_type;
      _actual_velocity_type st_actual_velocity;
      _actual_velocity_type * actual_velocity;
      uint32_t actual_effort_length;
      typedef double _actual_effort_type;
      _actual_effort_type st_actual_effort;
      _actual_effort_type * actual_effort;
      uint32_t gravity_model_effort_length;
      typedef double _gravity_model_effort_type;
      _gravity_model_effort_type st_gravity_model_effort;
      _gravity_model_effort_type * gravity_model_effort;
      uint32_t gravity_only_length;
      typedef double _gravity_only_type;
      _gravity_only_type st_gravity_only;
      _gravity_only_type * gravity_only;
      uint32_t hysteresis_model_effort_length;
      typedef double _hysteresis_model_effort_type;
      _hysteresis_model_effort_type st_hysteresis_model_effort;
      _hysteresis_model_effort_type * hysteresis_model_effort;
      uint32_t crosstalk_model_effort_length;
      typedef double _crosstalk_model_effort_type;
      _crosstalk_model_effort_type st_crosstalk_model_effort;
      _crosstalk_model_effort_type * crosstalk_model_effort;
      typedef double _hystState_type;
      _hystState_type hystState;

    SEAJointState():
      header(),
      name_length(0), name(NULL),
      commanded_position_length(0), commanded_position(NULL),
      commanded_velocity_length(0), commanded_velocity(NULL),
      commanded_acceleration_length(0), commanded_acceleration(NULL),
      commanded_effort_length(0), commanded_effort(NULL),
      actual_position_length(0), actual_position(NULL),
      actual_velocity_length(0), actual_velocity(NULL),
      actual_effort_length(0), actual_effort(NULL),
      gravity_model_effort_length(0), gravity_model_effort(NULL),
      gravity_only_length(0), gravity_only(NULL),
      hysteresis_model_effort_length(0), hysteresis_model_effort(NULL),
      crosstalk_model_effort_length(0), crosstalk_model_effort(NULL),
      hystState(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->commanded_position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commanded_position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commanded_position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commanded_position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commanded_position_length);
      for( uint32_t i = 0; i < commanded_position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_commanded_positioni;
      u_commanded_positioni.real = this->commanded_position[i];
      *(outbuffer + offset + 0) = (u_commanded_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commanded_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commanded_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commanded_positioni.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_commanded_positioni.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_commanded_positioni.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_commanded_positioni.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_commanded_positioni.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->commanded_position[i]);
      }
      *(outbuffer + offset + 0) = (this->commanded_velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commanded_velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commanded_velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commanded_velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commanded_velocity_length);
      for( uint32_t i = 0; i < commanded_velocity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_commanded_velocityi;
      u_commanded_velocityi.real = this->commanded_velocity[i];
      *(outbuffer + offset + 0) = (u_commanded_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commanded_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commanded_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commanded_velocityi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_commanded_velocityi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_commanded_velocityi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_commanded_velocityi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_commanded_velocityi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->commanded_velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->commanded_acceleration_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commanded_acceleration_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commanded_acceleration_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commanded_acceleration_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commanded_acceleration_length);
      for( uint32_t i = 0; i < commanded_acceleration_length; i++){
      union {
        double real;
        uint64_t base;
      } u_commanded_accelerationi;
      u_commanded_accelerationi.real = this->commanded_acceleration[i];
      *(outbuffer + offset + 0) = (u_commanded_accelerationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commanded_accelerationi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commanded_accelerationi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commanded_accelerationi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_commanded_accelerationi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_commanded_accelerationi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_commanded_accelerationi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_commanded_accelerationi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->commanded_acceleration[i]);
      }
      *(outbuffer + offset + 0) = (this->commanded_effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commanded_effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commanded_effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commanded_effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commanded_effort_length);
      for( uint32_t i = 0; i < commanded_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_commanded_efforti;
      u_commanded_efforti.real = this->commanded_effort[i];
      *(outbuffer + offset + 0) = (u_commanded_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commanded_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commanded_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commanded_efforti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_commanded_efforti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_commanded_efforti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_commanded_efforti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_commanded_efforti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->commanded_effort[i]);
      }
      *(outbuffer + offset + 0) = (this->actual_position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->actual_position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->actual_position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->actual_position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actual_position_length);
      for( uint32_t i = 0; i < actual_position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_actual_positioni;
      u_actual_positioni.real = this->actual_position[i];
      *(outbuffer + offset + 0) = (u_actual_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_actual_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_actual_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_actual_positioni.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_actual_positioni.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_actual_positioni.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_actual_positioni.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_actual_positioni.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->actual_position[i]);
      }
      *(outbuffer + offset + 0) = (this->actual_velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->actual_velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->actual_velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->actual_velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actual_velocity_length);
      for( uint32_t i = 0; i < actual_velocity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_actual_velocityi;
      u_actual_velocityi.real = this->actual_velocity[i];
      *(outbuffer + offset + 0) = (u_actual_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_actual_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_actual_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_actual_velocityi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_actual_velocityi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_actual_velocityi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_actual_velocityi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_actual_velocityi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->actual_velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->actual_effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->actual_effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->actual_effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->actual_effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->actual_effort_length);
      for( uint32_t i = 0; i < actual_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_actual_efforti;
      u_actual_efforti.real = this->actual_effort[i];
      *(outbuffer + offset + 0) = (u_actual_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_actual_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_actual_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_actual_efforti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_actual_efforti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_actual_efforti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_actual_efforti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_actual_efforti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->actual_effort[i]);
      }
      *(outbuffer + offset + 0) = (this->gravity_model_effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gravity_model_effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gravity_model_effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gravity_model_effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gravity_model_effort_length);
      for( uint32_t i = 0; i < gravity_model_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_gravity_model_efforti;
      u_gravity_model_efforti.real = this->gravity_model_effort[i];
      *(outbuffer + offset + 0) = (u_gravity_model_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gravity_model_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gravity_model_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gravity_model_efforti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gravity_model_efforti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gravity_model_efforti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gravity_model_efforti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gravity_model_efforti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gravity_model_effort[i]);
      }
      *(outbuffer + offset + 0) = (this->gravity_only_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gravity_only_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gravity_only_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gravity_only_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gravity_only_length);
      for( uint32_t i = 0; i < gravity_only_length; i++){
      union {
        double real;
        uint64_t base;
      } u_gravity_onlyi;
      u_gravity_onlyi.real = this->gravity_only[i];
      *(outbuffer + offset + 0) = (u_gravity_onlyi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gravity_onlyi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gravity_onlyi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gravity_onlyi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_gravity_onlyi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_gravity_onlyi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_gravity_onlyi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_gravity_onlyi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->gravity_only[i]);
      }
      *(outbuffer + offset + 0) = (this->hysteresis_model_effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->hysteresis_model_effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->hysteresis_model_effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->hysteresis_model_effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->hysteresis_model_effort_length);
      for( uint32_t i = 0; i < hysteresis_model_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_hysteresis_model_efforti;
      u_hysteresis_model_efforti.real = this->hysteresis_model_effort[i];
      *(outbuffer + offset + 0) = (u_hysteresis_model_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hysteresis_model_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hysteresis_model_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hysteresis_model_efforti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_hysteresis_model_efforti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_hysteresis_model_efforti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_hysteresis_model_efforti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_hysteresis_model_efforti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->hysteresis_model_effort[i]);
      }
      *(outbuffer + offset + 0) = (this->crosstalk_model_effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->crosstalk_model_effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->crosstalk_model_effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->crosstalk_model_effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->crosstalk_model_effort_length);
      for( uint32_t i = 0; i < crosstalk_model_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_crosstalk_model_efforti;
      u_crosstalk_model_efforti.real = this->crosstalk_model_effort[i];
      *(outbuffer + offset + 0) = (u_crosstalk_model_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_crosstalk_model_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_crosstalk_model_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_crosstalk_model_efforti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_crosstalk_model_efforti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_crosstalk_model_efforti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_crosstalk_model_efforti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_crosstalk_model_efforti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->crosstalk_model_effort[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_hystState;
      u_hystState.real = this->hystState;
      *(outbuffer + offset + 0) = (u_hystState.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_hystState.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_hystState.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_hystState.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_hystState.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_hystState.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_hystState.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_hystState.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->hystState);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t commanded_position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      commanded_position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      commanded_position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      commanded_position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->commanded_position_length);
      if(commanded_position_lengthT > commanded_position_length)
        this->commanded_position = (double*)realloc(this->commanded_position, commanded_position_lengthT * sizeof(double));
      commanded_position_length = commanded_position_lengthT;
      for( uint32_t i = 0; i < commanded_position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_commanded_position;
      u_st_commanded_position.base = 0;
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_commanded_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_commanded_position = u_st_commanded_position.real;
      offset += sizeof(this->st_commanded_position);
        memcpy( &(this->commanded_position[i]), &(this->st_commanded_position), sizeof(double));
      }
      uint32_t commanded_velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      commanded_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      commanded_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      commanded_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->commanded_velocity_length);
      if(commanded_velocity_lengthT > commanded_velocity_length)
        this->commanded_velocity = (double*)realloc(this->commanded_velocity, commanded_velocity_lengthT * sizeof(double));
      commanded_velocity_length = commanded_velocity_lengthT;
      for( uint32_t i = 0; i < commanded_velocity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_commanded_velocity;
      u_st_commanded_velocity.base = 0;
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_commanded_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_commanded_velocity = u_st_commanded_velocity.real;
      offset += sizeof(this->st_commanded_velocity);
        memcpy( &(this->commanded_velocity[i]), &(this->st_commanded_velocity), sizeof(double));
      }
      uint32_t commanded_acceleration_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      commanded_acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      commanded_acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      commanded_acceleration_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->commanded_acceleration_length);
      if(commanded_acceleration_lengthT > commanded_acceleration_length)
        this->commanded_acceleration = (double*)realloc(this->commanded_acceleration, commanded_acceleration_lengthT * sizeof(double));
      commanded_acceleration_length = commanded_acceleration_lengthT;
      for( uint32_t i = 0; i < commanded_acceleration_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_commanded_acceleration;
      u_st_commanded_acceleration.base = 0;
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_commanded_acceleration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_commanded_acceleration = u_st_commanded_acceleration.real;
      offset += sizeof(this->st_commanded_acceleration);
        memcpy( &(this->commanded_acceleration[i]), &(this->st_commanded_acceleration), sizeof(double));
      }
      uint32_t commanded_effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      commanded_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      commanded_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      commanded_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->commanded_effort_length);
      if(commanded_effort_lengthT > commanded_effort_length)
        this->commanded_effort = (double*)realloc(this->commanded_effort, commanded_effort_lengthT * sizeof(double));
      commanded_effort_length = commanded_effort_lengthT;
      for( uint32_t i = 0; i < commanded_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_commanded_effort;
      u_st_commanded_effort.base = 0;
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_commanded_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_commanded_effort = u_st_commanded_effort.real;
      offset += sizeof(this->st_commanded_effort);
        memcpy( &(this->commanded_effort[i]), &(this->st_commanded_effort), sizeof(double));
      }
      uint32_t actual_position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      actual_position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      actual_position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      actual_position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->actual_position_length);
      if(actual_position_lengthT > actual_position_length)
        this->actual_position = (double*)realloc(this->actual_position, actual_position_lengthT * sizeof(double));
      actual_position_length = actual_position_lengthT;
      for( uint32_t i = 0; i < actual_position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_actual_position;
      u_st_actual_position.base = 0;
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_actual_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_actual_position = u_st_actual_position.real;
      offset += sizeof(this->st_actual_position);
        memcpy( &(this->actual_position[i]), &(this->st_actual_position), sizeof(double));
      }
      uint32_t actual_velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      actual_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      actual_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      actual_velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->actual_velocity_length);
      if(actual_velocity_lengthT > actual_velocity_length)
        this->actual_velocity = (double*)realloc(this->actual_velocity, actual_velocity_lengthT * sizeof(double));
      actual_velocity_length = actual_velocity_lengthT;
      for( uint32_t i = 0; i < actual_velocity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_actual_velocity;
      u_st_actual_velocity.base = 0;
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_actual_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_actual_velocity = u_st_actual_velocity.real;
      offset += sizeof(this->st_actual_velocity);
        memcpy( &(this->actual_velocity[i]), &(this->st_actual_velocity), sizeof(double));
      }
      uint32_t actual_effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      actual_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      actual_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      actual_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->actual_effort_length);
      if(actual_effort_lengthT > actual_effort_length)
        this->actual_effort = (double*)realloc(this->actual_effort, actual_effort_lengthT * sizeof(double));
      actual_effort_length = actual_effort_lengthT;
      for( uint32_t i = 0; i < actual_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_actual_effort;
      u_st_actual_effort.base = 0;
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_actual_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_actual_effort = u_st_actual_effort.real;
      offset += sizeof(this->st_actual_effort);
        memcpy( &(this->actual_effort[i]), &(this->st_actual_effort), sizeof(double));
      }
      uint32_t gravity_model_effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      gravity_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      gravity_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      gravity_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->gravity_model_effort_length);
      if(gravity_model_effort_lengthT > gravity_model_effort_length)
        this->gravity_model_effort = (double*)realloc(this->gravity_model_effort, gravity_model_effort_lengthT * sizeof(double));
      gravity_model_effort_length = gravity_model_effort_lengthT;
      for( uint32_t i = 0; i < gravity_model_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_gravity_model_effort;
      u_st_gravity_model_effort.base = 0;
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_gravity_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_gravity_model_effort = u_st_gravity_model_effort.real;
      offset += sizeof(this->st_gravity_model_effort);
        memcpy( &(this->gravity_model_effort[i]), &(this->st_gravity_model_effort), sizeof(double));
      }
      uint32_t gravity_only_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      gravity_only_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      gravity_only_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      gravity_only_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->gravity_only_length);
      if(gravity_only_lengthT > gravity_only_length)
        this->gravity_only = (double*)realloc(this->gravity_only, gravity_only_lengthT * sizeof(double));
      gravity_only_length = gravity_only_lengthT;
      for( uint32_t i = 0; i < gravity_only_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_gravity_only;
      u_st_gravity_only.base = 0;
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_gravity_only.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_gravity_only = u_st_gravity_only.real;
      offset += sizeof(this->st_gravity_only);
        memcpy( &(this->gravity_only[i]), &(this->st_gravity_only), sizeof(double));
      }
      uint32_t hysteresis_model_effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      hysteresis_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      hysteresis_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      hysteresis_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->hysteresis_model_effort_length);
      if(hysteresis_model_effort_lengthT > hysteresis_model_effort_length)
        this->hysteresis_model_effort = (double*)realloc(this->hysteresis_model_effort, hysteresis_model_effort_lengthT * sizeof(double));
      hysteresis_model_effort_length = hysteresis_model_effort_lengthT;
      for( uint32_t i = 0; i < hysteresis_model_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_hysteresis_model_effort;
      u_st_hysteresis_model_effort.base = 0;
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_hysteresis_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_hysteresis_model_effort = u_st_hysteresis_model_effort.real;
      offset += sizeof(this->st_hysteresis_model_effort);
        memcpy( &(this->hysteresis_model_effort[i]), &(this->st_hysteresis_model_effort), sizeof(double));
      }
      uint32_t crosstalk_model_effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      crosstalk_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      crosstalk_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      crosstalk_model_effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->crosstalk_model_effort_length);
      if(crosstalk_model_effort_lengthT > crosstalk_model_effort_length)
        this->crosstalk_model_effort = (double*)realloc(this->crosstalk_model_effort, crosstalk_model_effort_lengthT * sizeof(double));
      crosstalk_model_effort_length = crosstalk_model_effort_lengthT;
      for( uint32_t i = 0; i < crosstalk_model_effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_crosstalk_model_effort;
      u_st_crosstalk_model_effort.base = 0;
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_crosstalk_model_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_crosstalk_model_effort = u_st_crosstalk_model_effort.real;
      offset += sizeof(this->st_crosstalk_model_effort);
        memcpy( &(this->crosstalk_model_effort[i]), &(this->st_crosstalk_model_effort), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_hystState;
      u_hystState.base = 0;
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_hystState.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->hystState = u_hystState.real;
      offset += sizeof(this->hystState);
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/SEAJointState"; };
    const char * getMD5(){ return "d36406dcbb6d860b1b39c4e28f81352b"; };

  };

}
#endif
