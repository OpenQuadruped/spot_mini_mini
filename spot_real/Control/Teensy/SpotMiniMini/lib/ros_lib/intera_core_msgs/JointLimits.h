#ifndef _ROS_intera_core_msgs_JointLimits_h
#define _ROS_intera_core_msgs_JointLimits_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class JointLimits : public ros::Msg
  {
    public:
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t position_lower_length;
      typedef double _position_lower_type;
      _position_lower_type st_position_lower;
      _position_lower_type * position_lower;
      uint32_t position_upper_length;
      typedef double _position_upper_type;
      _position_upper_type st_position_upper;
      _position_upper_type * position_upper;
      uint32_t velocity_length;
      typedef double _velocity_type;
      _velocity_type st_velocity;
      _velocity_type * velocity;
      uint32_t accel_length;
      typedef double _accel_type;
      _accel_type st_accel;
      _accel_type * accel;
      uint32_t effort_length;
      typedef double _effort_type;
      _effort_type st_effort;
      _effort_type * effort;

    JointLimits():
      joint_names_length(0), joint_names(NULL),
      position_lower_length(0), position_lower(NULL),
      position_upper_length(0), position_upper(NULL),
      velocity_length(0), velocity(NULL),
      accel_length(0), accel(NULL),
      effort_length(0), effort(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      *(outbuffer + offset + 0) = (this->position_lower_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_lower_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_lower_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_lower_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_lower_length);
      for( uint32_t i = 0; i < position_lower_length; i++){
      union {
        double real;
        uint64_t base;
      } u_position_loweri;
      u_position_loweri.real = this->position_lower[i];
      *(outbuffer + offset + 0) = (u_position_loweri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_loweri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_loweri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_loweri.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_loweri.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_loweri.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_loweri.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_loweri.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_lower[i]);
      }
      *(outbuffer + offset + 0) = (this->position_upper_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_upper_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_upper_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_upper_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_upper_length);
      for( uint32_t i = 0; i < position_upper_length; i++){
      union {
        double real;
        uint64_t base;
      } u_position_upperi;
      u_position_upperi.real = this->position_upper[i];
      *(outbuffer + offset + 0) = (u_position_upperi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_upperi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_upperi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_upperi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_upperi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_upperi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_upperi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_upperi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_upper[i]);
      }
      *(outbuffer + offset + 0) = (this->velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_length);
      for( uint32_t i = 0; i < velocity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_velocityi;
      u_velocityi.real = this->velocity[i];
      *(outbuffer + offset + 0) = (u_velocityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocityi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocityi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocityi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocityi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocityi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocityi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocityi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->accel_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accel_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accel_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accel_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_length);
      for( uint32_t i = 0; i < accel_length; i++){
      union {
        double real;
        uint64_t base;
      } u_acceli;
      u_acceli.real = this->accel[i];
      *(outbuffer + offset + 0) = (u_acceli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acceli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acceli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acceli.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_acceli.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_acceli.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_acceli.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_acceli.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->accel[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_length);
      for( uint32_t i = 0; i < effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_efforti;
      u_efforti.real = this->effort[i];
      *(outbuffer + offset + 0) = (u_efforti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_efforti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_efforti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_efforti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_efforti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_efforti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_efforti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_efforti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->effort[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
      uint32_t position_lower_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lower_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lower_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lower_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_lower_length);
      if(position_lower_lengthT > position_lower_length)
        this->position_lower = (double*)realloc(this->position_lower, position_lower_lengthT * sizeof(double));
      position_lower_length = position_lower_lengthT;
      for( uint32_t i = 0; i < position_lower_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_position_lower;
      u_st_position_lower.base = 0;
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_position_lower.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_position_lower = u_st_position_lower.real;
      offset += sizeof(this->st_position_lower);
        memcpy( &(this->position_lower[i]), &(this->st_position_lower), sizeof(double));
      }
      uint32_t position_upper_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_upper_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_upper_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_upper_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_upper_length);
      if(position_upper_lengthT > position_upper_length)
        this->position_upper = (double*)realloc(this->position_upper, position_upper_lengthT * sizeof(double));
      position_upper_length = position_upper_lengthT;
      for( uint32_t i = 0; i < position_upper_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_position_upper;
      u_st_position_upper.base = 0;
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_position_upper.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_position_upper = u_st_position_upper.real;
      offset += sizeof(this->st_position_upper);
        memcpy( &(this->position_upper[i]), &(this->st_position_upper), sizeof(double));
      }
      uint32_t velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_length);
      if(velocity_lengthT > velocity_length)
        this->velocity = (double*)realloc(this->velocity, velocity_lengthT * sizeof(double));
      velocity_length = velocity_lengthT;
      for( uint32_t i = 0; i < velocity_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_velocity;
      u_st_velocity.base = 0;
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_velocity.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_velocity = u_st_velocity.real;
      offset += sizeof(this->st_velocity);
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(double));
      }
      uint32_t accel_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      accel_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->accel_length);
      if(accel_lengthT > accel_length)
        this->accel = (double*)realloc(this->accel, accel_lengthT * sizeof(double));
      accel_length = accel_lengthT;
      for( uint32_t i = 0; i < accel_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_accel;
      u_st_accel.base = 0;
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_accel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_accel = u_st_accel.real;
      offset += sizeof(this->st_accel);
        memcpy( &(this->accel[i]), &(this->st_accel), sizeof(double));
      }
      uint32_t effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_length);
      if(effort_lengthT > effort_length)
        this->effort = (double*)realloc(this->effort, effort_lengthT * sizeof(double));
      effort_length = effort_lengthT;
      for( uint32_t i = 0; i < effort_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_effort;
      u_st_effort.base = 0;
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_effort = u_st_effort.real;
      offset += sizeof(this->st_effort);
        memcpy( &(this->effort[i]), &(this->st_effort), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/JointLimits"; };
    const char * getMD5(){ return "c4c445eb2c9324525a704c84ca1e7598"; };

  };

}
#endif
