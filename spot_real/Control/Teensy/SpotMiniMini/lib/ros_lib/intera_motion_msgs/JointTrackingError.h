#ifndef _ROS_intera_motion_msgs_JointTrackingError_h
#define _ROS_intera_motion_msgs_JointTrackingError_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace intera_motion_msgs
{

  class JointTrackingError : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef double _trajectory_time_type;
      _trajectory_time_type trajectory_time;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t position_error_length;
      typedef double _position_error_type;
      _position_error_type st_position_error;
      _position_error_type * position_error;
      uint32_t velocity_error_length;
      typedef double _velocity_error_type;
      _velocity_error_type st_velocity_error;
      _velocity_error_type * velocity_error;
      uint32_t position_command_length;
      typedef double _position_command_type;
      _position_command_type st_position_command;
      _position_command_type * position_command;
      uint32_t velocity_command_length;
      typedef double _velocity_command_type;
      _velocity_command_type st_velocity_command;
      _velocity_command_type * velocity_command;

    JointTrackingError():
      header(),
      trajectory_id(0),
      trajectory_time(0),
      joint_names_length(0), joint_names(NULL),
      position_error_length(0), position_error(NULL),
      velocity_error_length(0), velocity_error(NULL),
      position_command_length(0), position_command(NULL),
      velocity_command_length(0), velocity_command(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->trajectory_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id);
      union {
        double real;
        uint64_t base;
      } u_trajectory_time;
      u_trajectory_time.real = this->trajectory_time;
      *(outbuffer + offset + 0) = (u_trajectory_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trajectory_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trajectory_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trajectory_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_trajectory_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_trajectory_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_trajectory_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_trajectory_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->trajectory_time);
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
      *(outbuffer + offset + 0) = (this->position_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_error_length);
      for( uint32_t i = 0; i < position_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_position_errori;
      u_position_errori.real = this->position_error[i];
      *(outbuffer + offset + 0) = (u_position_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_errori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_errori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_errori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_errori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_errori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_error[i]);
      }
      *(outbuffer + offset + 0) = (this->velocity_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_error_length);
      for( uint32_t i = 0; i < velocity_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_velocity_errori;
      u_velocity_errori.real = this->velocity_error[i];
      *(outbuffer + offset + 0) = (u_velocity_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_errori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_errori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_errori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_errori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_errori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_error[i]);
      }
      *(outbuffer + offset + 0) = (this->position_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_command_length);
      for( uint32_t i = 0; i < position_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_position_commandi;
      u_position_commandi.real = this->position_command[i];
      *(outbuffer + offset + 0) = (u_position_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position_command[i]);
      }
      *(outbuffer + offset + 0) = (this->velocity_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_command_length);
      for( uint32_t i = 0; i < velocity_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_velocity_commandi;
      u_velocity_commandi.real = this->velocity_command[i];
      *(outbuffer + offset + 0) = (u_velocity_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocity_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocity_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocity_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocity_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocity_command[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->trajectory_id =  ((uint32_t) (*(inbuffer + offset)));
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->trajectory_id);
      union {
        double real;
        uint64_t base;
      } u_trajectory_time;
      u_trajectory_time.base = 0;
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->trajectory_time = u_trajectory_time.real;
      offset += sizeof(this->trajectory_time);
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
      uint32_t position_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_error_length);
      if(position_error_lengthT > position_error_length)
        this->position_error = (double*)realloc(this->position_error, position_error_lengthT * sizeof(double));
      position_error_length = position_error_lengthT;
      for( uint32_t i = 0; i < position_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_position_error;
      u_st_position_error.base = 0;
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_position_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_position_error = u_st_position_error.real;
      offset += sizeof(this->st_position_error);
        memcpy( &(this->position_error[i]), &(this->st_position_error), sizeof(double));
      }
      uint32_t velocity_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_error_length);
      if(velocity_error_lengthT > velocity_error_length)
        this->velocity_error = (double*)realloc(this->velocity_error, velocity_error_lengthT * sizeof(double));
      velocity_error_length = velocity_error_lengthT;
      for( uint32_t i = 0; i < velocity_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_velocity_error;
      u_st_velocity_error.base = 0;
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_velocity_error = u_st_velocity_error.real;
      offset += sizeof(this->st_velocity_error);
        memcpy( &(this->velocity_error[i]), &(this->st_velocity_error), sizeof(double));
      }
      uint32_t position_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_command_length);
      if(position_command_lengthT > position_command_length)
        this->position_command = (double*)realloc(this->position_command, position_command_lengthT * sizeof(double));
      position_command_length = position_command_lengthT;
      for( uint32_t i = 0; i < position_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_position_command;
      u_st_position_command.base = 0;
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_position_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_position_command = u_st_position_command.real;
      offset += sizeof(this->st_position_command);
        memcpy( &(this->position_command[i]), &(this->st_position_command), sizeof(double));
      }
      uint32_t velocity_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_command_length);
      if(velocity_command_lengthT > velocity_command_length)
        this->velocity_command = (double*)realloc(this->velocity_command, velocity_command_lengthT * sizeof(double));
      velocity_command_length = velocity_command_lengthT;
      for( uint32_t i = 0; i < velocity_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_velocity_command;
      u_st_velocity_command.base = 0;
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_velocity_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_velocity_command = u_st_velocity_command.real;
      offset += sizeof(this->st_velocity_command);
        memcpy( &(this->velocity_command[i]), &(this->st_velocity_command), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/JointTrackingError"; };
    const char * getMD5(){ return "a564fcc23df67e287d3c4c2aa3aef83b"; };

  };

}
#endif
