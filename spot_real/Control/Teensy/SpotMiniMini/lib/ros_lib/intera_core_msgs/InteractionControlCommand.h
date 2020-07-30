#ifndef _ROS_intera_core_msgs_InteractionControlCommand_h
#define _ROS_intera_core_msgs_InteractionControlCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace intera_core_msgs
{

  class InteractionControlCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _interaction_control_active_type;
      _interaction_control_active_type interaction_control_active;
      uint32_t K_impedance_length;
      typedef double _K_impedance_type;
      _K_impedance_type st_K_impedance;
      _K_impedance_type * K_impedance;
      uint32_t max_impedance_length;
      typedef bool _max_impedance_type;
      _max_impedance_type st_max_impedance;
      _max_impedance_type * max_impedance;
      uint32_t D_impedance_length;
      typedef double _D_impedance_type;
      _D_impedance_type st_D_impedance;
      _D_impedance_type * D_impedance;
      uint32_t K_nullspace_length;
      typedef double _K_nullspace_type;
      _K_nullspace_type st_K_nullspace;
      _K_nullspace_type * K_nullspace;
      uint32_t force_command_length;
      typedef double _force_command_type;
      _force_command_type st_force_command;
      _force_command_type * force_command;
      typedef geometry_msgs::Pose _interaction_frame_type;
      _interaction_frame_type interaction_frame;
      typedef const char* _endpoint_name_type;
      _endpoint_name_type endpoint_name;
      typedef bool _in_endpoint_frame_type;
      _in_endpoint_frame_type in_endpoint_frame;
      typedef bool _disable_damping_in_force_control_type;
      _disable_damping_in_force_control_type disable_damping_in_force_control;
      typedef bool _disable_reference_resetting_type;
      _disable_reference_resetting_type disable_reference_resetting;
      uint32_t interaction_control_mode_length;
      typedef uint8_t _interaction_control_mode_type;
      _interaction_control_mode_type st_interaction_control_mode;
      _interaction_control_mode_type * interaction_control_mode;
      typedef bool _rotations_for_constrained_zeroG_type;
      _rotations_for_constrained_zeroG_type rotations_for_constrained_zeroG;
      enum { IMPEDANCE_MODE = 1 };
      enum { FORCE_MODE = 2 };
      enum { IMPEDANCE_WITH_FORCE_LIMIT_MODE = 3 };
      enum { FORCE_WITH_MOTION_LIMIT_MODE = 4 };

    InteractionControlCommand():
      header(),
      interaction_control_active(0),
      K_impedance_length(0), K_impedance(NULL),
      max_impedance_length(0), max_impedance(NULL),
      D_impedance_length(0), D_impedance(NULL),
      K_nullspace_length(0), K_nullspace(NULL),
      force_command_length(0), force_command(NULL),
      interaction_frame(),
      endpoint_name(""),
      in_endpoint_frame(0),
      disable_damping_in_force_control(0),
      disable_reference_resetting(0),
      interaction_control_mode_length(0), interaction_control_mode(NULL),
      rotations_for_constrained_zeroG(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_interaction_control_active;
      u_interaction_control_active.real = this->interaction_control_active;
      *(outbuffer + offset + 0) = (u_interaction_control_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->interaction_control_active);
      *(outbuffer + offset + 0) = (this->K_impedance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->K_impedance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->K_impedance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->K_impedance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->K_impedance_length);
      for( uint32_t i = 0; i < K_impedance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_K_impedancei;
      u_K_impedancei.real = this->K_impedance[i];
      *(outbuffer + offset + 0) = (u_K_impedancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_K_impedancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_K_impedancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_K_impedancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_K_impedancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_K_impedancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_K_impedancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_K_impedancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->K_impedance[i]);
      }
      *(outbuffer + offset + 0) = (this->max_impedance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_impedance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_impedance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_impedance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_impedance_length);
      for( uint32_t i = 0; i < max_impedance_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_max_impedancei;
      u_max_impedancei.real = this->max_impedance[i];
      *(outbuffer + offset + 0) = (u_max_impedancei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->max_impedance[i]);
      }
      *(outbuffer + offset + 0) = (this->D_impedance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->D_impedance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->D_impedance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->D_impedance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->D_impedance_length);
      for( uint32_t i = 0; i < D_impedance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_D_impedancei;
      u_D_impedancei.real = this->D_impedance[i];
      *(outbuffer + offset + 0) = (u_D_impedancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_D_impedancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_D_impedancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_D_impedancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_D_impedancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_D_impedancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_D_impedancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_D_impedancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->D_impedance[i]);
      }
      *(outbuffer + offset + 0) = (this->K_nullspace_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->K_nullspace_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->K_nullspace_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->K_nullspace_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->K_nullspace_length);
      for( uint32_t i = 0; i < K_nullspace_length; i++){
      union {
        double real;
        uint64_t base;
      } u_K_nullspacei;
      u_K_nullspacei.real = this->K_nullspace[i];
      *(outbuffer + offset + 0) = (u_K_nullspacei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_K_nullspacei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_K_nullspacei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_K_nullspacei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_K_nullspacei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_K_nullspacei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_K_nullspacei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_K_nullspacei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->K_nullspace[i]);
      }
      *(outbuffer + offset + 0) = (this->force_command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->force_command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->force_command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->force_command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force_command_length);
      for( uint32_t i = 0; i < force_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_force_commandi;
      u_force_commandi.real = this->force_command[i];
      *(outbuffer + offset + 0) = (u_force_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_force_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_force_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_force_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_force_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->force_command[i]);
      }
      offset += this->interaction_frame.serialize(outbuffer + offset);
      uint32_t length_endpoint_name = strlen(this->endpoint_name);
      varToArr(outbuffer + offset, length_endpoint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->endpoint_name, length_endpoint_name);
      offset += length_endpoint_name;
      union {
        bool real;
        uint8_t base;
      } u_in_endpoint_frame;
      u_in_endpoint_frame.real = this->in_endpoint_frame;
      *(outbuffer + offset + 0) = (u_in_endpoint_frame.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->in_endpoint_frame);
      union {
        bool real;
        uint8_t base;
      } u_disable_damping_in_force_control;
      u_disable_damping_in_force_control.real = this->disable_damping_in_force_control;
      *(outbuffer + offset + 0) = (u_disable_damping_in_force_control.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->disable_damping_in_force_control);
      union {
        bool real;
        uint8_t base;
      } u_disable_reference_resetting;
      u_disable_reference_resetting.real = this->disable_reference_resetting;
      *(outbuffer + offset + 0) = (u_disable_reference_resetting.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->disable_reference_resetting);
      *(outbuffer + offset + 0) = (this->interaction_control_mode_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->interaction_control_mode_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->interaction_control_mode_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->interaction_control_mode_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->interaction_control_mode_length);
      for( uint32_t i = 0; i < interaction_control_mode_length; i++){
      *(outbuffer + offset + 0) = (this->interaction_control_mode[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->interaction_control_mode[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_rotations_for_constrained_zeroG;
      u_rotations_for_constrained_zeroG.real = this->rotations_for_constrained_zeroG;
      *(outbuffer + offset + 0) = (u_rotations_for_constrained_zeroG.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rotations_for_constrained_zeroG);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_interaction_control_active;
      u_interaction_control_active.base = 0;
      u_interaction_control_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->interaction_control_active = u_interaction_control_active.real;
      offset += sizeof(this->interaction_control_active);
      uint32_t K_impedance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      K_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      K_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      K_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->K_impedance_length);
      if(K_impedance_lengthT > K_impedance_length)
        this->K_impedance = (double*)realloc(this->K_impedance, K_impedance_lengthT * sizeof(double));
      K_impedance_length = K_impedance_lengthT;
      for( uint32_t i = 0; i < K_impedance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_K_impedance;
      u_st_K_impedance.base = 0;
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_K_impedance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_K_impedance = u_st_K_impedance.real;
      offset += sizeof(this->st_K_impedance);
        memcpy( &(this->K_impedance[i]), &(this->st_K_impedance), sizeof(double));
      }
      uint32_t max_impedance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      max_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      max_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      max_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->max_impedance_length);
      if(max_impedance_lengthT > max_impedance_length)
        this->max_impedance = (bool*)realloc(this->max_impedance, max_impedance_lengthT * sizeof(bool));
      max_impedance_length = max_impedance_lengthT;
      for( uint32_t i = 0; i < max_impedance_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_max_impedance;
      u_st_max_impedance.base = 0;
      u_st_max_impedance.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_max_impedance = u_st_max_impedance.real;
      offset += sizeof(this->st_max_impedance);
        memcpy( &(this->max_impedance[i]), &(this->st_max_impedance), sizeof(bool));
      }
      uint32_t D_impedance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      D_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      D_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      D_impedance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->D_impedance_length);
      if(D_impedance_lengthT > D_impedance_length)
        this->D_impedance = (double*)realloc(this->D_impedance, D_impedance_lengthT * sizeof(double));
      D_impedance_length = D_impedance_lengthT;
      for( uint32_t i = 0; i < D_impedance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_D_impedance;
      u_st_D_impedance.base = 0;
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_D_impedance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_D_impedance = u_st_D_impedance.real;
      offset += sizeof(this->st_D_impedance);
        memcpy( &(this->D_impedance[i]), &(this->st_D_impedance), sizeof(double));
      }
      uint32_t K_nullspace_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      K_nullspace_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      K_nullspace_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      K_nullspace_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->K_nullspace_length);
      if(K_nullspace_lengthT > K_nullspace_length)
        this->K_nullspace = (double*)realloc(this->K_nullspace, K_nullspace_lengthT * sizeof(double));
      K_nullspace_length = K_nullspace_lengthT;
      for( uint32_t i = 0; i < K_nullspace_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_K_nullspace;
      u_st_K_nullspace.base = 0;
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_K_nullspace.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_K_nullspace = u_st_K_nullspace.real;
      offset += sizeof(this->st_K_nullspace);
        memcpy( &(this->K_nullspace[i]), &(this->st_K_nullspace), sizeof(double));
      }
      uint32_t force_command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      force_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      force_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      force_command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->force_command_length);
      if(force_command_lengthT > force_command_length)
        this->force_command = (double*)realloc(this->force_command, force_command_lengthT * sizeof(double));
      force_command_length = force_command_lengthT;
      for( uint32_t i = 0; i < force_command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_force_command;
      u_st_force_command.base = 0;
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_force_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_force_command = u_st_force_command.real;
      offset += sizeof(this->st_force_command);
        memcpy( &(this->force_command[i]), &(this->st_force_command), sizeof(double));
      }
      offset += this->interaction_frame.deserialize(inbuffer + offset);
      uint32_t length_endpoint_name;
      arrToVar(length_endpoint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_endpoint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_endpoint_name-1]=0;
      this->endpoint_name = (char *)(inbuffer + offset-1);
      offset += length_endpoint_name;
      union {
        bool real;
        uint8_t base;
      } u_in_endpoint_frame;
      u_in_endpoint_frame.base = 0;
      u_in_endpoint_frame.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->in_endpoint_frame = u_in_endpoint_frame.real;
      offset += sizeof(this->in_endpoint_frame);
      union {
        bool real;
        uint8_t base;
      } u_disable_damping_in_force_control;
      u_disable_damping_in_force_control.base = 0;
      u_disable_damping_in_force_control.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->disable_damping_in_force_control = u_disable_damping_in_force_control.real;
      offset += sizeof(this->disable_damping_in_force_control);
      union {
        bool real;
        uint8_t base;
      } u_disable_reference_resetting;
      u_disable_reference_resetting.base = 0;
      u_disable_reference_resetting.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->disable_reference_resetting = u_disable_reference_resetting.real;
      offset += sizeof(this->disable_reference_resetting);
      uint32_t interaction_control_mode_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      interaction_control_mode_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      interaction_control_mode_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      interaction_control_mode_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->interaction_control_mode_length);
      if(interaction_control_mode_lengthT > interaction_control_mode_length)
        this->interaction_control_mode = (uint8_t*)realloc(this->interaction_control_mode, interaction_control_mode_lengthT * sizeof(uint8_t));
      interaction_control_mode_length = interaction_control_mode_lengthT;
      for( uint32_t i = 0; i < interaction_control_mode_length; i++){
      this->st_interaction_control_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_interaction_control_mode);
        memcpy( &(this->interaction_control_mode[i]), &(this->st_interaction_control_mode), sizeof(uint8_t));
      }
      union {
        bool real;
        uint8_t base;
      } u_rotations_for_constrained_zeroG;
      u_rotations_for_constrained_zeroG.base = 0;
      u_rotations_for_constrained_zeroG.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rotations_for_constrained_zeroG = u_rotations_for_constrained_zeroG.real;
      offset += sizeof(this->rotations_for_constrained_zeroG);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/InteractionControlCommand"; };
    const char * getMD5(){ return "762b5d197d5d786f83741af7fe6261a6"; };

  };

}
#endif
