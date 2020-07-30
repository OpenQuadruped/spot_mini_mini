#ifndef _ROS_control_msgs_JointJog_h
#define _ROS_control_msgs_JointJog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace control_msgs
{

  class JointJog : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t displacements_length;
      typedef double _displacements_type;
      _displacements_type st_displacements;
      _displacements_type * displacements;
      uint32_t velocities_length;
      typedef double _velocities_type;
      _velocities_type st_velocities;
      _velocities_type * velocities;
      typedef double _duration_type;
      _duration_type duration;

    JointJog():
      header(),
      joint_names_length(0), joint_names(NULL),
      displacements_length(0), displacements(NULL),
      velocities_length(0), velocities(NULL),
      duration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      *(outbuffer + offset + 0) = (this->displacements_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->displacements_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->displacements_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->displacements_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->displacements_length);
      for( uint32_t i = 0; i < displacements_length; i++){
      union {
        double real;
        uint64_t base;
      } u_displacementsi;
      u_displacementsi.real = this->displacements[i];
      *(outbuffer + offset + 0) = (u_displacementsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_displacementsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_displacementsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_displacementsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_displacementsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_displacementsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_displacementsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_displacementsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->displacements[i]);
      }
      *(outbuffer + offset + 0) = (this->velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_length);
      for( uint32_t i = 0; i < velocities_length; i++){
      union {
        double real;
        uint64_t base;
      } u_velocitiesi;
      u_velocitiesi.real = this->velocities[i];
      *(outbuffer + offset + 0) = (u_velocitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocitiesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_velocitiesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_velocitiesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_velocitiesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_velocitiesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->velocities[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_duration;
      u_duration.real = this->duration;
      *(outbuffer + offset + 0) = (u_duration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_duration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_duration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_duration.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_duration.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_duration.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_duration.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_duration.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->duration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
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
      uint32_t displacements_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      displacements_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      displacements_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      displacements_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->displacements_length);
      if(displacements_lengthT > displacements_length)
        this->displacements = (double*)realloc(this->displacements, displacements_lengthT * sizeof(double));
      displacements_length = displacements_lengthT;
      for( uint32_t i = 0; i < displacements_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_displacements;
      u_st_displacements.base = 0;
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_displacements.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_displacements = u_st_displacements.real;
      offset += sizeof(this->st_displacements);
        memcpy( &(this->displacements[i]), &(this->st_displacements), sizeof(double));
      }
      uint32_t velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_length);
      if(velocities_lengthT > velocities_length)
        this->velocities = (double*)realloc(this->velocities, velocities_lengthT * sizeof(double));
      velocities_length = velocities_lengthT;
      for( uint32_t i = 0; i < velocities_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_velocities;
      u_st_velocities.base = 0;
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_velocities.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_velocities = u_st_velocities.real;
      offset += sizeof(this->st_velocities);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_duration;
      u_duration.base = 0;
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_duration.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->duration = u_duration.real;
      offset += sizeof(this->duration);
     return offset;
    }

    const char * getType(){ return "control_msgs/JointJog"; };
    const char * getMD5(){ return "1685da700c8c2e1254afc92a5fb89c96"; };

  };

}
#endif
