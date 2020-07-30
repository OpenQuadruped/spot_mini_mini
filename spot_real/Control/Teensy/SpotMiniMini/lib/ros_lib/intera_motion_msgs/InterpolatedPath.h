#ifndef _ROS_intera_motion_msgs_InterpolatedPath_h
#define _ROS_intera_motion_msgs_InterpolatedPath_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "intera_motion_msgs/WaypointSimple.h"

namespace intera_motion_msgs
{

  class InterpolatedPath : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _label_type;
      _label_type label;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;
      uint32_t interpolated_path_length;
      typedef intera_motion_msgs::WaypointSimple _interpolated_path_type;
      _interpolated_path_type st_interpolated_path;
      _interpolated_path_type * interpolated_path;

    InterpolatedPath():
      header(),
      label(""),
      joint_names_length(0), joint_names(NULL),
      interpolated_path_length(0), interpolated_path(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
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
      *(outbuffer + offset + 0) = (this->interpolated_path_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->interpolated_path_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->interpolated_path_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->interpolated_path_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->interpolated_path_length);
      for( uint32_t i = 0; i < interpolated_path_length; i++){
      offset += this->interpolated_path[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
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
      uint32_t interpolated_path_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      interpolated_path_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      interpolated_path_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      interpolated_path_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->interpolated_path_length);
      if(interpolated_path_lengthT > interpolated_path_length)
        this->interpolated_path = (intera_motion_msgs::WaypointSimple*)realloc(this->interpolated_path, interpolated_path_lengthT * sizeof(intera_motion_msgs::WaypointSimple));
      interpolated_path_length = interpolated_path_lengthT;
      for( uint32_t i = 0; i < interpolated_path_length; i++){
      offset += this->st_interpolated_path.deserialize(inbuffer + offset);
        memcpy( &(this->interpolated_path[i]), &(this->st_interpolated_path), sizeof(intera_motion_msgs::WaypointSimple));
      }
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/InterpolatedPath"; };
    const char * getMD5(){ return "4e078bdc2ed88b86420f5b19cbd78219"; };

  };

}
#endif
