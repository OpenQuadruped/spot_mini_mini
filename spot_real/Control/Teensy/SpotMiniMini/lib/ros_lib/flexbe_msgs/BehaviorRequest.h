#ifndef _ROS_flexbe_msgs_BehaviorRequest_h
#define _ROS_flexbe_msgs_BehaviorRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/Container.h"

namespace flexbe_msgs
{

  class BehaviorRequest : public ros::Msg
  {
    public:
      typedef const char* _behavior_name_type;
      _behavior_name_type behavior_name;
      typedef uint8_t _autonomy_level_type;
      _autonomy_level_type autonomy_level;
      uint32_t arg_keys_length;
      typedef char* _arg_keys_type;
      _arg_keys_type st_arg_keys;
      _arg_keys_type * arg_keys;
      uint32_t arg_values_length;
      typedef char* _arg_values_type;
      _arg_values_type st_arg_values;
      _arg_values_type * arg_values;
      uint32_t structure_length;
      typedef flexbe_msgs::Container _structure_type;
      _structure_type st_structure;
      _structure_type * structure;

    BehaviorRequest():
      behavior_name(""),
      autonomy_level(0),
      arg_keys_length(0), arg_keys(NULL),
      arg_values_length(0), arg_values(NULL),
      structure_length(0), structure(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_behavior_name = strlen(this->behavior_name);
      varToArr(outbuffer + offset, length_behavior_name);
      offset += 4;
      memcpy(outbuffer + offset, this->behavior_name, length_behavior_name);
      offset += length_behavior_name;
      *(outbuffer + offset + 0) = (this->autonomy_level >> (8 * 0)) & 0xFF;
      offset += sizeof(this->autonomy_level);
      *(outbuffer + offset + 0) = (this->arg_keys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arg_keys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arg_keys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arg_keys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg_keys_length);
      for( uint32_t i = 0; i < arg_keys_length; i++){
      uint32_t length_arg_keysi = strlen(this->arg_keys[i]);
      varToArr(outbuffer + offset, length_arg_keysi);
      offset += 4;
      memcpy(outbuffer + offset, this->arg_keys[i], length_arg_keysi);
      offset += length_arg_keysi;
      }
      *(outbuffer + offset + 0) = (this->arg_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->arg_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->arg_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->arg_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->arg_values_length);
      for( uint32_t i = 0; i < arg_values_length; i++){
      uint32_t length_arg_valuesi = strlen(this->arg_values[i]);
      varToArr(outbuffer + offset, length_arg_valuesi);
      offset += 4;
      memcpy(outbuffer + offset, this->arg_values[i], length_arg_valuesi);
      offset += length_arg_valuesi;
      }
      *(outbuffer + offset + 0) = (this->structure_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->structure_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->structure_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->structure_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->structure_length);
      for( uint32_t i = 0; i < structure_length; i++){
      offset += this->structure[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_behavior_name;
      arrToVar(length_behavior_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_behavior_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_behavior_name-1]=0;
      this->behavior_name = (char *)(inbuffer + offset-1);
      offset += length_behavior_name;
      this->autonomy_level =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->autonomy_level);
      uint32_t arg_keys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      arg_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      arg_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      arg_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->arg_keys_length);
      if(arg_keys_lengthT > arg_keys_length)
        this->arg_keys = (char**)realloc(this->arg_keys, arg_keys_lengthT * sizeof(char*));
      arg_keys_length = arg_keys_lengthT;
      for( uint32_t i = 0; i < arg_keys_length; i++){
      uint32_t length_st_arg_keys;
      arrToVar(length_st_arg_keys, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_arg_keys; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_arg_keys-1]=0;
      this->st_arg_keys = (char *)(inbuffer + offset-1);
      offset += length_st_arg_keys;
        memcpy( &(this->arg_keys[i]), &(this->st_arg_keys), sizeof(char*));
      }
      uint32_t arg_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      arg_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      arg_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      arg_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->arg_values_length);
      if(arg_values_lengthT > arg_values_length)
        this->arg_values = (char**)realloc(this->arg_values, arg_values_lengthT * sizeof(char*));
      arg_values_length = arg_values_lengthT;
      for( uint32_t i = 0; i < arg_values_length; i++){
      uint32_t length_st_arg_values;
      arrToVar(length_st_arg_values, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_arg_values; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_arg_values-1]=0;
      this->st_arg_values = (char *)(inbuffer + offset-1);
      offset += length_st_arg_values;
        memcpy( &(this->arg_values[i]), &(this->st_arg_values), sizeof(char*));
      }
      uint32_t structure_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      structure_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      structure_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      structure_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->structure_length);
      if(structure_lengthT > structure_length)
        this->structure = (flexbe_msgs::Container*)realloc(this->structure, structure_lengthT * sizeof(flexbe_msgs::Container));
      structure_length = structure_lengthT;
      for( uint32_t i = 0; i < structure_length; i++){
      offset += this->st_structure.deserialize(inbuffer + offset);
        memcpy( &(this->structure[i]), &(this->st_structure), sizeof(flexbe_msgs::Container));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorRequest"; };
    const char * getMD5(){ return "0b55949502d4c602376ee00a64d0d294"; };

  };

}
#endif
