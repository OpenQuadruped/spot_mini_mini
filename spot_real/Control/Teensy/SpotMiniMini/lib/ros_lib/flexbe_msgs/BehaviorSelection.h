#ifndef _ROS_flexbe_msgs_BehaviorSelection_h
#define _ROS_flexbe_msgs_BehaviorSelection_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/BehaviorModification.h"

namespace flexbe_msgs
{

  class BehaviorSelection : public ros::Msg
  {
    public:
      typedef int32_t _behavior_id_type;
      _behavior_id_type behavior_id;
      typedef int32_t _behavior_checksum_type;
      _behavior_checksum_type behavior_checksum;
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
      uint32_t input_keys_length;
      typedef char* _input_keys_type;
      _input_keys_type st_input_keys;
      _input_keys_type * input_keys;
      uint32_t input_values_length;
      typedef char* _input_values_type;
      _input_values_type st_input_values;
      _input_values_type * input_values;
      uint32_t modifications_length;
      typedef flexbe_msgs::BehaviorModification _modifications_type;
      _modifications_type st_modifications;
      _modifications_type * modifications;

    BehaviorSelection():
      behavior_id(0),
      behavior_checksum(0),
      autonomy_level(0),
      arg_keys_length(0), arg_keys(NULL),
      arg_values_length(0), arg_values(NULL),
      input_keys_length(0), input_keys(NULL),
      input_values_length(0), input_values(NULL),
      modifications_length(0), modifications(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_id;
      u_behavior_id.real = this->behavior_id;
      *(outbuffer + offset + 0) = (u_behavior_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_behavior_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_behavior_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_behavior_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->behavior_id);
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_checksum;
      u_behavior_checksum.real = this->behavior_checksum;
      *(outbuffer + offset + 0) = (u_behavior_checksum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_behavior_checksum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_behavior_checksum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_behavior_checksum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->behavior_checksum);
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
      *(outbuffer + offset + 0) = (this->input_keys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input_keys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->input_keys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->input_keys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_keys_length);
      for( uint32_t i = 0; i < input_keys_length; i++){
      uint32_t length_input_keysi = strlen(this->input_keys[i]);
      varToArr(outbuffer + offset, length_input_keysi);
      offset += 4;
      memcpy(outbuffer + offset, this->input_keys[i], length_input_keysi);
      offset += length_input_keysi;
      }
      *(outbuffer + offset + 0) = (this->input_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->input_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->input_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_values_length);
      for( uint32_t i = 0; i < input_values_length; i++){
      uint32_t length_input_valuesi = strlen(this->input_values[i]);
      varToArr(outbuffer + offset, length_input_valuesi);
      offset += 4;
      memcpy(outbuffer + offset, this->input_values[i], length_input_valuesi);
      offset += length_input_valuesi;
      }
      *(outbuffer + offset + 0) = (this->modifications_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->modifications_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->modifications_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->modifications_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->modifications_length);
      for( uint32_t i = 0; i < modifications_length; i++){
      offset += this->modifications[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_id;
      u_behavior_id.base = 0;
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->behavior_id = u_behavior_id.real;
      offset += sizeof(this->behavior_id);
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_checksum;
      u_behavior_checksum.base = 0;
      u_behavior_checksum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_behavior_checksum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_behavior_checksum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_behavior_checksum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->behavior_checksum = u_behavior_checksum.real;
      offset += sizeof(this->behavior_checksum);
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
      uint32_t input_keys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      input_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      input_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      input_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->input_keys_length);
      if(input_keys_lengthT > input_keys_length)
        this->input_keys = (char**)realloc(this->input_keys, input_keys_lengthT * sizeof(char*));
      input_keys_length = input_keys_lengthT;
      for( uint32_t i = 0; i < input_keys_length; i++){
      uint32_t length_st_input_keys;
      arrToVar(length_st_input_keys, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_input_keys; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_input_keys-1]=0;
      this->st_input_keys = (char *)(inbuffer + offset-1);
      offset += length_st_input_keys;
        memcpy( &(this->input_keys[i]), &(this->st_input_keys), sizeof(char*));
      }
      uint32_t input_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      input_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      input_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      input_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->input_values_length);
      if(input_values_lengthT > input_values_length)
        this->input_values = (char**)realloc(this->input_values, input_values_lengthT * sizeof(char*));
      input_values_length = input_values_lengthT;
      for( uint32_t i = 0; i < input_values_length; i++){
      uint32_t length_st_input_values;
      arrToVar(length_st_input_values, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_input_values; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_input_values-1]=0;
      this->st_input_values = (char *)(inbuffer + offset-1);
      offset += length_st_input_values;
        memcpy( &(this->input_values[i]), &(this->st_input_values), sizeof(char*));
      }
      uint32_t modifications_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      modifications_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      modifications_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      modifications_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->modifications_length);
      if(modifications_lengthT > modifications_length)
        this->modifications = (flexbe_msgs::BehaviorModification*)realloc(this->modifications, modifications_lengthT * sizeof(flexbe_msgs::BehaviorModification));
      modifications_length = modifications_lengthT;
      for( uint32_t i = 0; i < modifications_length; i++){
      offset += this->st_modifications.deserialize(inbuffer + offset);
        memcpy( &(this->modifications[i]), &(this->st_modifications), sizeof(flexbe_msgs::BehaviorModification));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorSelection"; };
    const char * getMD5(){ return "514f1b101295bdbd417ee6cc46e3823c"; };

  };

}
#endif
