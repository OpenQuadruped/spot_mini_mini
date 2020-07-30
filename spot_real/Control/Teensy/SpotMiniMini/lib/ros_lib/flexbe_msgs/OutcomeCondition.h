#ifndef _ROS_flexbe_msgs_OutcomeCondition_h
#define _ROS_flexbe_msgs_OutcomeCondition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class OutcomeCondition : public ros::Msg
  {
    public:
      uint32_t state_name_length;
      typedef char* _state_name_type;
      _state_name_type st_state_name;
      _state_name_type * state_name;
      uint32_t state_outcome_length;
      typedef char* _state_outcome_type;
      _state_outcome_type st_state_outcome;
      _state_outcome_type * state_outcome;

    OutcomeCondition():
      state_name_length(0), state_name(NULL),
      state_outcome_length(0), state_outcome(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_name_length);
      for( uint32_t i = 0; i < state_name_length; i++){
      uint32_t length_state_namei = strlen(this->state_name[i]);
      varToArr(outbuffer + offset, length_state_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->state_name[i], length_state_namei);
      offset += length_state_namei;
      }
      *(outbuffer + offset + 0) = (this->state_outcome_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_outcome_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_outcome_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_outcome_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_outcome_length);
      for( uint32_t i = 0; i < state_outcome_length; i++){
      uint32_t length_state_outcomei = strlen(this->state_outcome[i]);
      varToArr(outbuffer + offset, length_state_outcomei);
      offset += 4;
      memcpy(outbuffer + offset, this->state_outcome[i], length_state_outcomei);
      offset += length_state_outcomei;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t state_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_name_length);
      if(state_name_lengthT > state_name_length)
        this->state_name = (char**)realloc(this->state_name, state_name_lengthT * sizeof(char*));
      state_name_length = state_name_lengthT;
      for( uint32_t i = 0; i < state_name_length; i++){
      uint32_t length_st_state_name;
      arrToVar(length_st_state_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_state_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_state_name-1]=0;
      this->st_state_name = (char *)(inbuffer + offset-1);
      offset += length_st_state_name;
        memcpy( &(this->state_name[i]), &(this->st_state_name), sizeof(char*));
      }
      uint32_t state_outcome_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_outcome_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_outcome_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_outcome_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_outcome_length);
      if(state_outcome_lengthT > state_outcome_length)
        this->state_outcome = (char**)realloc(this->state_outcome, state_outcome_lengthT * sizeof(char*));
      state_outcome_length = state_outcome_lengthT;
      for( uint32_t i = 0; i < state_outcome_length; i++){
      uint32_t length_st_state_outcome;
      arrToVar(length_st_state_outcome, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_state_outcome; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_state_outcome-1]=0;
      this->st_state_outcome = (char *)(inbuffer + offset-1);
      offset += length_st_state_outcome;
        memcpy( &(this->state_outcome[i]), &(this->st_state_outcome), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/OutcomeCondition"; };
    const char * getMD5(){ return "3f00d7f2d34167712ca29ef288547863"; };

  };

}
#endif
