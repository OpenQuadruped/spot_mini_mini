#ifndef _ROS_flexbe_msgs_SynthesisRequest_h
#define _ROS_flexbe_msgs_SynthesisRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class SynthesisRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _system_type;
      _system_type system;
      typedef const char* _goal_type;
      _goal_type goal;
      typedef const char* _initial_condition_type;
      _initial_condition_type initial_condition;
      uint32_t sm_outcomes_length;
      typedef char* _sm_outcomes_type;
      _sm_outcomes_type st_sm_outcomes;
      _sm_outcomes_type * sm_outcomes;

    SynthesisRequest():
      name(""),
      system(""),
      goal(""),
      initial_condition(""),
      sm_outcomes_length(0), sm_outcomes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_system = strlen(this->system);
      varToArr(outbuffer + offset, length_system);
      offset += 4;
      memcpy(outbuffer + offset, this->system, length_system);
      offset += length_system;
      uint32_t length_goal = strlen(this->goal);
      varToArr(outbuffer + offset, length_goal);
      offset += 4;
      memcpy(outbuffer + offset, this->goal, length_goal);
      offset += length_goal;
      uint32_t length_initial_condition = strlen(this->initial_condition);
      varToArr(outbuffer + offset, length_initial_condition);
      offset += 4;
      memcpy(outbuffer + offset, this->initial_condition, length_initial_condition);
      offset += length_initial_condition;
      *(outbuffer + offset + 0) = (this->sm_outcomes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sm_outcomes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sm_outcomes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sm_outcomes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sm_outcomes_length);
      for( uint32_t i = 0; i < sm_outcomes_length; i++){
      uint32_t length_sm_outcomesi = strlen(this->sm_outcomes[i]);
      varToArr(outbuffer + offset, length_sm_outcomesi);
      offset += 4;
      memcpy(outbuffer + offset, this->sm_outcomes[i], length_sm_outcomesi);
      offset += length_sm_outcomesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_system;
      arrToVar(length_system, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_system; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_system-1]=0;
      this->system = (char *)(inbuffer + offset-1);
      offset += length_system;
      uint32_t length_goal;
      arrToVar(length_goal, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_goal; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_goal-1]=0;
      this->goal = (char *)(inbuffer + offset-1);
      offset += length_goal;
      uint32_t length_initial_condition;
      arrToVar(length_initial_condition, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_initial_condition; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_initial_condition-1]=0;
      this->initial_condition = (char *)(inbuffer + offset-1);
      offset += length_initial_condition;
      uint32_t sm_outcomes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sm_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sm_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sm_outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sm_outcomes_length);
      if(sm_outcomes_lengthT > sm_outcomes_length)
        this->sm_outcomes = (char**)realloc(this->sm_outcomes, sm_outcomes_lengthT * sizeof(char*));
      sm_outcomes_length = sm_outcomes_lengthT;
      for( uint32_t i = 0; i < sm_outcomes_length; i++){
      uint32_t length_st_sm_outcomes;
      arrToVar(length_st_sm_outcomes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_sm_outcomes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_sm_outcomes-1]=0;
      this->st_sm_outcomes = (char *)(inbuffer + offset-1);
      offset += length_st_sm_outcomes;
        memcpy( &(this->sm_outcomes[i]), &(this->st_sm_outcomes), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/SynthesisRequest"; };
    const char * getMD5(){ return "99257dbfe8a196d006a75837dcabf3f6"; };

  };

}
#endif
