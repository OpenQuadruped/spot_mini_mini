#ifndef _ROS_flexbe_msgs_BehaviorSynthesisResult_h
#define _ROS_flexbe_msgs_BehaviorSynthesisResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/SynthesisErrorCodes.h"
#include "flexbe_msgs/StateInstantiation.h"

namespace flexbe_msgs
{

  class BehaviorSynthesisResult : public ros::Msg
  {
    public:
      typedef flexbe_msgs::SynthesisErrorCodes _error_code_type;
      _error_code_type error_code;
      uint32_t states_length;
      typedef flexbe_msgs::StateInstantiation _states_type;
      _states_type st_states;
      _states_type * states;

    BehaviorSynthesisResult():
      error_code(),
      states_length(0), states(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->error_code.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->states_length);
      for( uint32_t i = 0; i < states_length; i++){
      offset += this->states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->error_code.deserialize(inbuffer + offset);
      uint32_t states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->states_length);
      if(states_lengthT > states_length)
        this->states = (flexbe_msgs::StateInstantiation*)realloc(this->states, states_lengthT * sizeof(flexbe_msgs::StateInstantiation));
      states_length = states_lengthT;
      for( uint32_t i = 0; i < states_length; i++){
      offset += this->st_states.deserialize(inbuffer + offset);
        memcpy( &(this->states[i]), &(this->st_states), sizeof(flexbe_msgs::StateInstantiation));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorSynthesisResult"; };
    const char * getMD5(){ return "088bd6aee1c7ee2902db2babfd6af1c2"; };

  };

}
#endif
