#ifndef _ROS_flexbe_msgs_BehaviorExecutionFeedback_h
#define _ROS_flexbe_msgs_BehaviorExecutionFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class BehaviorExecutionFeedback : public ros::Msg
  {
    public:
      typedef const char* _current_state_type;
      _current_state_type current_state;

    BehaviorExecutionFeedback():
      current_state("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_current_state = strlen(this->current_state);
      varToArr(outbuffer + offset, length_current_state);
      offset += 4;
      memcpy(outbuffer + offset, this->current_state, length_current_state);
      offset += length_current_state;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_current_state;
      arrToVar(length_current_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_state-1]=0;
      this->current_state = (char *)(inbuffer + offset-1);
      offset += length_current_state;
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorExecutionFeedback"; };
    const char * getMD5(){ return "6b3c1b68b57cc9263bf79fc4ad6ec8c7"; };

  };

}
#endif
