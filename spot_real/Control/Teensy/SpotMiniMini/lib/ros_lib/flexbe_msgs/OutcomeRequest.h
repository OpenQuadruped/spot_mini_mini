#ifndef _ROS_flexbe_msgs_OutcomeRequest_h
#define _ROS_flexbe_msgs_OutcomeRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class OutcomeRequest : public ros::Msg
  {
    public:
      typedef uint8_t _outcome_type;
      _outcome_type outcome;
      typedef const char* _target_type;
      _target_type target;

    OutcomeRequest():
      outcome(0),
      target("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->outcome >> (8 * 0)) & 0xFF;
      offset += sizeof(this->outcome);
      uint32_t length_target = strlen(this->target);
      varToArr(outbuffer + offset, length_target);
      offset += 4;
      memcpy(outbuffer + offset, this->target, length_target);
      offset += length_target;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->outcome =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->outcome);
      uint32_t length_target;
      arrToVar(length_target, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_target; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_target-1]=0;
      this->target = (char *)(inbuffer + offset-1);
      offset += length_target;
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/OutcomeRequest"; };
    const char * getMD5(){ return "b057cf075cb943bf0dbb3443419a61ed"; };

  };

}
#endif
