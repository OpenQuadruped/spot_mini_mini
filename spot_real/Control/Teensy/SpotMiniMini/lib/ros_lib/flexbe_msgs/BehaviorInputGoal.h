#ifndef _ROS_flexbe_msgs_BehaviorInputGoal_h
#define _ROS_flexbe_msgs_BehaviorInputGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class BehaviorInputGoal : public ros::Msg
  {
    public:
      typedef uint8_t _request_type_type;
      _request_type_type request_type;
      typedef const char* _msg_type;
      _msg_type msg;

    BehaviorInputGoal():
      request_type(0),
      msg("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->request_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->request_type);
      uint32_t length_msg = strlen(this->msg);
      varToArr(outbuffer + offset, length_msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, length_msg);
      offset += length_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->request_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->request_type);
      uint32_t length_msg;
      arrToVar(length_msg, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorInputGoal"; };
    const char * getMD5(){ return "724150348fd57a5ebc07765871d3f316"; };

  };

}
#endif
