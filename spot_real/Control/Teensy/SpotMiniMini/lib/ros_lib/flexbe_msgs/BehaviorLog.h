#ifndef _ROS_flexbe_msgs_BehaviorLog_h
#define _ROS_flexbe_msgs_BehaviorLog_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class BehaviorLog : public ros::Msg
  {
    public:
      typedef const char* _text_type;
      _text_type text;
      typedef uint8_t _status_code_type;
      _status_code_type status_code;
      enum { INFO =  0 };
      enum { WARN =  1 };
      enum { HINT =  2 };
      enum { ERROR =  3 };
      enum { DEBUG =  10 };

    BehaviorLog():
      text(""),
      status_code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_text = strlen(this->text);
      varToArr(outbuffer + offset, length_text);
      offset += 4;
      memcpy(outbuffer + offset, this->text, length_text);
      offset += length_text;
      *(outbuffer + offset + 0) = (this->status_code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status_code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_text;
      arrToVar(length_text, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_text; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_text-1]=0;
      this->text = (char *)(inbuffer + offset-1);
      offset += length_text;
      this->status_code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status_code);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorLog"; };
    const char * getMD5(){ return "03d7689372c7e2617b8d61fbf552e694"; };

  };

}
#endif
