#ifndef _ROS_flexbe_msgs_BehaviorInputResult_h
#define _ROS_flexbe_msgs_BehaviorInputResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class BehaviorInputResult : public ros::Msg
  {
    public:
      typedef uint8_t _result_code_type;
      _result_code_type result_code;
      typedef const char* _data_type;
      _data_type data;
      enum { RESULT_OK =  0 };
      enum { RESULT_FAILED =  1 };
      enum { RESULT_ABORTED =  2 };

    BehaviorInputResult():
      result_code(0),
      data("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->result_code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result_code);
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->result_code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->result_code);
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorInputResult"; };
    const char * getMD5(){ return "e0509f159e7c7bb2268efbc625f63b3f"; };

  };

}
#endif
