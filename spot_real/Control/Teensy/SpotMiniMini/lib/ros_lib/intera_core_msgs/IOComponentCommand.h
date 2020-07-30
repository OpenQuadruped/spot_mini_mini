#ifndef _ROS_intera_core_msgs_IOComponentCommand_h
#define _ROS_intera_core_msgs_IOComponentCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace intera_core_msgs
{

  class IOComponentCommand : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef const char* _op_type;
      _op_type op;
      typedef const char* _args_type;
      _args_type args;

    IOComponentCommand():
      time(),
      op(""),
      args("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
      uint32_t length_op = strlen(this->op);
      varToArr(outbuffer + offset, length_op);
      offset += 4;
      memcpy(outbuffer + offset, this->op, length_op);
      offset += length_op;
      uint32_t length_args = strlen(this->args);
      varToArr(outbuffer + offset, length_args);
      offset += 4;
      memcpy(outbuffer + offset, this->args, length_args);
      offset += length_args;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
      uint32_t length_op;
      arrToVar(length_op, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_op; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_op-1]=0;
      this->op = (char *)(inbuffer + offset-1);
      offset += length_op;
      uint32_t length_args;
      arrToVar(length_args, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_args; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_args-1]=0;
      this->args = (char *)(inbuffer + offset-1);
      offset += length_args;
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IOComponentCommand"; };
    const char * getMD5(){ return "ede95ba2953dc221dc82cac20f697530"; };

  };

}
#endif
