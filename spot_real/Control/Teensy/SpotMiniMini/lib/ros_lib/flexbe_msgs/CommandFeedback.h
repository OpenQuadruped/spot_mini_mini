#ifndef _ROS_flexbe_msgs_CommandFeedback_h
#define _ROS_flexbe_msgs_CommandFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class CommandFeedback : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      uint32_t args_length;
      typedef char* _args_type;
      _args_type st_args;
      _args_type * args;

    CommandFeedback():
      command(""),
      args_length(0), args(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      *(outbuffer + offset + 0) = (this->args_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->args_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->args_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->args_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->args_length);
      for( uint32_t i = 0; i < args_length; i++){
      uint32_t length_argsi = strlen(this->args[i]);
      varToArr(outbuffer + offset, length_argsi);
      offset += 4;
      memcpy(outbuffer + offset, this->args[i], length_argsi);
      offset += length_argsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      uint32_t args_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      args_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      args_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      args_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->args_length);
      if(args_lengthT > args_length)
        this->args = (char**)realloc(this->args, args_lengthT * sizeof(char*));
      args_length = args_lengthT;
      for( uint32_t i = 0; i < args_length; i++){
      uint32_t length_st_args;
      arrToVar(length_st_args, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_args; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_args-1]=0;
      this->st_args = (char *)(inbuffer + offset-1);
      offset += length_st_args;
        memcpy( &(this->args[i]), &(this->st_args), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/CommandFeedback"; };
    const char * getMD5(){ return "0c984ea7f8fc562a9c04a778877e1a03"; };

  };

}
#endif
