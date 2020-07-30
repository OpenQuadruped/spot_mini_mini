#ifndef _ROS_flexbe_msgs_BEStatus_h
#define _ROS_flexbe_msgs_BEStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace flexbe_msgs
{

  class BEStatus : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int32_t _behavior_id_type;
      _behavior_id_type behavior_id;
      typedef uint8_t _code_type;
      _code_type code;
      uint32_t args_length;
      typedef char* _args_type;
      _args_type st_args;
      _args_type * args;
      enum { STARTED =  0 };
      enum { FINISHED =  1 };
      enum { FAILED =  2 };
      enum { LOCKED =  4 };
      enum { WAITING =  5 };
      enum { SWITCHING =  6 };
      enum { WARNING =  10 };
      enum { ERROR =  11 };
      enum { READY =  20 };

    BEStatus():
      stamp(),
      behavior_id(0),
      code(0),
      args_length(0), args(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_id;
      u_behavior_id.real = this->behavior_id;
      *(outbuffer + offset + 0) = (u_behavior_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_behavior_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_behavior_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_behavior_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->behavior_id);
      *(outbuffer + offset + 0) = (this->code >> (8 * 0)) & 0xFF;
      offset += sizeof(this->code);
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
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_id;
      u_behavior_id.base = 0;
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->behavior_id = u_behavior_id.real;
      offset += sizeof(this->behavior_id);
      this->code =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->code);
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

    const char * getType(){ return "flexbe_msgs/BEStatus"; };
    const char * getMD5(){ return "467355de4ad406f864268f41621cb885"; };

  };

}
#endif
