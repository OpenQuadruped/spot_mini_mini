#ifndef _ROS_baxter_core_msgs_EndEffectorCommand_h
#define _ROS_baxter_core_msgs_EndEffectorCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_core_msgs
{

  class EndEffectorCommand : public ros::Msg
  {
    public:
      typedef uint32_t _id_type;
      _id_type id;
      typedef const char* _command_type;
      _command_type command;
      typedef const char* _args_type;
      _args_type args;
      typedef const char* _sender_type;
      _sender_type sender;
      typedef uint32_t _sequence_type;
      _sequence_type sequence;
      enum { CMD_NO_OP =  no_op };
      enum { CMD_SET =  set };
      enum { CMD_CONFIGURE =  configure };
      enum { CMD_REBOOT =  reboot };
      enum { CMD_RESET =  reset };
      enum { CMD_CALIBRATE =  calibrate };
      enum { CMD_CLEAR_CALIBRATION =  clear_calibration };
      enum { CMD_PREPARE_TO_GRIP =  prepare_to_grip };
      enum { CMD_GRIP =  grip };
      enum { CMD_RELEASE =  release };
      enum { CMD_GO =  go };
      enum { CMD_STOP =  stop };

    EndEffectorCommand():
      id(0),
      command(""),
      args(""),
      sender(""),
      sequence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      uint32_t length_args = strlen(this->args);
      varToArr(outbuffer + offset, length_args);
      offset += 4;
      memcpy(outbuffer + offset, this->args, length_args);
      offset += length_args;
      uint32_t length_sender = strlen(this->sender);
      varToArr(outbuffer + offset, length_sender);
      offset += 4;
      memcpy(outbuffer + offset, this->sender, length_sender);
      offset += length_sender;
      *(outbuffer + offset + 0) = (this->sequence >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sequence >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sequence >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sequence >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sequence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      uint32_t length_args;
      arrToVar(length_args, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_args; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_args-1]=0;
      this->args = (char *)(inbuffer + offset-1);
      offset += length_args;
      uint32_t length_sender;
      arrToVar(length_sender, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sender; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sender-1]=0;
      this->sender = (char *)(inbuffer + offset-1);
      offset += length_sender;
      this->sequence =  ((uint32_t) (*(inbuffer + offset)));
      this->sequence |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sequence |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sequence |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sequence);
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/EndEffectorCommand"; };
    const char * getMD5(){ return "c003234e90416f2ca02ac7837c42cbb7"; };

  };

}
#endif
