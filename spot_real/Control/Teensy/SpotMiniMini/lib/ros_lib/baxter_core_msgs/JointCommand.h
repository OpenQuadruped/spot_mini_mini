#ifndef _ROS_baxter_core_msgs_JointCommand_h
#define _ROS_baxter_core_msgs_JointCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_core_msgs
{

  class JointCommand : public ros::Msg
  {
    public:
      typedef int32_t _mode_type;
      _mode_type mode;
      uint32_t command_length;
      typedef double _command_type;
      _command_type st_command;
      _command_type * command;
      uint32_t names_length;
      typedef char* _names_type;
      _names_type st_names;
      _names_type * names;
      enum { POSITION_MODE = 1 };
      enum { VELOCITY_MODE = 2 };
      enum { TORQUE_MODE = 3 };
      enum { RAW_POSITION_MODE = 4 };

    JointCommand():
      mode(0),
      command_length(0), command(NULL),
      names_length(0), names(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode);
      *(outbuffer + offset + 0) = (this->command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_length);
      for( uint32_t i = 0; i < command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_commandi;
      u_commandi.real = this->command[i];
      *(outbuffer + offset + 0) = (u_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commandi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_commandi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_commandi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_commandi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_commandi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->command[i]);
      }
      *(outbuffer + offset + 0) = (this->names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->names_length);
      for( uint32_t i = 0; i < names_length; i++){
      uint32_t length_namesi = strlen(this->names[i]);
      varToArr(outbuffer + offset, length_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->names[i], length_namesi);
      offset += length_namesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      uint32_t command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->command_length);
      if(command_lengthT > command_length)
        this->command = (double*)realloc(this->command, command_lengthT * sizeof(double));
      command_length = command_lengthT;
      for( uint32_t i = 0; i < command_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_command;
      u_st_command.base = 0;
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_command.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_command = u_st_command.real;
      offset += sizeof(this->st_command);
        memcpy( &(this->command[i]), &(this->st_command), sizeof(double));
      }
      uint32_t names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->names_length);
      if(names_lengthT > names_length)
        this->names = (char**)realloc(this->names, names_lengthT * sizeof(char*));
      names_length = names_lengthT;
      for( uint32_t i = 0; i < names_length; i++){
      uint32_t length_st_names;
      arrToVar(length_st_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_names-1]=0;
      this->st_names = (char *)(inbuffer + offset-1);
      offset += length_st_names;
        memcpy( &(this->names[i]), &(this->st_names), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/JointCommand"; };
    const char * getMD5(){ return "19bfec8434dd568ab3c633d187c36f2e"; };

  };

}
#endif
