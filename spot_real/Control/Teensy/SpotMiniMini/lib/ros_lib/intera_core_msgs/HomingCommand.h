#ifndef _ROS_intera_core_msgs_HomingCommand_h
#define _ROS_intera_core_msgs_HomingCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class HomingCommand : public ros::Msg
  {
    public:
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t command_length;
      typedef int32_t _command_type;
      _command_type st_command;
      _command_type * command;
      enum { MANUAL = 1 };
      enum { AUTO = 2 };
      enum { NONE = 0 };

    HomingCommand():
      name_length(0), name(NULL),
      command_length(0), command(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->command_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->command_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->command_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_length);
      for( uint32_t i = 0; i < command_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_commandi;
      u_commandi.real = this->command[i];
      *(outbuffer + offset + 0) = (u_commandi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_commandi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_commandi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_commandi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t command_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      command_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->command_length);
      if(command_lengthT > command_length)
        this->command = (int32_t*)realloc(this->command, command_lengthT * sizeof(int32_t));
      command_length = command_lengthT;
      for( uint32_t i = 0; i < command_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_command;
      u_st_command.base = 0;
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_command.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_command = u_st_command.real;
      offset += sizeof(this->st_command);
        memcpy( &(this->command[i]), &(this->st_command), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/HomingCommand"; };
    const char * getMD5(){ return "ff29c86109f0f4cada5bbde945dd55c4"; };

  };

}
#endif
