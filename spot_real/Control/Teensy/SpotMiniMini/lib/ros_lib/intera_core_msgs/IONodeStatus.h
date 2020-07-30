#ifndef _ROS_intera_core_msgs_IONodeStatus_h
#define _ROS_intera_core_msgs_IONodeStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "intera_core_msgs/IOComponentStatus.h"

namespace intera_core_msgs
{

  class IONodeStatus : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef intera_core_msgs::IOComponentStatus _node_type;
      _node_type node;
      uint32_t devices_length;
      typedef intera_core_msgs::IOComponentStatus _devices_type;
      _devices_type st_devices;
      _devices_type * devices;
      uint32_t commands_length;
      typedef ros::Time _commands_type;
      _commands_type st_commands;
      _commands_type * commands;

    IONodeStatus():
      time(),
      node(),
      devices_length(0), devices(NULL),
      commands_length(0), commands(NULL)
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
      offset += this->node.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->devices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->devices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->devices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->devices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->devices_length);
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->devices[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->commands_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commands_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commands_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commands_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commands_length);
      for( uint32_t i = 0; i < commands_length; i++){
      *(outbuffer + offset + 0) = (this->commands[i].sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commands[i].sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commands[i].sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commands[i].sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commands[i].sec);
      *(outbuffer + offset + 0) = (this->commands[i].nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->commands[i].nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->commands[i].nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->commands[i].nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->commands[i].nsec);
      }
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
      offset += this->node.deserialize(inbuffer + offset);
      uint32_t devices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      devices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->devices_length);
      if(devices_lengthT > devices_length)
        this->devices = (intera_core_msgs::IOComponentStatus*)realloc(this->devices, devices_lengthT * sizeof(intera_core_msgs::IOComponentStatus));
      devices_length = devices_lengthT;
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->st_devices.deserialize(inbuffer + offset);
        memcpy( &(this->devices[i]), &(this->st_devices), sizeof(intera_core_msgs::IOComponentStatus));
      }
      uint32_t commands_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      commands_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      commands_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      commands_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->commands_length);
      if(commands_lengthT > commands_length)
        this->commands = (ros::Time*)realloc(this->commands, commands_lengthT * sizeof(ros::Time));
      commands_length = commands_lengthT;
      for( uint32_t i = 0; i < commands_length; i++){
      this->st_commands.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_commands.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_commands.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_commands.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_commands.sec);
      this->st_commands.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->st_commands.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_commands.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_commands.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_commands.nsec);
        memcpy( &(this->commands[i]), &(this->st_commands), sizeof(ros::Time));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IONodeStatus"; };
    const char * getMD5(){ return "260fce3c02f43bd977c92642b3c09c1d"; };

  };

}
#endif
