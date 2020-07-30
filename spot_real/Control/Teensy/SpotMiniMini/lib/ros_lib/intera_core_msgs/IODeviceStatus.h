#ifndef _ROS_intera_core_msgs_IODeviceStatus_h
#define _ROS_intera_core_msgs_IODeviceStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "intera_core_msgs/IOComponentStatus.h"
#include "intera_core_msgs/IODataStatus.h"

namespace intera_core_msgs
{

  class IODeviceStatus : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef intera_core_msgs::IOComponentStatus _device_type;
      _device_type device;
      uint32_t ports_length;
      typedef intera_core_msgs::IODataStatus _ports_type;
      _ports_type st_ports;
      _ports_type * ports;
      uint32_t signals_length;
      typedef intera_core_msgs::IODataStatus _signals_type;
      _signals_type st_signals;
      _signals_type * signals;
      uint32_t commands_length;
      typedef ros::Time _commands_type;
      _commands_type st_commands;
      _commands_type * commands;
      uint32_t responses_length;
      typedef char* _responses_type;
      _responses_type st_responses;
      _responses_type * responses;

    IODeviceStatus():
      time(),
      device(),
      ports_length(0), ports(NULL),
      signals_length(0), signals(NULL),
      commands_length(0), commands(NULL),
      responses_length(0), responses(NULL)
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
      offset += this->device.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->ports_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ports_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ports_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ports_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ports_length);
      for( uint32_t i = 0; i < ports_length; i++){
      offset += this->ports[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->signals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->signals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->signals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->signals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->signals_length);
      for( uint32_t i = 0; i < signals_length; i++){
      offset += this->signals[i].serialize(outbuffer + offset);
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
      *(outbuffer + offset + 0) = (this->responses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->responses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->responses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->responses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->responses_length);
      for( uint32_t i = 0; i < responses_length; i++){
      uint32_t length_responsesi = strlen(this->responses[i]);
      varToArr(outbuffer + offset, length_responsesi);
      offset += 4;
      memcpy(outbuffer + offset, this->responses[i], length_responsesi);
      offset += length_responsesi;
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
      offset += this->device.deserialize(inbuffer + offset);
      uint32_t ports_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ports_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ports_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ports_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ports_length);
      if(ports_lengthT > ports_length)
        this->ports = (intera_core_msgs::IODataStatus*)realloc(this->ports, ports_lengthT * sizeof(intera_core_msgs::IODataStatus));
      ports_length = ports_lengthT;
      for( uint32_t i = 0; i < ports_length; i++){
      offset += this->st_ports.deserialize(inbuffer + offset);
        memcpy( &(this->ports[i]), &(this->st_ports), sizeof(intera_core_msgs::IODataStatus));
      }
      uint32_t signals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->signals_length);
      if(signals_lengthT > signals_length)
        this->signals = (intera_core_msgs::IODataStatus*)realloc(this->signals, signals_lengthT * sizeof(intera_core_msgs::IODataStatus));
      signals_length = signals_lengthT;
      for( uint32_t i = 0; i < signals_length; i++){
      offset += this->st_signals.deserialize(inbuffer + offset);
        memcpy( &(this->signals[i]), &(this->st_signals), sizeof(intera_core_msgs::IODataStatus));
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
      uint32_t responses_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      responses_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      responses_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      responses_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->responses_length);
      if(responses_lengthT > responses_length)
        this->responses = (char**)realloc(this->responses, responses_lengthT * sizeof(char*));
      responses_length = responses_lengthT;
      for( uint32_t i = 0; i < responses_length; i++){
      uint32_t length_st_responses;
      arrToVar(length_st_responses, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_responses; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_responses-1]=0;
      this->st_responses = (char *)(inbuffer + offset-1);
      offset += length_st_responses;
        memcpy( &(this->responses[i]), &(this->st_responses), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IODeviceStatus"; };
    const char * getMD5(){ return "0d0c97a2d700848e7ad46e09a77cf896"; };

  };

}
#endif
