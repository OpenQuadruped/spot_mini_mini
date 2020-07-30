#ifndef _ROS_intera_core_msgs_IODeviceConfiguration_h
#define _ROS_intera_core_msgs_IODeviceConfiguration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "intera_core_msgs/IOComponentConfiguration.h"

namespace intera_core_msgs
{

  class IODeviceConfiguration : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef const char* _commanded_type;
      _commanded_type commanded;
      typedef const char* _upgraded_type;
      _upgraded_type upgraded;
      typedef intera_core_msgs::IOComponentConfiguration _device_type;
      _device_type device;
      uint32_t ports_length;
      typedef intera_core_msgs::IOComponentConfiguration _ports_type;
      _ports_type st_ports;
      _ports_type * ports;
      uint32_t signals_length;
      typedef intera_core_msgs::IOComponentConfiguration _signals_type;
      _signals_type st_signals;
      _signals_type * signals;

    IODeviceConfiguration():
      time(),
      commanded(""),
      upgraded(""),
      device(),
      ports_length(0), ports(NULL),
      signals_length(0), signals(NULL)
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
      uint32_t length_commanded = strlen(this->commanded);
      varToArr(outbuffer + offset, length_commanded);
      offset += 4;
      memcpy(outbuffer + offset, this->commanded, length_commanded);
      offset += length_commanded;
      uint32_t length_upgraded = strlen(this->upgraded);
      varToArr(outbuffer + offset, length_upgraded);
      offset += 4;
      memcpy(outbuffer + offset, this->upgraded, length_upgraded);
      offset += length_upgraded;
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
      uint32_t length_commanded;
      arrToVar(length_commanded, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_commanded; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_commanded-1]=0;
      this->commanded = (char *)(inbuffer + offset-1);
      offset += length_commanded;
      uint32_t length_upgraded;
      arrToVar(length_upgraded, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_upgraded; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_upgraded-1]=0;
      this->upgraded = (char *)(inbuffer + offset-1);
      offset += length_upgraded;
      offset += this->device.deserialize(inbuffer + offset);
      uint32_t ports_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ports_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ports_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ports_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ports_length);
      if(ports_lengthT > ports_length)
        this->ports = (intera_core_msgs::IOComponentConfiguration*)realloc(this->ports, ports_lengthT * sizeof(intera_core_msgs::IOComponentConfiguration));
      ports_length = ports_lengthT;
      for( uint32_t i = 0; i < ports_length; i++){
      offset += this->st_ports.deserialize(inbuffer + offset);
        memcpy( &(this->ports[i]), &(this->st_ports), sizeof(intera_core_msgs::IOComponentConfiguration));
      }
      uint32_t signals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      signals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->signals_length);
      if(signals_lengthT > signals_length)
        this->signals = (intera_core_msgs::IOComponentConfiguration*)realloc(this->signals, signals_lengthT * sizeof(intera_core_msgs::IOComponentConfiguration));
      signals_length = signals_lengthT;
      for( uint32_t i = 0; i < signals_length; i++){
      offset += this->st_signals.deserialize(inbuffer + offset);
        memcpy( &(this->signals[i]), &(this->st_signals), sizeof(intera_core_msgs::IOComponentConfiguration));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IODeviceConfiguration"; };
    const char * getMD5(){ return "6757fad6217033498191470cb08f1674"; };

  };

}
#endif
