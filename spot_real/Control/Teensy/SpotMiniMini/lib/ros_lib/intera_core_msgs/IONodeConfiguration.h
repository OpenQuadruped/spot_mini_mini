#ifndef _ROS_intera_core_msgs_IONodeConfiguration_h
#define _ROS_intera_core_msgs_IONodeConfiguration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "intera_core_msgs/IOComponentConfiguration.h"

namespace intera_core_msgs
{

  class IONodeConfiguration : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef intera_core_msgs::IOComponentConfiguration _node_type;
      _node_type node;
      uint32_t devices_length;
      typedef intera_core_msgs::IOComponentConfiguration _devices_type;
      _devices_type st_devices;
      _devices_type * devices;
      uint32_t plugins_length;
      typedef intera_core_msgs::IOComponentConfiguration _plugins_type;
      _plugins_type st_plugins;
      _plugins_type * plugins;

    IONodeConfiguration():
      time(),
      node(),
      devices_length(0), devices(NULL),
      plugins_length(0), plugins(NULL)
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
      *(outbuffer + offset + 0) = (this->plugins_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->plugins_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->plugins_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->plugins_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->plugins_length);
      for( uint32_t i = 0; i < plugins_length; i++){
      offset += this->plugins[i].serialize(outbuffer + offset);
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
        this->devices = (intera_core_msgs::IOComponentConfiguration*)realloc(this->devices, devices_lengthT * sizeof(intera_core_msgs::IOComponentConfiguration));
      devices_length = devices_lengthT;
      for( uint32_t i = 0; i < devices_length; i++){
      offset += this->st_devices.deserialize(inbuffer + offset);
        memcpy( &(this->devices[i]), &(this->st_devices), sizeof(intera_core_msgs::IOComponentConfiguration));
      }
      uint32_t plugins_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      plugins_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      plugins_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      plugins_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->plugins_length);
      if(plugins_lengthT > plugins_length)
        this->plugins = (intera_core_msgs::IOComponentConfiguration*)realloc(this->plugins, plugins_lengthT * sizeof(intera_core_msgs::IOComponentConfiguration));
      plugins_length = plugins_lengthT;
      for( uint32_t i = 0; i < plugins_length; i++){
      offset += this->st_plugins.deserialize(inbuffer + offset);
        memcpy( &(this->plugins[i]), &(this->st_plugins), sizeof(intera_core_msgs::IOComponentConfiguration));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IONodeConfiguration"; };
    const char * getMD5(){ return "66800b32dbb52df00e2454d58668ac33"; };

  };

}
#endif
