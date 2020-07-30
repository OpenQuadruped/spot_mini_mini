#ifndef _ROS_intera_core_msgs_IOComponentConfiguration_h
#define _ROS_intera_core_msgs_IOComponentConfiguration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class IOComponentConfiguration : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _config_type;
      _config_type config;

    IOComponentConfiguration():
      name(""),
      config("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_config = strlen(this->config);
      varToArr(outbuffer + offset, length_config);
      offset += 4;
      memcpy(outbuffer + offset, this->config, length_config);
      offset += length_config;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_config;
      arrToVar(length_config, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_config; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_config-1]=0;
      this->config = (char *)(inbuffer + offset-1);
      offset += length_config;
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IOComponentConfiguration"; };
    const char * getMD5(){ return "cb7717d13a521b51b5c0a02d493c42fd"; };

  };

}
#endif
