#ifndef _ROS_intera_core_msgs_IODataStatus_h
#define _ROS_intera_core_msgs_IODataStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "intera_core_msgs/IOStatus.h"

namespace intera_core_msgs
{

  class IODataStatus : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _format_type;
      _format_type format;
      typedef const char* _data_type;
      _data_type data;
      typedef intera_core_msgs::IOStatus _status_type;
      _status_type status;

    IODataStatus():
      name(""),
      format(""),
      data(""),
      status()
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
      uint32_t length_format = strlen(this->format);
      varToArr(outbuffer + offset, length_format);
      offset += 4;
      memcpy(outbuffer + offset, this->format, length_format);
      offset += length_format;
      uint32_t length_data = strlen(this->data);
      varToArr(outbuffer + offset, length_data);
      offset += 4;
      memcpy(outbuffer + offset, this->data, length_data);
      offset += length_data;
      offset += this->status.serialize(outbuffer + offset);
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
      uint32_t length_format;
      arrToVar(length_format, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_format; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_format-1]=0;
      this->format = (char *)(inbuffer + offset-1);
      offset += length_format;
      uint32_t length_data;
      arrToVar(length_data, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_data; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_data-1]=0;
      this->data = (char *)(inbuffer + offset-1);
      offset += length_data;
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IODataStatus"; };
    const char * getMD5(){ return "bb31283c6afc4ddea2f5f157264e5909"; };

  };

}
#endif
