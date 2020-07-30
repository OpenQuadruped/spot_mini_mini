#ifndef _ROS_intera_core_msgs_IOComponentStatus_h
#define _ROS_intera_core_msgs_IOComponentStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "intera_core_msgs/IOStatus.h"

namespace intera_core_msgs
{

  class IOComponentStatus : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef intera_core_msgs::IOStatus _status_type;
      _status_type status;

    IOComponentStatus():
      name(""),
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
      offset += this->status.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IOComponentStatus"; };
    const char * getMD5(){ return "7daed407477edd76573df598b0375a48"; };

  };

}
#endif
