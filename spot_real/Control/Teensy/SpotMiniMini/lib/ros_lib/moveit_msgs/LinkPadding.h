#ifndef _ROS_moveit_msgs_LinkPadding_h
#define _ROS_moveit_msgs_LinkPadding_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moveit_msgs
{

  class LinkPadding : public ros::Msg
  {
    public:
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef double _padding_type;
      _padding_type padding;

    LinkPadding():
      link_name(""),
      padding(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_link_name = strlen(this->link_name);
      varToArr(outbuffer + offset, length_link_name);
      offset += 4;
      memcpy(outbuffer + offset, this->link_name, length_link_name);
      offset += length_link_name;
      union {
        double real;
        uint64_t base;
      } u_padding;
      u_padding.real = this->padding;
      *(outbuffer + offset + 0) = (u_padding.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_padding.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_padding.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_padding.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_padding.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_padding.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_padding.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_padding.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->padding);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_link_name;
      arrToVar(length_link_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link_name-1]=0;
      this->link_name = (char *)(inbuffer + offset-1);
      offset += length_link_name;
      union {
        double real;
        uint64_t base;
      } u_padding;
      u_padding.base = 0;
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_padding.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->padding = u_padding.real;
      offset += sizeof(this->padding);
     return offset;
    }

    const char * getType(){ return "moveit_msgs/LinkPadding"; };
    const char * getMD5(){ return "b3ea75670df55c696fedee97774d5947"; };

  };

}
#endif
