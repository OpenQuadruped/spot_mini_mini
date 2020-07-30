#ifndef _ROS_moveit_msgs_LinkScale_h
#define _ROS_moveit_msgs_LinkScale_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moveit_msgs
{

  class LinkScale : public ros::Msg
  {
    public:
      typedef const char* _link_name_type;
      _link_name_type link_name;
      typedef double _scale_type;
      _scale_type scale;

    LinkScale():
      link_name(""),
      scale(0)
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
      } u_scale;
      u_scale.real = this->scale;
      *(outbuffer + offset + 0) = (u_scale.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_scale.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_scale.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_scale.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_scale.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_scale.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_scale.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_scale.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->scale);
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
      } u_scale;
      u_scale.base = 0;
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_scale.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->scale = u_scale.real;
      offset += sizeof(this->scale);
     return offset;
    }

    const char * getType(){ return "moveit_msgs/LinkScale"; };
    const char * getMD5(){ return "19faf226446bfb2f645a4da6f2a56166"; };

  };

}
#endif
