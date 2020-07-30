#ifndef _ROS_flexbe_msgs_BehaviorModification_h
#define _ROS_flexbe_msgs_BehaviorModification_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class BehaviorModification : public ros::Msg
  {
    public:
      typedef int32_t _index_begin_type;
      _index_begin_type index_begin;
      typedef int32_t _index_end_type;
      _index_end_type index_end;
      typedef const char* _new_content_type;
      _new_content_type new_content;

    BehaviorModification():
      index_begin(0),
      index_end(0),
      new_content("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_index_begin;
      u_index_begin.real = this->index_begin;
      *(outbuffer + offset + 0) = (u_index_begin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_index_begin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_index_begin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_index_begin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index_begin);
      union {
        int32_t real;
        uint32_t base;
      } u_index_end;
      u_index_end.real = this->index_end;
      *(outbuffer + offset + 0) = (u_index_end.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_index_end.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_index_end.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_index_end.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index_end);
      uint32_t length_new_content = strlen(this->new_content);
      varToArr(outbuffer + offset, length_new_content);
      offset += 4;
      memcpy(outbuffer + offset, this->new_content, length_new_content);
      offset += length_new_content;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_index_begin;
      u_index_begin.base = 0;
      u_index_begin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_index_begin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_index_begin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_index_begin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->index_begin = u_index_begin.real;
      offset += sizeof(this->index_begin);
      union {
        int32_t real;
        uint32_t base;
      } u_index_end;
      u_index_end.base = 0;
      u_index_end.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_index_end.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_index_end.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_index_end.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->index_end = u_index_end.real;
      offset += sizeof(this->index_end);
      uint32_t length_new_content;
      arrToVar(length_new_content, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_new_content; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_new_content-1]=0;
      this->new_content = (char *)(inbuffer + offset-1);
      offset += length_new_content;
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorModification"; };
    const char * getMD5(){ return "ac997193d826b145a432b8e3e528f6b4"; };

  };

}
#endif
