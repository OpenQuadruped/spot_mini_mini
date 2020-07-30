#ifndef _ROS_intera_core_msgs_IOStatus_h
#define _ROS_intera_core_msgs_IOStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class IOStatus : public ros::Msg
  {
    public:
      typedef const char* _tag_type;
      _tag_type tag;
      typedef const char* _id_type;
      _id_type id;
      typedef const char* _detail_type;
      _detail_type detail;
      enum { DOWN =  down };
      enum { READY =  ready };
      enum { BUSY =  busy };
      enum { UNREADY =  unready };
      enum { ERROR =  error };

    IOStatus():
      tag(""),
      id(""),
      detail("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_tag = strlen(this->tag);
      varToArr(outbuffer + offset, length_tag);
      offset += 4;
      memcpy(outbuffer + offset, this->tag, length_tag);
      offset += length_tag;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      uint32_t length_detail = strlen(this->detail);
      varToArr(outbuffer + offset, length_detail);
      offset += 4;
      memcpy(outbuffer + offset, this->detail, length_detail);
      offset += length_detail;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_tag;
      arrToVar(length_tag, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_tag; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_tag-1]=0;
      this->tag = (char *)(inbuffer + offset-1);
      offset += length_tag;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      uint32_t length_detail;
      arrToVar(length_detail, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_detail; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_detail-1]=0;
      this->detail = (char *)(inbuffer + offset-1);
      offset += length_detail;
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/IOStatus"; };
    const char * getMD5(){ return "a8daeb84c9abffc88ad8ca636f5fd8a0"; };

  };

}
#endif
