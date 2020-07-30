#ifndef _ROS_baxter_maintenance_msgs_UpdateStatus_h
#define _ROS_baxter_maintenance_msgs_UpdateStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_maintenance_msgs
{

  class UpdateStatus : public ros::Msg
  {
    public:
      typedef uint16_t _status_type;
      _status_type status;
      typedef float _progress_type;
      _progress_type progress;
      typedef const char* _long_description_type;
      _long_description_type long_description;
      enum { STS_IDLE =  0 };
      enum { STS_INVALID =  1 };
      enum { STS_BUSY =  2 };
      enum { STS_CANCELLED =  3 };
      enum { STS_ERR =  4 };
      enum { STS_MOUNT_UPDATE =  5 };
      enum { STS_VERIFY_UPDATE =  6 };
      enum { STS_PREP_STAGING =  7 };
      enum { STS_MOUNT_STAGING =  8 };
      enum { STS_EXTRACT_UPDATE =  9 };
      enum { STS_LOAD_KEXEC =  10 };

    UpdateStatus():
      status(0),
      progress(0),
      long_description("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      union {
        float real;
        uint32_t base;
      } u_progress;
      u_progress.real = this->progress;
      *(outbuffer + offset + 0) = (u_progress.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_progress.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_progress.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_progress.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->progress);
      uint32_t length_long_description = strlen(this->long_description);
      varToArr(outbuffer + offset, length_long_description);
      offset += 4;
      memcpy(outbuffer + offset, this->long_description, length_long_description);
      offset += length_long_description;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->status =  ((uint16_t) (*(inbuffer + offset)));
      this->status |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->status);
      union {
        float real;
        uint32_t base;
      } u_progress;
      u_progress.base = 0;
      u_progress.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_progress.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_progress.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_progress.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->progress = u_progress.real;
      offset += sizeof(this->progress);
      uint32_t length_long_description;
      arrToVar(length_long_description, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_long_description; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_long_description-1]=0;
      this->long_description = (char *)(inbuffer + offset-1);
      offset += length_long_description;
     return offset;
    }

    const char * getType(){ return "baxter_maintenance_msgs/UpdateStatus"; };
    const char * getMD5(){ return "74e246350421569590252c39e8aa7b85"; };

  };

}
#endif
