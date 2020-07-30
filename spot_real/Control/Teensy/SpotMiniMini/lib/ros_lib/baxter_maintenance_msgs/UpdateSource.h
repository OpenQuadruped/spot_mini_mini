#ifndef _ROS_baxter_maintenance_msgs_UpdateSource_h
#define _ROS_baxter_maintenance_msgs_UpdateSource_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_maintenance_msgs
{

  class UpdateSource : public ros::Msg
  {
    public:
      typedef const char* _devname_type;
      _devname_type devname;
      typedef const char* _filename_type;
      _filename_type filename;
      typedef const char* _version_type;
      _version_type version;
      typedef const char* _uuid_type;
      _uuid_type uuid;

    UpdateSource():
      devname(""),
      filename(""),
      version(""),
      uuid("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_devname = strlen(this->devname);
      varToArr(outbuffer + offset, length_devname);
      offset += 4;
      memcpy(outbuffer + offset, this->devname, length_devname);
      offset += length_devname;
      uint32_t length_filename = strlen(this->filename);
      varToArr(outbuffer + offset, length_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      uint32_t length_version = strlen(this->version);
      varToArr(outbuffer + offset, length_version);
      offset += 4;
      memcpy(outbuffer + offset, this->version, length_version);
      offset += length_version;
      uint32_t length_uuid = strlen(this->uuid);
      varToArr(outbuffer + offset, length_uuid);
      offset += 4;
      memcpy(outbuffer + offset, this->uuid, length_uuid);
      offset += length_uuid;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_devname;
      arrToVar(length_devname, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_devname; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_devname-1]=0;
      this->devname = (char *)(inbuffer + offset-1);
      offset += length_devname;
      uint32_t length_filename;
      arrToVar(length_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
      uint32_t length_version;
      arrToVar(length_version, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_version; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_version-1]=0;
      this->version = (char *)(inbuffer + offset-1);
      offset += length_version;
      uint32_t length_uuid;
      arrToVar(length_uuid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uuid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uuid-1]=0;
      this->uuid = (char *)(inbuffer + offset-1);
      offset += length_uuid;
     return offset;
    }

    const char * getType(){ return "baxter_maintenance_msgs/UpdateSource"; };
    const char * getMD5(){ return "88ad69e3ed4d619e167c9d83e6d9310f"; };

  };

}
#endif
