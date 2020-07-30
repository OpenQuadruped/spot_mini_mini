#ifndef _ROS_SERVICE_OpenCamera_h
#define _ROS_SERVICE_OpenCamera_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "baxter_core_msgs/CameraSettings.h"

namespace baxter_core_msgs
{

static const char OPENCAMERA[] = "baxter_core_msgs/OpenCamera";

  class OpenCameraRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef baxter_core_msgs::CameraSettings _settings_type;
      _settings_type settings;

    OpenCameraRequest():
      name(""),
      settings()
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
      offset += this->settings.serialize(outbuffer + offset);
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
      offset += this->settings.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return OPENCAMERA; };
    const char * getMD5(){ return "c4194eee32741c5a98ef8da9666fa6c9"; };

  };

  class OpenCameraResponse : public ros::Msg
  {
    public:
      typedef int32_t _err_type;
      _err_type err;

    OpenCameraResponse():
      err(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_err;
      u_err.real = this->err;
      *(outbuffer + offset + 0) = (u_err.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_err.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_err.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_err.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->err);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_err;
      u_err.base = 0;
      u_err.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_err.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_err.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_err.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->err = u_err.real;
      offset += sizeof(this->err);
     return offset;
    }

    const char * getType(){ return OPENCAMERA; };
    const char * getMD5(){ return "b6e094011a4dfaee5eddf447220446cf"; };

  };

  class OpenCamera {
    public:
    typedef OpenCameraRequest Request;
    typedef OpenCameraResponse Response;
  };

}
#endif
