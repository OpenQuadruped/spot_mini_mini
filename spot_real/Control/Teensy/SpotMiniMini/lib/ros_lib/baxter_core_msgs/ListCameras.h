#ifndef _ROS_SERVICE_ListCameras_h
#define _ROS_SERVICE_ListCameras_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_core_msgs
{

static const char LISTCAMERAS[] = "baxter_core_msgs/ListCameras";

  class ListCamerasRequest : public ros::Msg
  {
    public:

    ListCamerasRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return LISTCAMERAS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListCamerasResponse : public ros::Msg
  {
    public:
      uint32_t cameras_length;
      typedef char* _cameras_type;
      _cameras_type st_cameras;
      _cameras_type * cameras;

    ListCamerasResponse():
      cameras_length(0), cameras(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->cameras_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cameras_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cameras_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cameras_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cameras_length);
      for( uint32_t i = 0; i < cameras_length; i++){
      uint32_t length_camerasi = strlen(this->cameras[i]);
      varToArr(outbuffer + offset, length_camerasi);
      offset += 4;
      memcpy(outbuffer + offset, this->cameras[i], length_camerasi);
      offset += length_camerasi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t cameras_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cameras_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cameras_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cameras_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cameras_length);
      if(cameras_lengthT > cameras_length)
        this->cameras = (char**)realloc(this->cameras, cameras_lengthT * sizeof(char*));
      cameras_length = cameras_lengthT;
      for( uint32_t i = 0; i < cameras_length; i++){
      uint32_t length_st_cameras;
      arrToVar(length_st_cameras, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_cameras; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_cameras-1]=0;
      this->st_cameras = (char *)(inbuffer + offset-1);
      offset += length_st_cameras;
        memcpy( &(this->cameras[i]), &(this->st_cameras), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return LISTCAMERAS; };
    const char * getMD5(){ return "855b31192ab61744e7deb992d94db7ff"; };

  };

  class ListCameras {
    public:
    typedef ListCamerasRequest Request;
    typedef ListCamerasResponse Response;
  };

}
#endif
