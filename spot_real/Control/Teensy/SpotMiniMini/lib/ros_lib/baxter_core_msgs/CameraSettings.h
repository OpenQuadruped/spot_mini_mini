#ifndef _ROS_baxter_core_msgs_CameraSettings_h
#define _ROS_baxter_core_msgs_CameraSettings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "baxter_core_msgs/CameraControl.h"

namespace baxter_core_msgs
{

  class CameraSettings : public ros::Msg
  {
    public:
      typedef int32_t _width_type;
      _width_type width;
      typedef int32_t _height_type;
      _height_type height;
      typedef float _fps_type;
      _fps_type fps;
      uint32_t controls_length;
      typedef baxter_core_msgs::CameraControl _controls_type;
      _controls_type st_controls;
      _controls_type * controls;

    CameraSettings():
      width(0),
      height(0),
      fps(0),
      controls_length(0), controls(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        float real;
        uint32_t base;
      } u_fps;
      u_fps.real = this->fps;
      *(outbuffer + offset + 0) = (u_fps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fps);
      *(outbuffer + offset + 0) = (this->controls_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->controls_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->controls_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->controls_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->controls_length);
      for( uint32_t i = 0; i < controls_length; i++){
      offset += this->controls[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        float real;
        uint32_t base;
      } u_fps;
      u_fps.base = 0;
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fps = u_fps.real;
      offset += sizeof(this->fps);
      uint32_t controls_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      controls_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      controls_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      controls_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->controls_length);
      if(controls_lengthT > controls_length)
        this->controls = (baxter_core_msgs::CameraControl*)realloc(this->controls, controls_lengthT * sizeof(baxter_core_msgs::CameraControl));
      controls_length = controls_lengthT;
      for( uint32_t i = 0; i < controls_length; i++){
      offset += this->st_controls.deserialize(inbuffer + offset);
        memcpy( &(this->controls[i]), &(this->st_controls), sizeof(baxter_core_msgs::CameraControl));
      }
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/CameraSettings"; };
    const char * getMD5(){ return "d133bef4a3bd9a6e490a5dc91d20f429"; };

  };

}
#endif
