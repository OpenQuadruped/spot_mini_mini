#ifndef _ROS_moveit_msgs_MotionSequenceItem_h
#define _ROS_moveit_msgs_MotionSequenceItem_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moveit_msgs/MotionPlanRequest.h"

namespace moveit_msgs
{

  class MotionSequenceItem : public ros::Msg
  {
    public:
      typedef moveit_msgs::MotionPlanRequest _req_type;
      _req_type req;
      typedef double _blend_radius_type;
      _blend_radius_type blend_radius;

    MotionSequenceItem():
      req(),
      blend_radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->req.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_blend_radius;
      u_blend_radius.real = this->blend_radius;
      *(outbuffer + offset + 0) = (u_blend_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blend_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blend_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blend_radius.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_blend_radius.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_blend_radius.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_blend_radius.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_blend_radius.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->blend_radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->req.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_blend_radius;
      u_blend_radius.base = 0;
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_blend_radius.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->blend_radius = u_blend_radius.real;
      offset += sizeof(this->blend_radius);
     return offset;
    }

    const char * getType(){ return "moveit_msgs/MotionSequenceItem"; };
    const char * getMD5(){ return "932aef4280f479e42c693b8b285624bf"; };

  };

}
#endif
