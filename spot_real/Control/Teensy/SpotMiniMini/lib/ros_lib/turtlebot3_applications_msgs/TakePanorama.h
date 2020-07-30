#ifndef _ROS_SERVICE_TakePanorama_h
#define _ROS_SERVICE_TakePanorama_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot3_applications_msgs
{

static const char TAKEPANORAMA[] = "turtlebot3_applications_msgs/TakePanorama";

  class TakePanoramaRequest : public ros::Msg
  {
    public:
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef float _pano_angle_type;
      _pano_angle_type pano_angle;
      typedef float _snap_interval_type;
      _snap_interval_type snap_interval;
      typedef float _rot_vel_type;
      _rot_vel_type rot_vel;
      enum { SNAPANDROTATE = 0  };
      enum { CONTINUOUS = 1     };
      enum { STOP = 2           };
      enum { STARTED = 0        };
      enum { IN_PROGRESS = 1    };
      enum { STOPPED = 2        };

    TakePanoramaRequest():
      mode(0),
      pano_angle(0),
      snap_interval(0),
      rot_vel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_pano_angle;
      u_pano_angle.real = this->pano_angle;
      *(outbuffer + offset + 0) = (u_pano_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pano_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pano_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pano_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pano_angle);
      union {
        float real;
        uint32_t base;
      } u_snap_interval;
      u_snap_interval.real = this->snap_interval;
      *(outbuffer + offset + 0) = (u_snap_interval.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_snap_interval.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_snap_interval.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_snap_interval.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->snap_interval);
      union {
        float real;
        uint32_t base;
      } u_rot_vel;
      u_rot_vel.real = this->rot_vel;
      *(outbuffer + offset + 0) = (u_rot_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rot_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rot_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rot_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rot_vel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_pano_angle;
      u_pano_angle.base = 0;
      u_pano_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pano_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pano_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pano_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pano_angle = u_pano_angle.real;
      offset += sizeof(this->pano_angle);
      union {
        float real;
        uint32_t base;
      } u_snap_interval;
      u_snap_interval.base = 0;
      u_snap_interval.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_snap_interval.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_snap_interval.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_snap_interval.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->snap_interval = u_snap_interval.real;
      offset += sizeof(this->snap_interval);
      union {
        float real;
        uint32_t base;
      } u_rot_vel;
      u_rot_vel.base = 0;
      u_rot_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rot_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rot_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rot_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rot_vel = u_rot_vel.real;
      offset += sizeof(this->rot_vel);
     return offset;
    }

    const char * getType(){ return TAKEPANORAMA; };
    const char * getMD5(){ return "ee720ee47c4798b7447cb7a5755b0062"; };

  };

  class TakePanoramaResponse : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;

    TakePanoramaResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return TAKEPANORAMA; };
    const char * getMD5(){ return "284aa12dd9e9e760802ac9f38036ea5e"; };

  };

  class TakePanorama {
    public:
    typedef TakePanoramaRequest Request;
    typedef TakePanoramaResponse Response;
  };

}
#endif
