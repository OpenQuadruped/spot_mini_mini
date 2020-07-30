#ifndef _ROS_SERVICE_SetPidGains_h
#define _ROS_SERVICE_SetPidGains_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace control_toolbox
{

static const char SETPIDGAINS[] = "control_toolbox/SetPidGains";

  class SetPidGainsRequest : public ros::Msg
  {
    public:
      typedef double _p_type;
      _p_type p;
      typedef double _i_type;
      _i_type i;
      typedef double _d_type;
      _d_type d;
      typedef double _i_clamp_type;
      _i_clamp_type i_clamp;
      typedef bool _antiwindup_type;
      _antiwindup_type antiwindup;

    SetPidGainsRequest():
      p(0),
      i(0),
      d(0),
      i_clamp(0),
      antiwindup(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_p;
      u_p.real = this->p;
      *(outbuffer + offset + 0) = (u_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_p.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_p.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_p.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_p.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->p);
      union {
        double real;
        uint64_t base;
      } u_i;
      u_i.real = this->i;
      *(outbuffer + offset + 0) = (u_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i);
      union {
        double real;
        uint64_t base;
      } u_d;
      u_d.real = this->d;
      *(outbuffer + offset + 0) = (u_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_d.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_d.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_d.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_d.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->d);
      union {
        double real;
        uint64_t base;
      } u_i_clamp;
      u_i_clamp.real = this->i_clamp;
      *(outbuffer + offset + 0) = (u_i_clamp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_clamp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_clamp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_clamp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_clamp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_clamp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_clamp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_clamp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_clamp);
      union {
        bool real;
        uint8_t base;
      } u_antiwindup;
      u_antiwindup.real = this->antiwindup;
      *(outbuffer + offset + 0) = (u_antiwindup.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->antiwindup);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_p;
      u_p.base = 0;
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_p.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->p = u_p.real;
      offset += sizeof(this->p);
      union {
        double real;
        uint64_t base;
      } u_i;
      u_i.base = 0;
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i = u_i.real;
      offset += sizeof(this->i);
      union {
        double real;
        uint64_t base;
      } u_d;
      u_d.base = 0;
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_d.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->d = u_d.real;
      offset += sizeof(this->d);
      union {
        double real;
        uint64_t base;
      } u_i_clamp;
      u_i_clamp.base = 0;
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i_clamp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i_clamp = u_i_clamp.real;
      offset += sizeof(this->i_clamp);
      union {
        bool real;
        uint8_t base;
      } u_antiwindup;
      u_antiwindup.base = 0;
      u_antiwindup.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->antiwindup = u_antiwindup.real;
      offset += sizeof(this->antiwindup);
     return offset;
    }

    const char * getType(){ return SETPIDGAINS; };
    const char * getMD5(){ return "4a43159879643e60937bf2893b633607"; };

  };

  class SetPidGainsResponse : public ros::Msg
  {
    public:

    SetPidGainsResponse()
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

    const char * getType(){ return SETPIDGAINS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPidGains {
    public:
    typedef SetPidGainsRequest Request;
    typedef SetPidGainsResponse Response;
  };

}
#endif
