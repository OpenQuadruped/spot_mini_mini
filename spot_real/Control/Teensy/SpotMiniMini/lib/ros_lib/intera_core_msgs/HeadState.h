#ifndef _ROS_intera_core_msgs_HeadState_h
#define _ROS_intera_core_msgs_HeadState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class HeadState : public ros::Msg
  {
    public:
      typedef float _pan_type;
      _pan_type pan;
      typedef bool _isTurning_type;
      _isTurning_type isTurning;
      typedef bool _isBlocked_type;
      _isBlocked_type isBlocked;
      typedef uint8_t _panMode_type;
      _panMode_type panMode;
      enum { PASSIVE_MODE =  0 };
      enum { ACTIVE_MODE =  1 };
      enum { ACTIVE_CANCELLATION_MODE =  2 };

    HeadState():
      pan(0),
      isTurning(0),
      isBlocked(0),
      panMode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan;
      u_pan.real = this->pan;
      *(outbuffer + offset + 0) = (u_pan.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pan.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pan.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pan.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pan);
      union {
        bool real;
        uint8_t base;
      } u_isTurning;
      u_isTurning.real = this->isTurning;
      *(outbuffer + offset + 0) = (u_isTurning.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isTurning);
      union {
        bool real;
        uint8_t base;
      } u_isBlocked;
      u_isBlocked.real = this->isBlocked;
      *(outbuffer + offset + 0) = (u_isBlocked.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isBlocked);
      *(outbuffer + offset + 0) = (this->panMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->panMode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pan;
      u_pan.base = 0;
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pan.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pan = u_pan.real;
      offset += sizeof(this->pan);
      union {
        bool real;
        uint8_t base;
      } u_isTurning;
      u_isTurning.base = 0;
      u_isTurning.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isTurning = u_isTurning.real;
      offset += sizeof(this->isTurning);
      union {
        bool real;
        uint8_t base;
      } u_isBlocked;
      u_isBlocked.base = 0;
      u_isBlocked.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isBlocked = u_isBlocked.real;
      offset += sizeof(this->isBlocked);
      this->panMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->panMode);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/HeadState"; };
    const char * getMD5(){ return "51024ade10ffefe117049c9ba6fd743c"; };

  };

}
#endif
