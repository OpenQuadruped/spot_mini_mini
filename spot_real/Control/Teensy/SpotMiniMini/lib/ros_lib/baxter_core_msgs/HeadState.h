#ifndef _ROS_baxter_core_msgs_HeadState_h
#define _ROS_baxter_core_msgs_HeadState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_core_msgs
{

  class HeadState : public ros::Msg
  {
    public:
      typedef float _pan_type;
      _pan_type pan;
      typedef bool _isTurning_type;
      _isTurning_type isTurning;
      typedef bool _isNodding_type;
      _isNodding_type isNodding;
      typedef bool _isPanEnabled_type;
      _isPanEnabled_type isPanEnabled;

    HeadState():
      pan(0),
      isTurning(0),
      isNodding(0),
      isPanEnabled(0)
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
      } u_isNodding;
      u_isNodding.real = this->isNodding;
      *(outbuffer + offset + 0) = (u_isNodding.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isNodding);
      union {
        bool real;
        uint8_t base;
      } u_isPanEnabled;
      u_isPanEnabled.real = this->isPanEnabled;
      *(outbuffer + offset + 0) = (u_isPanEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isPanEnabled);
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
      } u_isNodding;
      u_isNodding.base = 0;
      u_isNodding.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isNodding = u_isNodding.real;
      offset += sizeof(this->isNodding);
      union {
        bool real;
        uint8_t base;
      } u_isPanEnabled;
      u_isPanEnabled.base = 0;
      u_isPanEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isPanEnabled = u_isPanEnabled.real;
      offset += sizeof(this->isPanEnabled);
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/HeadState"; };
    const char * getMD5(){ return "71c43b264307205358e7e49be5601348"; };

  };

}
#endif
