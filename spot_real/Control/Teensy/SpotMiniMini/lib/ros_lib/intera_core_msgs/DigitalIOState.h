#ifndef _ROS_intera_core_msgs_DigitalIOState_h
#define _ROS_intera_core_msgs_DigitalIOState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class DigitalIOState : public ros::Msg
  {
    public:
      typedef int8_t _state_type;
      _state_type state;
      typedef bool _isInputOnly_type;
      _isInputOnly_type isInputOnly;
      enum { OFF =  0 };
      enum { ON =  1 };
      enum { PRESSED =  1 };
      enum { UNPRESSED =  0 };

    DigitalIOState():
      state(0),
      isInputOnly(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_isInputOnly;
      u_isInputOnly.real = this->isInputOnly;
      *(outbuffer + offset + 0) = (u_isInputOnly.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isInputOnly);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
      union {
        bool real;
        uint8_t base;
      } u_isInputOnly;
      u_isInputOnly.base = 0;
      u_isInputOnly.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isInputOnly = u_isInputOnly.real;
      offset += sizeof(this->isInputOnly);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/DigitalIOState"; };
    const char * getMD5(){ return "29d0be3859dae81a66b28f167ecec98c"; };

  };

}
#endif
