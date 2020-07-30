#ifndef _ROS_intera_core_msgs_HeadPanCommand_h
#define _ROS_intera_core_msgs_HeadPanCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class HeadPanCommand : public ros::Msg
  {
    public:
      typedef float _target_type;
      _target_type target;
      typedef float _speed_ratio_type;
      _speed_ratio_type speed_ratio;
      typedef uint8_t _pan_mode_type;
      _pan_mode_type pan_mode;
      enum { MAX_SPEED_RATIO =  1.0 };
      enum { MIN_SPEED_RATIO =  0.0 };
      enum { SET_PASSIVE_MODE =  0 };
      enum { SET_ACTIVE_MODE =  1 };
      enum { SET_ACTIVE_CANCELLATION_MODE =  2 };
      enum { NO_MODE_CHANGE =  3 };

    HeadPanCommand():
      target(0),
      speed_ratio(0),
      pan_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target;
      u_target.real = this->target;
      *(outbuffer + offset + 0) = (u_target.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->target);
      union {
        float real;
        uint32_t base;
      } u_speed_ratio;
      u_speed_ratio.real = this->speed_ratio;
      *(outbuffer + offset + 0) = (u_speed_ratio.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_ratio.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_ratio.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_ratio.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_ratio);
      *(outbuffer + offset + 0) = (this->pan_mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pan_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_target;
      u_target.base = 0;
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->target = u_target.real;
      offset += sizeof(this->target);
      union {
        float real;
        uint32_t base;
      } u_speed_ratio;
      u_speed_ratio.base = 0;
      u_speed_ratio.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_ratio.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_ratio.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_ratio.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_ratio = u_speed_ratio.real;
      offset += sizeof(this->speed_ratio);
      this->pan_mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pan_mode);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/HeadPanCommand"; };
    const char * getMD5(){ return "5cb68e8755646564cf47813f91cee216"; };

  };

}
#endif
