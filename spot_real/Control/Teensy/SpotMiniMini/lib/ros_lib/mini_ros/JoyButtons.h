#ifndef _ROS_mini_ros_JoyButtons_h
#define _ROS_mini_ros_JoyButtons_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

  class JoyButtons : public ros::Msg
  {
    public:
      typedef int8_t _updown_type;
      _updown_type updown;
      typedef int8_t _leftright_type;
      _leftright_type leftright;
      typedef int8_t _left_bump_type;
      _left_bump_type left_bump;
      typedef int8_t _right_bump_type;
      _right_bump_type right_bump;

    JoyButtons():
      updown(0),
      leftright(0),
      left_bump(0),
      right_bump(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_updown;
      u_updown.real = this->updown;
      *(outbuffer + offset + 0) = (u_updown.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->updown);
      union {
        int8_t real;
        uint8_t base;
      } u_leftright;
      u_leftright.real = this->leftright;
      *(outbuffer + offset + 0) = (u_leftright.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->leftright);
      union {
        int8_t real;
        uint8_t base;
      } u_left_bump;
      u_left_bump.real = this->left_bump;
      *(outbuffer + offset + 0) = (u_left_bump.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_bump);
      union {
        int8_t real;
        uint8_t base;
      } u_right_bump;
      u_right_bump.real = this->right_bump;
      *(outbuffer + offset + 0) = (u_right_bump.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_bump);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_updown;
      u_updown.base = 0;
      u_updown.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->updown = u_updown.real;
      offset += sizeof(this->updown);
      union {
        int8_t real;
        uint8_t base;
      } u_leftright;
      u_leftright.base = 0;
      u_leftright.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->leftright = u_leftright.real;
      offset += sizeof(this->leftright);
      union {
        int8_t real;
        uint8_t base;
      } u_left_bump;
      u_left_bump.base = 0;
      u_left_bump.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->left_bump = u_left_bump.real;
      offset += sizeof(this->left_bump);
      union {
        int8_t real;
        uint8_t base;
      } u_right_bump;
      u_right_bump.base = 0;
      u_right_bump.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->right_bump = u_right_bump.real;
      offset += sizeof(this->right_bump);
     return offset;
    }

    const char * getType(){ return "mini_ros/JoyButtons"; };
    const char * getMD5(){ return "be1de48c3b52ec87587be0e78c2cb8cd"; };

  };

}
#endif
