#ifndef _ROS_mini_ros_ContactData_h
#define _ROS_mini_ros_ContactData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

  class ContactData : public ros::Msg
  {
    public:
      typedef bool _FL_type;
      _FL_type FL;
      typedef bool _FR_type;
      _FR_type FR;
      typedef bool _BL_type;
      _BL_type BL;
      typedef bool _BR_type;
      _BR_type BR;

    ContactData():
      FL(0),
      FR(0),
      BL(0),
      BR(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_FL;
      u_FL.real = this->FL;
      *(outbuffer + offset + 0) = (u_FL.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->FL);
      union {
        bool real;
        uint8_t base;
      } u_FR;
      u_FR.real = this->FR;
      *(outbuffer + offset + 0) = (u_FR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->FR);
      union {
        bool real;
        uint8_t base;
      } u_BL;
      u_BL.real = this->BL;
      *(outbuffer + offset + 0) = (u_BL.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BL);
      union {
        bool real;
        uint8_t base;
      } u_BR;
      u_BR.real = this->BR;
      *(outbuffer + offset + 0) = (u_BR.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BR);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_FL;
      u_FL.base = 0;
      u_FL.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->FL = u_FL.real;
      offset += sizeof(this->FL);
      union {
        bool real;
        uint8_t base;
      } u_FR;
      u_FR.base = 0;
      u_FR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->FR = u_FR.real;
      offset += sizeof(this->FR);
      union {
        bool real;
        uint8_t base;
      } u_BL;
      u_BL.base = 0;
      u_BL.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->BL = u_BL.real;
      offset += sizeof(this->BL);
      union {
        bool real;
        uint8_t base;
      } u_BR;
      u_BR.base = 0;
      u_BR.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->BR = u_BR.real;
      offset += sizeof(this->BR);
     return offset;
    }

    const char * getType(){ return "mini_ros/ContactData"; };
    const char * getMD5(){ return "e1e8fe4de4334f7698a7f305ee06dce8"; };

  };

}
#endif
