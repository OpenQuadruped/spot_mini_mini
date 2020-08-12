#ifndef _ROS_mini_ros_JointPulse_h
#define _ROS_mini_ros_JointPulse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

  class JointPulse : public ros::Msg
  {
    public:
      typedef int8_t _servo_num_type;
      _servo_num_type servo_num;
      typedef int32_t _servo_pulse_type;
      _servo_pulse_type servo_pulse;

    JointPulse():
      servo_num(0),
      servo_pulse(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_servo_num;
      u_servo_num.real = this->servo_num;
      *(outbuffer + offset + 0) = (u_servo_num.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->servo_num);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_pulse;
      u_servo_pulse.real = this->servo_pulse;
      *(outbuffer + offset + 0) = (u_servo_pulse.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_pulse.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_pulse.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_pulse.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_pulse);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_servo_num;
      u_servo_num.base = 0;
      u_servo_num.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->servo_num = u_servo_num.real;
      offset += sizeof(this->servo_num);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_pulse;
      u_servo_pulse.base = 0;
      u_servo_pulse.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_pulse.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_pulse.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_pulse.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_pulse = u_servo_pulse.real;
      offset += sizeof(this->servo_pulse);
     return offset;
    }

    const char * getType(){ return "mini_ros/JointPulse"; };
    const char * getMD5(){ return "372c64510294fc8eec78b728b048d2c9"; };

  };

}
#endif
