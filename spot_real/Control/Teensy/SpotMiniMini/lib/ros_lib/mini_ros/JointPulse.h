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
      typedef int8_t _servo_pulse_type;
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
        int8_t real;
        uint8_t base;
      } u_servo_pulse;
      u_servo_pulse.real = this->servo_pulse;
      *(outbuffer + offset + 0) = (u_servo_pulse.base >> (8 * 0)) & 0xFF;
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
        int8_t real;
        uint8_t base;
      } u_servo_pulse;
      u_servo_pulse.base = 0;
      u_servo_pulse.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->servo_pulse = u_servo_pulse.real;
      offset += sizeof(this->servo_pulse);
     return offset;
    }

    const char * getType(){ return "mini_ros/JointPulse"; };
    const char * getMD5(){ return "42dfa34047444088cca83fbace0272c8"; };

  };

}
#endif
