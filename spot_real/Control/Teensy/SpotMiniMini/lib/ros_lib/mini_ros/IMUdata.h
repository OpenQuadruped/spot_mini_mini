#ifndef _ROS_mini_ros_IMUdata_h
#define _ROS_mini_ros_IMUdata_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

  class IMUdata : public ros::Msg
  {
    public:
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _acc_x_type;
      _acc_x_type acc_x;
      typedef float _acc_y_type;
      _acc_y_type acc_y;
      typedef float _acc_z_type;
      _acc_z_type acc_z;
      typedef float _gyro_x_type;
      _gyro_x_type gyro_x;
      typedef float _gyro_y_type;
      _gyro_y_type gyro_y;
      typedef float _gyro_z_type;
      _gyro_z_type gyro_z;

    IMUdata():
      roll(0),
      pitch(0),
      acc_x(0),
      acc_y(0),
      acc_z(0),
      gyro_x(0),
      gyro_y(0),
      gyro_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.real = this->roll;
      *(outbuffer + offset + 0) = (u_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.real = this->pitch;
      *(outbuffer + offset + 0) = (u_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_acc_x;
      u_acc_x.real = this->acc_x;
      *(outbuffer + offset + 0) = (u_acc_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_x);
      union {
        float real;
        uint32_t base;
      } u_acc_y;
      u_acc_y.real = this->acc_y;
      *(outbuffer + offset + 0) = (u_acc_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_y);
      union {
        float real;
        uint32_t base;
      } u_acc_z;
      u_acc_z.real = this->acc_z;
      *(outbuffer + offset + 0) = (u_acc_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acc_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_acc_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_acc_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->acc_z);
      union {
        float real;
        uint32_t base;
      } u_gyro_x;
      u_gyro_x.real = this->gyro_x;
      *(outbuffer + offset + 0) = (u_gyro_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_x);
      union {
        float real;
        uint32_t base;
      } u_gyro_y;
      u_gyro_y.real = this->gyro_y;
      *(outbuffer + offset + 0) = (u_gyro_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_y);
      union {
        float real;
        uint32_t base;
      } u_gyro_z;
      u_gyro_z.real = this->gyro_z;
      *(outbuffer + offset + 0) = (u_gyro_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gyro_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gyro_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gyro_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gyro_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_roll;
      u_roll.base = 0;
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll = u_roll.real;
      offset += sizeof(this->roll);
      union {
        float real;
        uint32_t base;
      } u_pitch;
      u_pitch.base = 0;
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch = u_pitch.real;
      offset += sizeof(this->pitch);
      union {
        float real;
        uint32_t base;
      } u_acc_x;
      u_acc_x.base = 0;
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_x = u_acc_x.real;
      offset += sizeof(this->acc_x);
      union {
        float real;
        uint32_t base;
      } u_acc_y;
      u_acc_y.base = 0;
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_y = u_acc_y.real;
      offset += sizeof(this->acc_y);
      union {
        float real;
        uint32_t base;
      } u_acc_z;
      u_acc_z.base = 0;
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_acc_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->acc_z = u_acc_z.real;
      offset += sizeof(this->acc_z);
      union {
        float real;
        uint32_t base;
      } u_gyro_x;
      u_gyro_x.base = 0;
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_x = u_gyro_x.real;
      offset += sizeof(this->gyro_x);
      union {
        float real;
        uint32_t base;
      } u_gyro_y;
      u_gyro_y.base = 0;
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_y = u_gyro_y.real;
      offset += sizeof(this->gyro_y);
      union {
        float real;
        uint32_t base;
      } u_gyro_z;
      u_gyro_z.base = 0;
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gyro_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gyro_z = u_gyro_z.real;
      offset += sizeof(this->gyro_z);
     return offset;
    }

    const char * getType(){ return "mini_ros/IMUdata"; };
    const char * getMD5(){ return "3a27cb61fd87ce5110e1d6a4a68a0126"; };

  };

}
#endif
