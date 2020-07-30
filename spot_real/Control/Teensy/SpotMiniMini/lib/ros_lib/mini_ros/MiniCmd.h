#ifndef _ROS_mini_ros_MiniCmd_h
#define _ROS_mini_ros_MiniCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

  class MiniCmd : public ros::Msg
  {
    public:
      typedef const char* _motion_type;
      _motion_type motion;
      typedef const char* _movement_type;
      _movement_type movement;
      typedef float _x_velocity_type;
      _x_velocity_type x_velocity;
      typedef float _y_velocity_type;
      _y_velocity_type y_velocity;
      typedef float _rate_type;
      _rate_type rate;
      typedef float _roll_type;
      _roll_type roll;
      typedef float _pitch_type;
      _pitch_type pitch;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _z_type;
      _z_type z;
      typedef float _faster_type;
      _faster_type faster;
      typedef float _slower_type;
      _slower_type slower;

    MiniCmd():
      motion(""),
      movement(""),
      x_velocity(0),
      y_velocity(0),
      rate(0),
      roll(0),
      pitch(0),
      yaw(0),
      z(0),
      faster(0),
      slower(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_motion = strlen(this->motion);
      varToArr(outbuffer + offset, length_motion);
      offset += 4;
      memcpy(outbuffer + offset, this->motion, length_motion);
      offset += length_motion;
      uint32_t length_movement = strlen(this->movement);
      varToArr(outbuffer + offset, length_movement);
      offset += 4;
      memcpy(outbuffer + offset, this->movement, length_movement);
      offset += length_movement;
      union {
        float real;
        uint32_t base;
      } u_x_velocity;
      u_x_velocity.real = this->x_velocity;
      *(outbuffer + offset + 0) = (u_x_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_velocity);
      union {
        float real;
        uint32_t base;
      } u_y_velocity;
      u_y_velocity.real = this->y_velocity;
      *(outbuffer + offset + 0) = (u_y_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_velocity);
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.real = this->rate;
      *(outbuffer + offset + 0) = (u_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rate);
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
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_faster;
      u_faster.real = this->faster;
      *(outbuffer + offset + 0) = (u_faster.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_faster.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_faster.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_faster.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->faster);
      union {
        float real;
        uint32_t base;
      } u_slower;
      u_slower.real = this->slower;
      *(outbuffer + offset + 0) = (u_slower.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_slower.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_slower.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_slower.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->slower);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_motion;
      arrToVar(length_motion, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motion; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motion-1]=0;
      this->motion = (char *)(inbuffer + offset-1);
      offset += length_motion;
      uint32_t length_movement;
      arrToVar(length_movement, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_movement; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_movement-1]=0;
      this->movement = (char *)(inbuffer + offset-1);
      offset += length_movement;
      union {
        float real;
        uint32_t base;
      } u_x_velocity;
      u_x_velocity.base = 0;
      u_x_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_velocity = u_x_velocity.real;
      offset += sizeof(this->x_velocity);
      union {
        float real;
        uint32_t base;
      } u_y_velocity;
      u_y_velocity.base = 0;
      u_y_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_velocity = u_y_velocity.real;
      offset += sizeof(this->y_velocity);
      union {
        float real;
        uint32_t base;
      } u_rate;
      u_rate.base = 0;
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rate = u_rate.real;
      offset += sizeof(this->rate);
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
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_faster;
      u_faster.base = 0;
      u_faster.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_faster.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_faster.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_faster.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->faster = u_faster.real;
      offset += sizeof(this->faster);
      union {
        float real;
        uint32_t base;
      } u_slower;
      u_slower.base = 0;
      u_slower.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_slower.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_slower.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_slower.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->slower = u_slower.real;
      offset += sizeof(this->slower);
     return offset;
    }

    const char * getType(){ return "mini_ros/MiniCmd"; };
    const char * getMD5(){ return "83510b9a5f454bd3ceb898b52e349406"; };

  };

}
#endif
