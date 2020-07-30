#ifndef _ROS_SERVICE_CalibServo_h
#define _ROS_SERVICE_CalibServo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

static const char CALIBSERVO[] = "mini_ros/CalibServo";

  class CalibServoRequest : public ros::Msg
  {
    public:
      typedef int8_t _servo_num_type;
      _servo_num_type servo_num;
      typedef float _servo_angle_type;
      _servo_angle_type servo_angle;

    CalibServoRequest():
      servo_num(0),
      servo_angle(0)
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
        float real;
        uint32_t base;
      } u_servo_angle;
      u_servo_angle.real = this->servo_angle;
      *(outbuffer + offset + 0) = (u_servo_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_angle);
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
        float real;
        uint32_t base;
      } u_servo_angle;
      u_servo_angle.base = 0;
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_angle = u_servo_angle.real;
      offset += sizeof(this->servo_angle);
     return offset;
    }

    const char * getType(){ return CALIBSERVO; };
    const char * getMD5(){ return "7aff9fa3a5043af717272f94b89deff9"; };

  };

  class CalibServoResponse : public ros::Msg
  {
    public:
      typedef const char* _Response_type;
      _Response_type Response;

    CalibServoResponse():
      Response("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_Response = strlen(this->Response);
      varToArr(outbuffer + offset, length_Response);
      offset += 4;
      memcpy(outbuffer + offset, this->Response, length_Response);
      offset += length_Response;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_Response;
      arrToVar(length_Response, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Response-1]=0;
      this->Response = (char *)(inbuffer + offset-1);
      offset += length_Response;
     return offset;
    }

    const char * getType(){ return CALIBSERVO; };
    const char * getMD5(){ return "e9ca8778f2b24ad03f8213b9fe82be44"; };

  };

  class CalibServo {
    public:
    typedef CalibServoRequest Request;
    typedef CalibServoResponse Response;
  };

}
#endif
