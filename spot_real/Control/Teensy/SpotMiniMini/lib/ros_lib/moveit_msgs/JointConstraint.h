#ifndef _ROS_moveit_msgs_JointConstraint_h
#define _ROS_moveit_msgs_JointConstraint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moveit_msgs
{

  class JointConstraint : public ros::Msg
  {
    public:
      typedef const char* _joint_name_type;
      _joint_name_type joint_name;
      typedef double _position_type;
      _position_type position;
      typedef double _tolerance_above_type;
      _tolerance_above_type tolerance_above;
      typedef double _tolerance_below_type;
      _tolerance_below_type tolerance_below;
      typedef double _weight_type;
      _weight_type weight;

    JointConstraint():
      joint_name(""),
      position(0),
      tolerance_above(0),
      tolerance_below(0),
      weight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_joint_name = strlen(this->joint_name);
      varToArr(outbuffer + offset, length_joint_name);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name, length_joint_name);
      offset += length_joint_name;
      union {
        double real;
        uint64_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_position.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_position.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_position.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_position.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position);
      union {
        double real;
        uint64_t base;
      } u_tolerance_above;
      u_tolerance_above.real = this->tolerance_above;
      *(outbuffer + offset + 0) = (u_tolerance_above.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerance_above.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerance_above.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerance_above.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tolerance_above.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tolerance_above.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tolerance_above.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tolerance_above.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tolerance_above);
      union {
        double real;
        uint64_t base;
      } u_tolerance_below;
      u_tolerance_below.real = this->tolerance_below;
      *(outbuffer + offset + 0) = (u_tolerance_below.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerance_below.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerance_below.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerance_below.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tolerance_below.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tolerance_below.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tolerance_below.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tolerance_below.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tolerance_below);
      union {
        double real;
        uint64_t base;
      } u_weight;
      u_weight.real = this->weight;
      *(outbuffer + offset + 0) = (u_weight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_weight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_weight.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_weight.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_weight.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_weight.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_weight.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->weight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_joint_name;
      arrToVar(length_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint_name-1]=0;
      this->joint_name = (char *)(inbuffer + offset-1);
      offset += length_joint_name;
      union {
        double real;
        uint64_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        double real;
        uint64_t base;
      } u_tolerance_above;
      u_tolerance_above.base = 0;
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tolerance_above.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tolerance_above = u_tolerance_above.real;
      offset += sizeof(this->tolerance_above);
      union {
        double real;
        uint64_t base;
      } u_tolerance_below;
      u_tolerance_below.base = 0;
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tolerance_below.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tolerance_below = u_tolerance_below.real;
      offset += sizeof(this->tolerance_below);
      union {
        double real;
        uint64_t base;
      } u_weight;
      u_weight.base = 0;
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_weight.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->weight = u_weight.real;
      offset += sizeof(this->weight);
     return offset;
    }

    const char * getType(){ return "moveit_msgs/JointConstraint"; };
    const char * getMD5(){ return "c02a15146bec0ce13564807805b008f0"; };

  };

}
#endif
