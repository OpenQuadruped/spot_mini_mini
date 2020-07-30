#ifndef _ROS_intera_core_msgs_CollisionDetectionState_h
#define _ROS_intera_core_msgs_CollisionDetectionState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace intera_core_msgs
{

  class CollisionDetectionState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _collision_state_type;
      _collision_state_type collision_state;

    CollisionDetectionState():
      header(),
      collision_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_collision_state;
      u_collision_state.real = this->collision_state;
      *(outbuffer + offset + 0) = (u_collision_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->collision_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_collision_state;
      u_collision_state.base = 0;
      u_collision_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->collision_state = u_collision_state.real;
      offset += sizeof(this->collision_state);
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/CollisionDetectionState"; };
    const char * getMD5(){ return "7bde38c182b4d08fdc0635b116f65d04"; };

  };

}
#endif
