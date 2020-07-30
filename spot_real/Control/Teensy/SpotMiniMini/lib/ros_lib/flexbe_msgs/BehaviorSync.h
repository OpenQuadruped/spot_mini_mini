#ifndef _ROS_flexbe_msgs_BehaviorSync_h
#define _ROS_flexbe_msgs_BehaviorSync_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class BehaviorSync : public ros::Msg
  {
    public:
      typedef int32_t _behavior_id_type;
      _behavior_id_type behavior_id;
      typedef int32_t _current_state_checksum_type;
      _current_state_checksum_type current_state_checksum;

    BehaviorSync():
      behavior_id(0),
      current_state_checksum(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_id;
      u_behavior_id.real = this->behavior_id;
      *(outbuffer + offset + 0) = (u_behavior_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_behavior_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_behavior_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_behavior_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->behavior_id);
      union {
        int32_t real;
        uint32_t base;
      } u_current_state_checksum;
      u_current_state_checksum.real = this->current_state_checksum;
      *(outbuffer + offset + 0) = (u_current_state_checksum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_state_checksum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_state_checksum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_state_checksum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_state_checksum);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_behavior_id;
      u_behavior_id.base = 0;
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_behavior_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->behavior_id = u_behavior_id.real;
      offset += sizeof(this->behavior_id);
      union {
        int32_t real;
        uint32_t base;
      } u_current_state_checksum;
      u_current_state_checksum.base = 0;
      u_current_state_checksum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_state_checksum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_state_checksum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_state_checksum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_state_checksum = u_current_state_checksum.real;
      offset += sizeof(this->current_state_checksum);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorSync"; };
    const char * getMD5(){ return "118d64f48696509906c4cf4a228608b1"; };

  };

}
#endif
