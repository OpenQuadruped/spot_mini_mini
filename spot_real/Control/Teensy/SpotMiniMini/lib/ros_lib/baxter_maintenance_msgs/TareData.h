#ifndef _ROS_baxter_maintenance_msgs_TareData_h
#define _ROS_baxter_maintenance_msgs_TareData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_maintenance_msgs
{

  class TareData : public ros::Msg
  {
    public:
      typedef bool _tuneGravitySpring_type;
      _tuneGravitySpring_type tuneGravitySpring;

    TareData():
      tuneGravitySpring(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_tuneGravitySpring;
      u_tuneGravitySpring.real = this->tuneGravitySpring;
      *(outbuffer + offset + 0) = (u_tuneGravitySpring.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->tuneGravitySpring);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_tuneGravitySpring;
      u_tuneGravitySpring.base = 0;
      u_tuneGravitySpring.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->tuneGravitySpring = u_tuneGravitySpring.real;
      offset += sizeof(this->tuneGravitySpring);
     return offset;
    }

    const char * getType(){ return "baxter_maintenance_msgs/TareData"; };
    const char * getMD5(){ return "241e9c2ceee7da9db50693da0b3f2741"; };

  };

}
#endif
