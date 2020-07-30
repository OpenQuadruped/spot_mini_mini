#ifndef _ROS_baxter_maintenance_msgs_CalibrateArmData_h
#define _ROS_baxter_maintenance_msgs_CalibrateArmData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_maintenance_msgs
{

  class CalibrateArmData : public ros::Msg
  {
    public:
      typedef bool _suppressWriteToFile_type;
      _suppressWriteToFile_type suppressWriteToFile;

    CalibrateArmData():
      suppressWriteToFile(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_suppressWriteToFile;
      u_suppressWriteToFile.real = this->suppressWriteToFile;
      *(outbuffer + offset + 0) = (u_suppressWriteToFile.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->suppressWriteToFile);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_suppressWriteToFile;
      u_suppressWriteToFile.base = 0;
      u_suppressWriteToFile.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->suppressWriteToFile = u_suppressWriteToFile.real;
      offset += sizeof(this->suppressWriteToFile);
     return offset;
    }

    const char * getType(){ return "baxter_maintenance_msgs/CalibrateArmData"; };
    const char * getMD5(){ return "ba9ee949ea363f7bcfc8cc74e0bcb69d"; };

  };

}
#endif
