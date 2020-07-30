#ifndef _ROS_baxter_maintenance_msgs_CalibrateArmEnable_h
#define _ROS_baxter_maintenance_msgs_CalibrateArmEnable_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "baxter_maintenance_msgs/CalibrateArmData.h"

namespace baxter_maintenance_msgs
{

  class CalibrateArmEnable : public ros::Msg
  {
    public:
      typedef bool _isEnabled_type;
      _isEnabled_type isEnabled;
      typedef const char* _uid_type;
      _uid_type uid;
      typedef baxter_maintenance_msgs::CalibrateArmData _data_type;
      _data_type data;

    CalibrateArmEnable():
      isEnabled(0),
      uid(""),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isEnabled;
      u_isEnabled.real = this->isEnabled;
      *(outbuffer + offset + 0) = (u_isEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isEnabled);
      uint32_t length_uid = strlen(this->uid);
      varToArr(outbuffer + offset, length_uid);
      offset += 4;
      memcpy(outbuffer + offset, this->uid, length_uid);
      offset += length_uid;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_isEnabled;
      u_isEnabled.base = 0;
      u_isEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isEnabled = u_isEnabled.real;
      offset += sizeof(this->isEnabled);
      uint32_t length_uid;
      arrToVar(length_uid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uid-1]=0;
      this->uid = (char *)(inbuffer + offset-1);
      offset += length_uid;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "baxter_maintenance_msgs/CalibrateArmEnable"; };
    const char * getMD5(){ return "d7ff300fd410d4ac849664ab8143da39"; };

  };

}
#endif
