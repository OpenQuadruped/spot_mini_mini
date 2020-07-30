#ifndef _ROS_intera_core_msgs_URDFConfiguration_h
#define _ROS_intera_core_msgs_URDFConfiguration_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace intera_core_msgs
{

  class URDFConfiguration : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef const char* _link_type;
      _link_type link;
      typedef const char* _joint_type;
      _joint_type joint;
      typedef const char* _urdf_type;
      _urdf_type urdf;

    URDFConfiguration():
      time(),
      link(""),
      joint(""),
      urdf("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
      uint32_t length_link = strlen(this->link);
      varToArr(outbuffer + offset, length_link);
      offset += 4;
      memcpy(outbuffer + offset, this->link, length_link);
      offset += length_link;
      uint32_t length_joint = strlen(this->joint);
      varToArr(outbuffer + offset, length_joint);
      offset += 4;
      memcpy(outbuffer + offset, this->joint, length_joint);
      offset += length_joint;
      uint32_t length_urdf = strlen(this->urdf);
      varToArr(outbuffer + offset, length_urdf);
      offset += 4;
      memcpy(outbuffer + offset, this->urdf, length_urdf);
      offset += length_urdf;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
      uint32_t length_link;
      arrToVar(length_link, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_link; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_link-1]=0;
      this->link = (char *)(inbuffer + offset-1);
      offset += length_link;
      uint32_t length_joint;
      arrToVar(length_joint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_joint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_joint-1]=0;
      this->joint = (char *)(inbuffer + offset-1);
      offset += length_joint;
      uint32_t length_urdf;
      arrToVar(length_urdf, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_urdf; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_urdf-1]=0;
      this->urdf = (char *)(inbuffer + offset-1);
      offset += length_urdf;
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/URDFConfiguration"; };
    const char * getMD5(){ return "0c7028d878027820eed2aa0cbf1f5e4a"; };

  };

}
#endif
