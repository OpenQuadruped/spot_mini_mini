#ifndef _ROS_intera_core_msgs_CollisionAvoidanceState_h
#define _ROS_intera_core_msgs_CollisionAvoidanceState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace intera_core_msgs
{

  class CollisionAvoidanceState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _other_arm_type;
      _other_arm_type other_arm;
      uint32_t collision_object_length;
      typedef char* _collision_object_type;
      _collision_object_type st_collision_object;
      _collision_object_type * collision_object;

    CollisionAvoidanceState():
      header(),
      other_arm(0),
      collision_object_length(0), collision_object(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_other_arm;
      u_other_arm.real = this->other_arm;
      *(outbuffer + offset + 0) = (u_other_arm.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->other_arm);
      *(outbuffer + offset + 0) = (this->collision_object_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->collision_object_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->collision_object_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->collision_object_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->collision_object_length);
      for( uint32_t i = 0; i < collision_object_length; i++){
      uint32_t length_collision_objecti = strlen(this->collision_object[i]);
      varToArr(outbuffer + offset, length_collision_objecti);
      offset += 4;
      memcpy(outbuffer + offset, this->collision_object[i], length_collision_objecti);
      offset += length_collision_objecti;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_other_arm;
      u_other_arm.base = 0;
      u_other_arm.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->other_arm = u_other_arm.real;
      offset += sizeof(this->other_arm);
      uint32_t collision_object_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      collision_object_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      collision_object_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      collision_object_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->collision_object_length);
      if(collision_object_lengthT > collision_object_length)
        this->collision_object = (char**)realloc(this->collision_object, collision_object_lengthT * sizeof(char*));
      collision_object_length = collision_object_lengthT;
      for( uint32_t i = 0; i < collision_object_length; i++){
      uint32_t length_st_collision_object;
      arrToVar(length_st_collision_object, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_collision_object; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_collision_object-1]=0;
      this->st_collision_object = (char *)(inbuffer + offset-1);
      offset += length_st_collision_object;
        memcpy( &(this->collision_object[i]), &(this->st_collision_object), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/CollisionAvoidanceState"; };
    const char * getMD5(){ return "30f5cb8ae019f1ffe8b599e6d2e589c7"; };

  };

}
#endif
