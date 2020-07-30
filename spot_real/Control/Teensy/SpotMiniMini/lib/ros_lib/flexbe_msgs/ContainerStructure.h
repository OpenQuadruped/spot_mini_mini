#ifndef _ROS_flexbe_msgs_ContainerStructure_h
#define _ROS_flexbe_msgs_ContainerStructure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/Container.h"

namespace flexbe_msgs
{

  class ContainerStructure : public ros::Msg
  {
    public:
      typedef int32_t _behavior_id_type;
      _behavior_id_type behavior_id;
      uint32_t containers_length;
      typedef flexbe_msgs::Container _containers_type;
      _containers_type st_containers;
      _containers_type * containers;

    ContainerStructure():
      behavior_id(0),
      containers_length(0), containers(NULL)
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
      *(outbuffer + offset + 0) = (this->containers_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->containers_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->containers_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->containers_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->containers_length);
      for( uint32_t i = 0; i < containers_length; i++){
      offset += this->containers[i].serialize(outbuffer + offset);
      }
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
      uint32_t containers_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      containers_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      containers_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      containers_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->containers_length);
      if(containers_lengthT > containers_length)
        this->containers = (flexbe_msgs::Container*)realloc(this->containers, containers_lengthT * sizeof(flexbe_msgs::Container));
      containers_length = containers_lengthT;
      for( uint32_t i = 0; i < containers_length; i++){
      offset += this->st_containers.deserialize(inbuffer + offset);
        memcpy( &(this->containers[i]), &(this->st_containers), sizeof(flexbe_msgs::Container));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/ContainerStructure"; };
    const char * getMD5(){ return "9cd26d15621063b90279d87117694328"; };

  };

}
#endif
