#ifndef _ROS_moveit_msgs_CostSource_h
#define _ROS_moveit_msgs_CostSource_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace moveit_msgs
{

  class CostSource : public ros::Msg
  {
    public:
      typedef double _cost_density_type;
      _cost_density_type cost_density;
      typedef geometry_msgs::Vector3 _aabb_min_type;
      _aabb_min_type aabb_min;
      typedef geometry_msgs::Vector3 _aabb_max_type;
      _aabb_max_type aabb_max;

    CostSource():
      cost_density(0),
      aabb_min(),
      aabb_max()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_cost_density;
      u_cost_density.real = this->cost_density;
      *(outbuffer + offset + 0) = (u_cost_density.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cost_density.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cost_density.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cost_density.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_cost_density.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_cost_density.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_cost_density.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_cost_density.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->cost_density);
      offset += this->aabb_min.serialize(outbuffer + offset);
      offset += this->aabb_max.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_cost_density;
      u_cost_density.base = 0;
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_cost_density.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->cost_density = u_cost_density.real;
      offset += sizeof(this->cost_density);
      offset += this->aabb_min.deserialize(inbuffer + offset);
      offset += this->aabb_max.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "moveit_msgs/CostSource"; };
    const char * getMD5(){ return "abb7e013237dacaaa8b97e704102f908"; };

  };

}
#endif
