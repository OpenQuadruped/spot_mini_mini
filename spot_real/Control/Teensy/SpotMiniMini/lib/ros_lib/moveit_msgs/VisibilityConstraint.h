#ifndef _ROS_moveit_msgs_VisibilityConstraint_h
#define _ROS_moveit_msgs_VisibilityConstraint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace moveit_msgs
{

  class VisibilityConstraint : public ros::Msg
  {
    public:
      typedef double _target_radius_type;
      _target_radius_type target_radius;
      typedef geometry_msgs::PoseStamped _target_pose_type;
      _target_pose_type target_pose;
      typedef int32_t _cone_sides_type;
      _cone_sides_type cone_sides;
      typedef geometry_msgs::PoseStamped _sensor_pose_type;
      _sensor_pose_type sensor_pose;
      typedef double _max_view_angle_type;
      _max_view_angle_type max_view_angle;
      typedef double _max_range_angle_type;
      _max_range_angle_type max_range_angle;
      typedef uint8_t _sensor_view_direction_type;
      _sensor_view_direction_type sensor_view_direction;
      typedef double _weight_type;
      _weight_type weight;
      enum { SENSOR_Z = 0 };
      enum { SENSOR_Y = 1 };
      enum { SENSOR_X = 2 };

    VisibilityConstraint():
      target_radius(0),
      target_pose(),
      cone_sides(0),
      sensor_pose(),
      max_view_angle(0),
      max_range_angle(0),
      sensor_view_direction(0),
      weight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_target_radius;
      u_target_radius.real = this->target_radius;
      *(outbuffer + offset + 0) = (u_target_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_target_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_target_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_target_radius.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_target_radius.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_target_radius.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_target_radius.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_target_radius.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->target_radius);
      offset += this->target_pose.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_cone_sides;
      u_cone_sides.real = this->cone_sides;
      *(outbuffer + offset + 0) = (u_cone_sides.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cone_sides.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cone_sides.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cone_sides.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cone_sides);
      offset += this->sensor_pose.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_max_view_angle;
      u_max_view_angle.real = this->max_view_angle;
      *(outbuffer + offset + 0) = (u_max_view_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_view_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_view_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_view_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_view_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_view_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_view_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_view_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_view_angle);
      union {
        double real;
        uint64_t base;
      } u_max_range_angle;
      u_max_range_angle.real = this->max_range_angle;
      *(outbuffer + offset + 0) = (u_max_range_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_range_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_range_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_range_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_range_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_range_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_range_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_range_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_range_angle);
      *(outbuffer + offset + 0) = (this->sensor_view_direction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sensor_view_direction);
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
      union {
        double real;
        uint64_t base;
      } u_target_radius;
      u_target_radius.base = 0;
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_target_radius.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->target_radius = u_target_radius.real;
      offset += sizeof(this->target_radius);
      offset += this->target_pose.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_cone_sides;
      u_cone_sides.base = 0;
      u_cone_sides.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cone_sides.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cone_sides.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cone_sides.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cone_sides = u_cone_sides.real;
      offset += sizeof(this->cone_sides);
      offset += this->sensor_pose.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_max_view_angle;
      u_max_view_angle.base = 0;
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_view_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_view_angle = u_max_view_angle.real;
      offset += sizeof(this->max_view_angle);
      union {
        double real;
        uint64_t base;
      } u_max_range_angle;
      u_max_range_angle.base = 0;
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_range_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_range_angle = u_max_range_angle.real;
      offset += sizeof(this->max_range_angle);
      this->sensor_view_direction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sensor_view_direction);
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

    const char * getType(){ return "moveit_msgs/VisibilityConstraint"; };
    const char * getMD5(){ return "62cda903bfe31ff2e5fcdc3810d577ad"; };

  };

}
#endif
